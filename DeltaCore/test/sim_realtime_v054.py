#!/usr/bin/env python3
import random

BAUD=250000
BYTE_US=10_000_000/BAUD
RX_CAP=512
PATH=16
PENDING=64
LOW=14
TARGET=28
MAX_BURST=8
BUDGET_US=1800
SEG_US=10000
GEN_US=260
PLAN_US=650
PARSE_LINE_US=90

class Sim:
    def __init__(self, moves, seed=1, inject_m105=True):
        self.r=random.Random(seed)
        self.moves=moves
        self.inject_m105=inject_m105
        self.host=[]
        for i in range(moves):
            x=self.r.randint(-40,40); y=self.r.randint(-40,40)
            self.host.extend((f'G1 X{x} Y{y} F6000\n').encode())
            if inject_m105 and i and i%37==0:
                self.host.extend(b'M105\n')
        self.host.extend(b'M400\n')
        self.tx_idx=0
        self.rx=[]
        self.line=bytearray()
        self.path=0
        self.pending=0
        self.accepted=0
        self.executed=0
        self.motorq=0
        self.starves=0
        self.next_motor_us=None
        self.now=0.0
        self.next_byte_us=0.0
        self.closed=False
        self.max_rx=0
        self.max_pending=0

    def host_arrive_until(self,t):
        while self.tx_idx < len(self.host) and self.next_byte_us <= t:
            if len(self.rx)>=RX_CAP:
                raise AssertionError('RX overflow')
            self.rx.append(self.host[self.tx_idx]); self.tx_idx+=1
            self.next_byte_us += BYTE_US
        self.max_rx=max(self.max_rx,len(self.rx))

    def consume_motor_until(self,t):
        if self.next_motor_us is None and self.motorq:
            self.next_motor_us=self.now+SEG_US
        while self.next_motor_us is not None and self.next_motor_us<=t:
            self.motorq-=1
            if self.motorq:
                self.next_motor_us += SEG_US
            else:
                self.next_motor_us=None
                if self.executed < self.accepted or self.path or self.pending:
                    self.starves+=1

    def advance(self,us):
        t=self.now+us
        self.host_arrive_until(t); self.consume_motor_until(t)
        self.now=t

    def serial_slice(self):
        lines=0; consumed=0
        while self.rx and lines<2 and consumed<96:
            c=self.rx.pop(0); consumed+=1; self.advance(2)
            if c==13: continue
            if c==10:
                s=self.line.decode(errors='strict'); self.line.clear(); lines+=1
                self.advance(PARSE_LINE_US)
                if s.startswith('G1'):
                    if self.path < PATH: self.path+=1
                    elif self.pending < PENDING: self.pending+=1
                    else: raise AssertionError('motion ingress overflow')
                    self.accepted+=1
                elif s=='M105':
                    pass
                elif s=='M400':
                    self.closed=True
                else:
                    raise AssertionError('corrupt line '+repr(s))
            else:
                self.line.append(c)
        self.max_rx=max(self.max_rx,len(self.rx))

    def fill_planner(self):
        while self.path<PATH and self.pending:
            self.pending-=1; self.path+=1
        self.max_pending=max(self.max_pending,self.pending)

    def produce(self):
        self.fill_planner()
        urgent=self.motorq<LOW
        limit=MAX_BURST if urgent else 1
        spent=0; produced=0
        while produced<limit and self.motorq<31:
            if not self.path: break
            # Keep four lookahead moves until stream closes.
            if not self.closed and self.path<=4: break
            if spent+PLAN_US+GEN_US>BUDGET_US and produced: break
            self.advance(PLAN_US+GEN_US); spent += PLAN_US+GEN_US
            self.motorq+=1; produced+=1
            if self.next_motor_us is None and self.motorq>=24:
                self.next_motor_us=self.now+SEG_US
            # Approximate one 10ms block per committed move for scheduler stress.
            self.path-=1; self.executed+=1
            self.fill_planner()
            if self.motorq>=TARGET: break
            if self.motorq>=LOW and self.rx: break

    def run(self):
        for _ in range(5_000_000):
            self.host_arrive_until(self.now)
            self.serial_slice()
            self.produce()
            self.advance(40)
            if self.closed and self.tx_idx==len(self.host) and not self.rx and not self.line and not self.path and not self.pending:
                # Drain motor reservoir.
                while self.motorq:
                    self.advance(SEG_US)
                break
        else: raise AssertionError('dead state')
        assert self.accepted==self.moves,(self.accepted,self.moves)
        assert self.executed==self.moves,(self.executed,self.moves)
        # One empty stop at final drain is not counted as starvation by firmware PERF.
        assert self.starves==0,self.starves
        return self.max_rx,self.max_pending,self.now


def main():
    cases=[45,65,200,500,1000,2000]
    for n in cases:
        worst_rx=0; worst_p=0
        for seed in range(80):
            s=Sim(n,seed,True); mr,mp,_=s.run(); worst_rx=max(worst_rx,mr); worst_p=max(worst_p,mp)
        print(f'PASS realtime stream moves={n} seeds=80 max_rx={worst_rx}/{RX_CAP} max_pending={worst_p}/{PENDING}')
    print('PASS UART 250000 + periodic M105 + adaptive producer: no corruption/overflow/starvation')

if __name__=='__main__': main()
