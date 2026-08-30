#!/usr/bin/env python3
from collections import deque
import random

BAUD = 250000
BYTE_US = 10_000_000 / BAUD
RX_CAP = 512
LINE_CAP = 128
PATH_CAP = 16
PENDING_CAP = 46
ADMISSION_LIMIT = PATH_CAP + PENDING_CAP - 2
LOOKAHEAD_RESERVE = 4
MOTOR_CAP = 63  # 64-slot ring keeps one slot empty
LOW = 24
TARGET = 56
MAX_BURST = 12
BUDGET_US = 6000
GEN_US = 260
# Real AVR profiling on the v0.5.8 hardware candidate measured the expensive
# planner/fill path at just under 10 ms inclusive. Model that worst observed
# stall instead of the old optimistic 650 us estimate so UART capacity is
# validated against real main-loop blocking time.
PLAN_US = 10000
PARSE_LINE_US = 90
ACK_TX_US = 160
START_PREFILL = 48


def make_commands(move_count, rng, include_m105=True):
    cmds=[]
    for i in range(move_count):
        x=rng.randint(-40,40); y=rng.randint(-40,40)
        # Each accepted move carries a deterministic synthetic segment count.
        segs=rng.randint(2,14)
        cmds.append((f'G1 X{x} Y{y} F6000\n'.encode(), 'G1', segs))
        if include_m105 and i and i % 37 == 0:
            cmds.append((b'M105\n','M105',0))
    cmds.append((b'M400\n','M400',0))
    return cmds


class RealtimeSim:
    def __init__(self, move_count, seed, paced, host_window=4):
        self.r=random.Random(seed)
        self.move_count=move_count
        self.commands=deque(make_commands(move_count,self.r,True))
        self.paced=paced
        self.host_window=host_window
        self.outstanding=0
        self.wire=deque()          # serialized bytes waiting on UART wire
        self.wire_next_us=0.0
        self.rx=deque()
        self.line=bytearray()
        self.line_meta=deque()     # command metadata in send order

        self.path=deque()          # segment counts for prepared moves
        self.pending=deque()
        self.current_segments=0
        self.accepted=0
        self.executed=0
        self.closed=False

        self.motorq=deque()        # block durations in microseconds
        self.motor_started=False
        self.motor_end_us=None
        self.starves=0

        self.now=0.0
        self.max_rx=0
        self.max_pending=0
        self.max_motorq=0
        self.acks_due=deque()

    def host_pump(self):
        limit=self.host_window if self.paced else 10**9
        while self.commands and self.outstanding < limit:
            raw,kind,segs=self.commands.popleft()
            self.wire.extend(raw)
            self.line_meta.append((kind,segs))
            self.outstanding += 1
            if not self.paced and len(self.wire) > 100000:
                break

    def service_acks(self):
        while self.acks_due and self.acks_due[0] <= self.now:
            self.acks_due.popleft()
            if self.outstanding:
                self.outstanding -= 1
        self.host_pump()

    def host_arrive_until(self,t):
        self.service_acks()
        if self.wire_next_us < self.now:
            self.wire_next_us=self.now
        while self.wire and self.wire_next_us <= t:
            if len(self.rx) >= RX_CAP:
                raise AssertionError('RX overflow')
            self.rx.append(self.wire.popleft())
            self.wire_next_us += BYTE_US
        self.max_rx=max(self.max_rx,len(self.rx))

    def consume_motor_until(self,t):
        while self.motor_started:
            if self.motor_end_us is None:
                if not self.motorq:
                    return
                self.motor_end_us=self.now+self.motorq[0]
            if self.motor_end_us > t:
                return
            self.now=self.motor_end_us
            self.motorq.popleft()
            self.motor_end_us=None
            # Queue empty while future generated work still exists = real starvation.
            if not self.motorq and (self.current_segments or self.path or self.pending or self.accepted>self.executed):
                self.starves += 1
                self.motor_started=False
                return

    def advance(self,us):
        target=self.now+us
        self.host_arrive_until(target)
        self.consume_motor_until(target)
        self.now=target
        self.service_acks()

    def ack(self):
        if self.paced:
            self.acks_due.append(self.now+ACK_TX_US)
        else:
            # In raw burst mode the sender ignores ACKs; account only for bookkeeping.
            if self.outstanding:
                self.outstanding -= 1

    def serial_slice(self):
        lines=0; consumed=0
        while self.rx and lines<2 and consumed<96:
            # Firmware admission control: only pause at a line boundary and
            # only when the compact motion ingress is genuinely near full.
            # Crucially, no ACK is generated while paused.
            if not self.line and (len(self.path)+len(self.pending)) >= ADMISSION_LIMIT:
                return
            c=self.rx.popleft(); consumed+=1
            self.advance(2)
            if c==13:
                continue
            if c!=10:
                self.line.append(c)
                if len(self.line)>=LINE_CAP:
                    raise AssertionError('line overflow')
                continue

            text=self.line.decode('ascii'); self.line.clear(); lines+=1
            if not self.line_meta:
                raise AssertionError('metadata desync')
            kind,segs=self.line_meta.popleft()
            self.advance(PARSE_LINE_US)
            if kind=='G1':
                if not text.startswith('G1 '):
                    raise AssertionError('corrupt G1 '+repr(text))
                if len(self.path) < PATH_CAP:
                    self.path.append(segs)
                elif len(self.pending) < PENDING_CAP:
                    self.pending.append(segs)
                else:
                    raise AssertionError('motion ingress overflow')
                self.accepted+=1
                self.ack()
            elif kind=='M105':
                if text!='M105': raise AssertionError('corrupt M105 '+repr(text))
                self.ack()
            elif kind=='M400':
                if text!='M400': raise AssertionError('corrupt M400 '+repr(text))
                self.closed=True
                # Real firmware withholds M400 ok until motion completion.
            else:
                raise AssertionError(kind)
        self.max_rx=max(self.max_rx,len(self.rx))

    def fill_planner(self):
        while len(self.path)<PATH_CAP and self.pending:
            self.path.append(self.pending.popleft())
        self.max_pending=max(self.max_pending,len(self.pending))

    def can_commit(self):
        return bool(self.path) and (self.closed or len(self.path)>LOOKAHEAD_RESERVE)

    def new_block_duration(self):
        # Deliberately harsh: actual v0.5 blocks in supplied logs span a broad range.
        return self.r.randint(1250,10000)

    def produce(self):
        self.fill_planner()
        urgent=len(self.motorq)<LOW
        limit=MAX_BURST if urgent else 1
        spent=0; produced=0

        while produced<limit and len(self.motorq)<MOTOR_CAP:
            if self.current_segments==0:
                if not self.can_commit(): break
                self.advance(PLAN_US); spent+=PLAN_US
                self.current_segments=self.path[0]

            if produced and spent+GEN_US>BUDGET_US: break
            self.advance(GEN_US); spent+=GEN_US
            self.motorq.append(self.new_block_duration())
            self.max_motorq=max(self.max_motorq,len(self.motorq))
            self.current_segments-=1; produced+=1

            if self.current_segments==0:
                self.path.popleft(); self.executed+=1; self.fill_planner()

            if not self.motor_started and (len(self.motorq)>=START_PREFILL or (self.closed and not self.path and not self.pending and not self.current_segments)):
                self.motor_started=True
                self.motor_end_us=None

            if len(self.motorq)>=TARGET: break
            if len(self.motorq)>=LOW and self.rx: break
            if spent>=BUDGET_US: break

    def done(self):
        return (self.closed and not self.commands and not self.wire and not self.rx and not self.line
                and not self.path and not self.pending and not self.current_segments
                and self.accepted==self.executed)

    def run(self):
        self.host_pump()
        for _ in range(8_000_000):
            self.host_arrive_until(self.now)
            self.serial_slice()
            self.produce()
            self.advance(250)
            if self.done():
                while self.motorq:
                    if not self.motor_started:
                        self.motor_started=True; self.motor_end_us=None
                    self.advance(max(self.motorq[0],40))
                break
        else:
            raise AssertionError('dead state')
        assert self.accepted==self.move_count,(self.accepted,self.move_count)
        assert self.executed==self.move_count,(self.executed,self.move_count)
        assert self.starves==0,self.starves
        return self.max_rx,self.max_pending,self.max_motorq,self.now


def run_burst():
    # Raw unpaced sender: bounded by finite AVR memory by definition. Validate the
    # real-world paste sizes that exposed corruption, including M105 polls, using
    # the actual 512-byte RX reservoir and measured ~10 ms planner stall.
    for moves in (45,65,75):
        worst=(0,0,0)
        for seed in range(30):
            s=RealtimeSim(moves,seed,paced=False)
            mr,mp,mq,_=s.run(); worst=(max(worst[0],mr),max(worst[1],mp),max(worst[2],mq))
        print(f'PASS raw-burst moves={moves} seeds=30 max_rx={worst[0]}/{RX_CAP} max_pending={worst[1]}/{PENDING_CAP} motorq_hi={worst[2]}/{MOTOR_CAP}')


def run_credit_stream():
    for moves,seeds in ((200,20),(1000,8),(3000,3)):
        worst=(0,0,0)
        for seed in range(seeds):
            s=RealtimeSim(moves,seed,paced=True,host_window=4)
            mr,mp,mq,_=s.run(); worst=(max(worst[0],mr),max(worst[1],mp),max(worst[2],mq))
        print(f'PASS credit-stream moves={moves} seeds={seeds} window=4 max_rx={worst[0]}/{RX_CAP} max_pending={worst[1]}/{PENDING_CAP} motorq_hi={worst[2]}/{MOTOR_CAP}')


def main():
    run_burst()
    run_credit_stream()
    print('PASS realtime UART + M105 + measured planner-stall model: no corruption, overflow, reorder, or motor starvation')

if __name__=='__main__':
    main()
