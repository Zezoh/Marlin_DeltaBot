#!/usr/bin/env python3
import random
from collections import deque

PATH_CAP = 16
PENDING_CAP = 32
HIGH_WATER = PATH_CAP + PENDING_CAP - 4
LOOKAHEAD_RESERVE = 4
LINES_PER_PASS = 2

class StreamSim:
    def __init__(self):
        self.planner = deque()
        self.pending = deque()
        self.host = deque()
        self.accepted = []
        self.committed = []
        self.generated = []
        self.stream_active = False
        self.flush = False
        self.generating = None
        self.seg_left = 0
        self.max_buffered = 0
        self.fault = None

    def buffered(self):
        return len(self.planner) + len(self.pending) + (1 if self.generating is not None else 0)

    def fill(self):
        while len(self.planner) < PATH_CAP and self.pending:
            self.planner.append(self.pending.popleft())

    def ingress(self):
        lines = 0
        while self.host and lines < LINES_PER_PASS:
            if self.buffered() >= HIGH_WATER:
                break
            cmd = self.host[0]
            if cmd == 'FLUSH':
                self.host.popleft(); self.flush = True; lines += 1; continue
            self.host.popleft()
            if len(self.pending) >= PENDING_CAP:
                self.fault = 'pending overflow'; return
            self.pending.append(cmd)
            self.accepted.append(cmd)
            self.fill()
            lines += 1

    def service(self):
        self.fill()
        if not self.stream_active and self.planner and (len(self.planner) == PATH_CAP or self.flush):
            self.stream_active = True
        if self.stream_active and self.generating is None and self.planner:
            if self.flush or len(self.planner) > LOOKAHEAD_RESERVE:
                self.generating = self.planner[0]
                self.committed.append(self.generating)
                self.seg_left = 1 + (sum(self.generating.encode()) & 7)
        if self.generating is not None:
            self.seg_left -= 1
            if self.seg_left <= 0:
                done = self.planner.popleft()
                if done != self.generating:
                    self.fault = 'planner reorder'; return
                self.generated.append(done)
                self.generating = None
                self.fill()
        self.max_buffered = max(self.max_buffered, self.buffered())

    def run(self, commands, max_passes=2_000_000):
        self.host.extend(commands)
        self.host.append('FLUSH')
        for _ in range(max_passes):
            self.ingress()
            if self.fault: break
            self.service()
            if self.fault: break
            if not self.host and self.flush and not self.planner and not self.pending and self.generating is None:
                return
        self.fault = self.fault or 'timeout'


def exact_user_path():
    pts = [(0,0,120),(5,0,120),(10,2,120),(15,5,120),(20,10,120),(23,15,120),
           (25,20,120),(23,25,120),(20,30,120),(15,35,120),(10,38,120),(5,40,120),
           (0,40,120),(-5,40,120),(-10,38,120),(-15,35,120),(-20,30,120),(-23,25,120),
           (-25,20,120),(-23,15,120),(-20,10,120),(-15,5,120),(-10,2,120),(-5,0,120),(0,0,120),
           (6,-2,120),(12,-5,120),(18,-10,120),(22,-16,120),(24,-22,120),(22,-28,120),
           (18,-34,120),(12,-38,120),(6,-40,120),(0,-40,120),(-6,-40,120),(-12,-38,120),
           (-18,-34,120),(-22,-28,120),(-24,-22,120),(-22,-16,120),(-18,-10,120),(-12,-5,120),
           (-6,-2,120),(0,0,120)]
    return [f'M{i}:{p}' for i,p in enumerate(pts)]

def check(commands):
    s = StreamSim(); s.run(commands)
    assert not s.fault, s.fault
    assert s.accepted == commands
    assert s.committed == commands
    assert s.generated == commands
    assert s.max_buffered <= HIGH_WATER + LINES_PER_PASS
    return s

s = check(exact_user_path())
print(f'PASS exact 45-move stream max_buffered={s.max_buffered}')

rng = random.Random(0xD311AC0)
for trial in range(2500):
    n = rng.randint(1, 2000)
    cmds = [f'T{trial}:{i}:{rng.randrange(-8500,8501)}:{rng.randrange(-8500,8501)}:{rng.randrange(7000,22001)}' for i in range(n)]
    check(cmds)
print('PASS 2500 randomized streams, 1..2000 moves, no loss/reorder/fault')
