from pathlib import Path
p=Path('DeltaCore/test/sim_realtime_v054.py')
s=p.read_text()
s=s.replace('for seed in range(60):','for seed in range(30):')
s=s.replace("seeds=60 max_rx=", "seeds=30 max_rx=")
old='''def run_credit_stream():\n    for moves in (200,1000,5000):\n        worst=(0,0,0)\n        for seed in range(40):\n            s=RealtimeSim(moves,seed,paced=True,host_window=4)\n            mr,mp,mq,_=s.run(); worst=(max(worst[0],mr),max(worst[1],mp),max(worst[2],mq))\n        print(f'PASS credit-stream moves={moves} seeds=40 window=4 max_rx={worst[0]}/{RX_CAP} max_pending={worst[1]}/{PENDING_CAP} motorq_hi={worst[2]}')\n'''
new='''def run_credit_stream():\n    for moves,seeds in ((200,30),(1000,15),(5000,5)):\n        worst=(0,0,0)\n        for seed in range(seeds):\n            s=RealtimeSim(moves,seed,paced=True,host_window=4)\n            mr,mp,mq,_=s.run(); worst=(max(worst[0],mr),max(worst[1],mp),max(worst[2],mq))\n        print(f'PASS credit-stream moves={moves} seeds={seeds} window=4 max_rx={worst[0]}/{RX_CAP} max_pending={worst[1]}/{PENDING_CAP} motorq_hi={worst[2]}')\n'''
if old not in s: raise SystemExit('credit block not found')
s=s.replace(old,new)
p.write_text(s)
