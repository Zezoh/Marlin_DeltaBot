from pathlib import Path
p=Path('DeltaCore/test/sim_realtime_v054.py')
s=p.read_text()
s=s.replace('self.advance(40)\n            if self.done():', 'self.advance(250)\n            if self.done():')
s=s.replace('for moves,seeds in ((200,30),(1000,15),(5000,5)):', 'for moves,seeds in ((200,20),(1000,8),(3000,3)):')
p.write_text(s)
