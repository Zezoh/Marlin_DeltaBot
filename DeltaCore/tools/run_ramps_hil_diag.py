#!/usr/bin/env python3
from __future__ import annotations
import argparse, queue, sys, threading, time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Install pyserial: python -m pip install pyserial", file=sys.stderr)
    raise SystemExit(2)

BAUD=250000

class Link:
    def __init__(self, port):
        self.ser=serial.Serial(port, BAUD, timeout=0.05, write_timeout=2)
        self.q=queue.Queue(); self.stop=threading.Event()
        self.t=threading.Thread(target=self.reader, daemon=True); self.t.start()
    def reader(self):
        while not self.stop.is_set():
            b=self.ser.readline()
            if not b: continue
            s=b.decode(errors='replace').strip(); print('<',s); self.q.put(s)
    def send(self,s):
        print('>',s); self.ser.write((s+'\n').encode()); self.ser.flush()
    def wait(self,pred,timeout=5):
        end=time.monotonic()+timeout; seen=[]
        while time.monotonic()<end:
            try: s=self.q.get(timeout=0.2)
            except queue.Empty: continue
            seen.append(s)
            if pred(s): return seen
        raise TimeoutError(seen[-10:])
    def cmd(self,s,timeout=5):
        self.send(s); return self.wait(lambda x:x.startswith('OK') or x=='OK',timeout)
    def close(self):
        self.stop.set(); self.t.join(timeout=.2); self.ser.close()

def port_auto():
    ps=list(serial.tools.list_ports.comports())
    if not ps: raise RuntimeError('No serial ports')
    return ps[0].device

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--port')
    ap.add_argument('--heater-pulse-ms',type=int,default=250)
    args=ap.parse_args()
    p=args.port or port_auto(); print('Using',p)
    l=Link(p)
    try:
        time.sleep(2.0)
        l.send('STATUS'); status='\n'.join(l.wait(lambda s:s=='OK',3))
        print('\nINPUT SNAPSHOT\n'+status)
        print('\nPhysically toggle Z-PROBE and filament switch once each, then press Enter.')
        input()
        l.send('STATUS'); status2='\n'.join(l.wait(lambda s:s=='OK',3))
        print(status2)

        ans=input('\nMotors mechanically safe? Type YES to jog A/B/C/E both directions: ').strip().upper()
        if ans!='YES':
            print('Motor/output tests skipped safely'); return 1

        l.cmd('ARM')
        for c in ('MA+','MA-','MB+','MB-','MC+','MC-','ME+','ME-'):
            l.cmd(c,5)

        for v in (0,64,128,192,255,0): l.cmd(f'FPWM {v}')
        l.cmd('FON 1'); time.sleep(.5); l.cmd('FON 0')

        hp=max(1,min(1000,args.heater_pulse_ms))
        print('\nHEATER test must use a dummy load/power resistor or lamp. Do NOT use an unattended hotend.')
        ans=input(f'Type HEAT to pulse D10 for {hp} ms: ').strip().upper()
        if ans=='HEAT':
            l.cmd('ARM'); l.cmd(f'HEAT {hp}')
            l.wait(lambda s:s=='HEATER_OFF',3)
        else:
            print('Heater pulse skipped')

        l.cmd('THERM')
        l.cmd('SAFE')
        print('\nRAMPS FULL-IO DIAGNOSTIC: PASS (manual electrical observations still required)')
        return 0
    except Exception as e:
        try: l.send('SAFE')
        except Exception: pass
        print('\nFAIL:',e)
        return 1
    finally:
        l.close()

if __name__=='__main__': raise SystemExit(main())
