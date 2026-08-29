import math
TIMER=2_000_000; STEPS=80.; R=90.; ROD=210.
start=(0.,0.,225.); target=(40.,0.,120.); L=math.sqrt(sum((target[i]-start[i])**2 for i in range(3))); unit=tuple((target[i]-start[i])/L for i in range(3))
v0=v1=2.; vmax=80.; acc=1600.; jerk=18000.
def tt(a,b):
 d=abs(b-a); lim=acc*acc/jerk
 return 2*math.sqrt(d/jerk) if d<=lim else 2*(acc/jerk)+(d/acc-acc/jerk)
def td(a,b): return .5*(a+b)*tt(a,b)
need=td(v0,vmax)+td(vmax,v1); cruise=(L-need)/vmax
ph=[]; t=s=0.; v=v0; a=0.
def add(d,j):
 global t,s,v,a
 if d<=1e-12:return
 ph.append((t,d,s,v,a,j)); s+=v*d+.5*a*d*d+j*d*d*d/6; v+=a*d+.5*j*d*d; a+=j*d; t+=d
q=math.sqrt((vmax-v0)/jerk); add(q,jerk);add(q,-jerk);add(cruise,0);add(q,-jerk);add(q,jerk); T=t
def sample(x):
 if x>=T:return L
 for t0,d,s0,v0x,a0,j in ph:
  if x<t0+d:
   u=x-t0;return s0+v0x*u+.5*a0*u*u+j*u*u*u/6
 return L
txy=[(-math.sqrt(3)*.5*R,-.5*R),(math.sqrt(3)*.5*R,-.5*R),(0.,R)]
def tower(p):return [p[2]+math.sqrt(ROD*ROD-(tx-p[0])**2-(ty-p[1])**2) for tx,ty in txy]
prev=[round(x*STEPS) for x in tower(start)]; now=0.; ticks=0; mins=65535; totals=[0,0,0]; blocks=0
while now<T-1e-10:
 nxt=min(T,now+.01); dist=sample(nxt); p=tuple(start[i]+unit[i]*dist for i in range(3)); cur=[round(x*STEPS) for x in tower(p)]; ds=[cur[i]-prev[i] for i in range(3)]; ev=max(max(abs(x) for x in ds),1); total=round((nxt-now)*TIMER); base=total//ev; rem=total%ev
 assert base>=120,(now,base,ds); assert base*ev+rem==total
 mins=min(mins,base); ticks+=total; blocks+=1
 for i in range(3):totals[i]+=abs(ds[i])
 prev=cur;now=nxt
assert abs(ticks/TIMER-T)<2e-4,(ticks/TIMER,T)
assert totals==[10153,7452,8741],totals
assert 1.532<T<1.534,T
assert mins>=260,mins
print(f'PASS v0.5.0 sim trajectory_s={T:.6f} blocks={blocks} steps={totals} min_interval_ticks={mins}')
