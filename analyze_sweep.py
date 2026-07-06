#!/usr/bin/env python3
# Usage: python3 analyze_sweep.py full_sweep_v3.csv
import csv, sys
from collections import defaultdict, Counter

fn = sys.argv[1] if len(sys.argv)>1 else 'full_sweep_v3.csv'
rows=list(csv.DictReader(open(fn)))

# ---- per (map,N) table ----
agg=defaultdict(lambda:{'n':0,'p':0,'w':0,'c':0,'fires':0,'maxt':0.0,'th':0})
for r in rows:
    k=(r['map'],int(r['N'])); a=agg[k]; a['n']+=1
    a['p']+=int(r['pure']); a['w']+=int(r['swap']); a['c']+=int(r['cnt'])
    a['fires']+=int(r['cnt_fires']); t=float(r['cnt_time'])
    a['maxt']=max(a['maxt'],t)
    if t>4: a['th']+=1

print("="*72)
print("PER-MAP / PER-N SUCCESS TABLE  (pure=PIBT, swap=Mike, cnt=Sandy)")
print("="*72)
print(f"{'map':<14}{'N':>4} | {'pure':>6}{'swap':>6}{'cnt':>6} | {'fires':>7}{'thrash':>7}{'maxt':>7}")
print("-"*72)
cur=None; tp=tw=tc=0
for k in sorted(agg):
    if k[0]!=cur: print(); cur=k[0]
    a=agg[k]; n=a['n']; tp+=a['p']; tw+=a['w']; tc+=a['c']
    print(f"{k[0]:<14}{k[1]:>4} | {a['p']:>4}/{n}{a['w']:>4}/{n}{a['c']:>4}/{n} | {a['fires']:>7}{a['th']:>7}{a['maxt']:>6.1f}s")
print("-"*72)
print(f"{'TOTALS':<19}| {tp:>4}    {tw:>4}    {tc:>4}   (out of {len(rows)})")

# ---- pairwise comparison helper ----
def pair(aidx,bidx,aname,bname):
    both=onlyA=onlyB=neither=0
    a_wins=[]; b_wins=[]; nfail=[]
    for r in rows:
        A=int(r[aidx]); B=int(r[bidx])
        if A and B: both+=1
        elif A and not B: onlyA+=1; a_wins.append((r['map'],r['N'],r['seed']))
        elif B and not A: onlyB+=1; b_wins.append((r['map'],r['N'],r['seed']))
        else: neither+=1; nfail.append((r['map'],r['N'],r['seed']))
    print(f"\n{'='*72}\n{aname} vs {bname}\n{'='*72}")
    print(f"  both solved:              {both}")
    print(f"  ONLY {aname} ({aname} solves, {bname} fails): {onlyA}")
    print(f"  ONLY {bname} ({bname} solves, {aname} fails): {onlyB}")
    print(f"  BOTH FAIL ({aname} & {bname} both fail):      {neither}")
    return a_wins,b_wins,nfail

def by_map(cases,label):
    if not cases: return
    print(f"    {label} by map:")
    for m,c in sorted(Counter(m for m,n,s in cases).items(),key=lambda x:-x[1]):
        print(f"      {m}: {c}")

# Sandy vs Pure
sw,pw,nf = pair('cnt','pure','Sandy','Pure')
by_map(sw,"Sandy wins (Pure fails)")
print("\n  Sandy REGRESSIONS (Pure solves, Sandy fails):")
for m,n,s in sorted(pw): print(f"      {m} N={n} seed={s}")
by_map(nf,"BOTH FAIL (Sandy & Pure)")

# Sandy vs Mike
sw2,mw,nf2 = pair('cnt','swap','Sandy','Mike')
by_map(sw2,"Sandy wins (Mike fails)")
print("\n  Sandy worse than Mike (Mike solves, Sandy fails):")
for m,n,s in sorted(mw): print(f"      {m} N={n} seed={s}")
by_map(nf2,"BOTH FAIL (Sandy & Mike)")

# Pure vs Mike
pm,mp,nf3 = pair('pure','swap','Pure','Mike')
by_map(pm,"Pure wins (Mike fails)")
by_map(mp,"Mike wins (Pure fails)")
by_map(nf3,"BOTH FAIL (Pure & Mike)")

# ---- ALL THREE FAIL ----
allfail=[(r['map'],r['N'],r['seed']) for r in rows
         if not int(r['pure']) and not int(r['swap']) and not int(r['cnt'])]
allsolve=sum(1 for r in rows if int(r['pure']) and int(r['swap']) and int(r['cnt']))
print(f"\n{'='*72}\nALL THREE\n{'='*72}")
print(f"  all three SOLVE: {allsolve}")
print(f"  all three FAIL:  {len(allfail)}")
by_map(allfail,"all-three-fail")

# ---- decidable view (drop all-three-fail) ----
dec=[r for r in rows if int(r['pure'])+int(r['swap'])+int(r['cnt'])>0]
d=len(dec)
print(f"\n{'='*72}\nDECIDABLE VIEW (only the {d} instances at least one method solves)\n{'='*72}")
print(f"  pure {sum(int(r['pure']) for r in dec)} ({100*sum(int(r['pure']) for r in dec)/d:.0f}%)")
print(f"  Mike {sum(int(r['swap']) for r in dec)} ({100*sum(int(r['swap']) for r in dec)/d:.0f}%)")
print(f"  Sandy {sum(int(r['cnt']) for r in dec)} ({100*sum(int(r['cnt']) for r in dec)/d:.0f}%)")
