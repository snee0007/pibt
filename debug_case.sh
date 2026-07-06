#!/bin/bash
# Debug ONE case with failure classification.
# Usage: ./debug_case.sh <map> <N> <seed>
MAP=$1; N=$2; SEED=$3
echo "======================================================"
echo "=== $MAP  N=$N  seed=$SEED ==="
echo "======================================================"
./build_andy/main -v 1 -t 8 -s $SEED -m scripts/map/$MAP.map -N $N > /tmp/run_full.txt 2>&1
cp /tmp/live_viz.txt ~/pibt/live_viz.txt

# result
if grep -q "invalid solution" /tmp/run_full.txt; then RES="FAILED"; echo "RESULT: FAILED (invalid)"
else RES="SOLVED"; echo "RESULT: SOLVED  $(grep -oE 'makespan: [0-9]+' /tmp/run_full.txt | head -1)"; fi

# how it ended
echo ""; echo "--- how the loop ended ---"
end=$(grep -oE "deadlock at step [0-9]+|done in [0-9]+ steps|viz cap" /tmp/run_full.txt | tail -1)
if [ -z "$end" ]; then echo "No clean exit msg. Last 4 log lines:"; tail -4 /tmp/run_full.txt
else echo "Ended via: $end"; fi

# signals
fires=$(grep -c ADMIT /tmp/run_full.txt)
parks=$(grep -c "\[PARK\]" /tmp/run_full.txt)
swaps=$(grep -c "MIKE-SWAP" /tmp/run_full.txt)
maxpri=$(grep -oE "pri [0-9]+" /tmp/run_full.txt | grep -oE "[0-9]+" | sort -n | tail -1)
steps=$(tail -1 /tmp/live_viz.txt | grep -oE "^[0-9]+")

echo ""; echo "--- signals ---"
echo "counter fires: $fires | parks: $parks | swaps: $swaps"
echo "max priority seen: $maxpri | steps run: $steps"

# ping-pong check: are the LAST swaps dominated by the SAME pair?
echo ""; echo "--- swap pattern (last 10 swaps, agent numbers) ---"
pairline=$(grep "MIKE-SWAP" /tmp/run_full.txt | tail -10 | grep -oE "agent [0-9]+ .* blocker agent [0-9]+")
echo "$pairline" | grep -oE "[0-9]+" | sort | uniq -c | sort -rn | head -4 | while read cnt ag; do echo "  agent $ag appeared $cnt times"; done

# stranded
echo ""; echo "--- stranded agents ---"
python3 -c "
lines=open('/tmp/live_viz.txt').read().split('\n')
goals=[];last=None
for ln in lines:
    if ln.startswith('goals:'): goals=[tuple(map(int,p.split(','))) for p in ln[6:].split(';') if p]
    elif '|pos:' in ln:
        pos=[tuple(map(int,p.split(','))) for p in ln.split('|pos:')[1].split('|pri:')[0].split(';') if p]
        last=(int(ln.split('|')[0]),pos)
sn,pos=last
stuck=[(i,p,goals[i]) for i,p in enumerate(pos) if i<len(goals) and p!=goals[i]]
print(f'step {sn}: {len(stuck)}/{len(pos)} stranded')
for i,p,g in stuck[:12]:
    d=abs(p[0]-g[0])+abs(p[1]-g[1]); print(f'  a{i}: at {p} goal {g} (dist {d})')
" 2>/dev/null

# CLASSIFY
echo ""; echo "--- LIKELY FAILURE CATEGORY ---"
if [ "$RES" = "SOLVED" ]; then echo "  [OK] solved"; else
  nstuck=$(python3 -c "
lines=open('/tmp/live_viz.txt').read().split('\n')
goals=[];last=None
for ln in lines:
    if ln.startswith('goals:'): goals=[tuple(map(int,p.split(','))) for p in ln[6:].split(';') if p]
    elif '|pos:' in ln: last=[tuple(map(int,p.split(','))) for p in ln.split('|pos:')[1].split('|pri:')[0].split(';') if p]
print(sum(1 for i,p in enumerate(last) if i<len(goals) and p!=goals[i]))
" 2>/dev/null)
  # A: priority inflation
  if [ -n "$maxpri" ] && [ -n "$steps" ] && [ "$maxpri" -gt $((steps + 100)) ]; then
    echo "  [A] PRIORITY-INFLATION: max pri $maxpri >> steps $steps -> exit-boost (+999) active? (should be OFF now)"
  fi
  # B: same-pair ping-pong (one or two agents dominate the last swaps)
  top=$(echo "$pairline" | grep -oE "[0-9]+" | sort | uniq -c | sort -rn | head -1 | awk '{print $1}')
  if [ -n "$top" ] && [ "$top" -ge 8 ]; then
    echo "  [B] SAME-PAIR PING-PONG: one agent in nearly every recent swap -> swap livelock (2 agents can't pass)"
  fi
  # C: park-freeze
  if echo "$end" | grep -q "deadlock" && [ "$parks" -gt 0 ]; then
    echo "  [C] PARK-FREEZE: deadlock-break with parks (should be fixed; if here, frozen cap too low)"
  fi
  # E: small local tangle vs mass gridlock
  if [ -n "$nstuck" ]; then
    if [ "$nstuck" -le 5 ]; then echo "  [E] SMALL LOCAL TANGLE: only $nstuck agents stuck (a few agents in a tight spot, not map-wide)"
    else echo "  [F] MASS GRIDLOCK: $nstuck agents stranded (map-wide jam)"; fi
  fi
  # D: counter not involved
  if [ "$fires" -le 2 ]; then echo "  [D] COUNTER NOT INVOLVED ($fires fires) -> difference from pure is the SWAP, not counter"; fi
fi
echo ""
echo "=== VISUALIZE: load scripts/map/$MAP.map + ~/pibt/live_viz.txt in viz_player.html ==="
