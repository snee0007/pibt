#!/bin/bash
# Sweep: for each (map,N,seed) record solve + counter fires + time, all 3 builds. Writes CSV incrementally.
# Usage: ./sweep_full.sh out.csv "map1 map2" "20 30 40" num_seeds
OUT=$1; MAPS=$2; NS=$3; SEEDS=${4:-30}; TLIMIT=5

echo "map,N,seed,pure,swap,cnt,cnt_fires,cnt_time" > "$OUT"

run1() {  # build map N seed -> "solved fires time"
  local t0=$(date +%s.%N)
  local out=$(timeout $((TLIMIT+3)) "$1"/main -v 1 -t $TLIMIT -s "$4" -m "$2" -N "$3" 2>/dev/null)
  local t1=$(date +%s.%N)
  local solved=0
  if echo "$out" | grep -q "makespan:" && ! echo "$out" | grep -q "invalid solution"; then solved=1; fi
  local fires=$(echo "$out" | grep -c ADMIT)
  local dt=$(echo "$t1 - $t0" | bc)
  echo "$solved $fires $dt"
}

for mp in $MAPS; do
  MAPPATH=scripts/map/$mp.map
  for N in $NS; do
    for s in $(seq 0 $((SEEDS-1))); do
      read p_s p_f p_t <<< "$(run1 build_pure $MAPPATH $N $s)"
      read w_s w_f w_t <<< "$(run1 build_nocnt $MAPPATH $N $s)"
      read c_s c_f c_t <<< "$(run1 build_andy $MAPPATH $N $s)"
      echo "$mp,$N,$s,$p_s,$w_s,$c_s,$c_f,$c_t" >> "$OUT"
    done
    echo "done $mp N=$N"
  done
done
echo "=== DONE -> $OUT ==="
