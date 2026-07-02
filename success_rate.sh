#!/bin/bash
# Success-rate harness: many random instances per (map, N), fixed seeds, measure SOLVABILITY.
# Usage: ./success_rate.sh <map> <N> <num_seeds>
MAP=$1; N=$2; SEEDS=${3:-100}
TLIMIT=8

solved() {  # $1=build dir, $2=seed ; echoes 1 if valid solve, 0 otherwise
  local out=$(timeout $((TLIMIT+5)) "$1"/main -v 1 -t $TLIMIT -s "$2" -m "$MAP" -N "$N" 2>/dev/null)
  if echo "$out" | grep -q "invalid solution"; then echo 0
  elif echo "$out" | grep -q "makespan:"; then echo 1
  else echo 0; fi
}

pure_ok=0; swap_ok=0; cnt_ok=0
win_seeds=""      # pure fails, counter solves (our evidence)
cnt_fail_seeds="" # counter fails (to investigate)

for s in $(seq 0 $((SEEDS-1))); do
  p=$(solved build_pure $s)
  w=$(solved build_nocnt $s)
  c=$(solved build_andy $s)
  pure_ok=$((pure_ok+p)); swap_ok=$((swap_ok+w)); cnt_ok=$((cnt_ok+c))
  if [ "$p" = "0" ] && [ "$c" = "1" ]; then win_seeds="$win_seeds $s"; fi
  if [ "$c" = "0" ]; then cnt_fail_seeds="$cnt_fail_seeds $s"; fi
done

echo "=========================================="
echo "MAP=$MAP  N=$N  instances=$SEEDS"
echo "  pure PIBT solved:    $pure_ok / $SEEDS"
echo "  swap   solved:       $swap_ok / $SEEDS"
echo "  counter(Sandy):      $cnt_ok / $SEEDS"
echo "  WINS (pure fail, Sandy solve): seeds =$win_seeds"
echo "  Sandy FAILURES:                seeds =$cnt_fail_seeds"
echo "=========================================="
