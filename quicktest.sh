#!/bin/bash
# Fast regression check (~2 min). Covers wins, known regressions, livelock, each map.
declare -a CASES=(
  "four_rooms 30 0" "four_rooms 30 5" "four_rooms 30 21" "four_rooms 25 15"
  "four_rooms 35 5" "four_rooms 35 22" "flow 40 2"
  "roomfull 25 4" "roomfull 30 4" "flow 30 11" "flow 25 4" "flow 40 16"
  "demo_rooms 30 0" "room_corridor 30 0" "full 30 0" "deadlock5 15 7"
)
pass=0; fail=0
for c in "${CASES[@]}"; do
  set -- $c
  out=$(timeout 10 ./build_andy/main -v 1 -t 8 -s $3 -m scripts/map/$1.map -N $2 2>&1)
  if echo "$out" | grep -q "invalid goals"; then echo "FAIL    $1 N=$2 s=$3"; fail=$((fail+1))
  elif echo "$out" | grep -q "solved:"; then echo "ok      $1 N=$2 s=$3 $(echo "$out"|grep -oE 'makespan: [0-9]+'|head -1)"; pass=$((pass+1))
  else echo "TIMEOUT $1 N=$2 s=$3"; fail=$((fail+1)); fi
done
echo "=== $pass ok, $fail fail ==="
