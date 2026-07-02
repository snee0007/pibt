#!/bin/bash
# Tune FROZEN_CAP across all 7 maps. Uses the SAME detection as working sweep_full.sh.
MAPS="four_rooms roomfull flow deadlock5 room_corridor demo_rooms full"
NS="15 20 25 30 35 40"; SEEDS=30
echo "cap,total_solved,total_time_s" > frozen_tuning.csv
for CAP in 10 40 60 100 150; do
  solved=0; t0=$(date +%s)
  for mp in $MAPS; do for N in $NS; do for s in $(seq 0 $((SEEDS-1))); do
    out=$(FROZEN_CAP=$CAP timeout 5 ./build_andy/main -v 1 -t 3 -s $s -m scripts/map/$mp.map -N $N 2>&1)
    if echo "$out" | grep -q "makespan:" && ! echo "$out" | grep -q "invalid solution"; then solved=$((solved+1)); fi
  done; done; done
  t1=$(date +%s); echo "$CAP,$solved,$((t1-t0))" >> frozen_tuning.csv
  echo "cap=$CAP -> $solved solved ($((t1-t0))s)"
done
echo "=== DONE - frozen_tuning.csv ==="
