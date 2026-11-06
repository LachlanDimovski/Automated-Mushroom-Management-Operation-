#!/bin/bash
# One 2+1 fleet run on each released shuffled layout. Rules fixed before the first run:
# 1800 s wall cap, stall after 600 s with no new collection, no reruns. QT_QPA_PLATFORM=offscreen keeps
# the Fleet Debug process running with no window. setsid is needed: the coordinator ends the mission by
# signalling its process group. TZ below is the CPU temperature file on the team laptop.
REPO=$(cd "$(dirname "$0")/../.." && pwd)
OUT=${OUT:-$HOME/ammo_layouts}
mkdir -p "$OUT"; cd "$REPO" || exit 1
TZ=/sys/class/thermal/thermal_zone6/temp
temp() { echo $(( $(cat $TZ) / 1000 )); }
echo "start $(date '+%F %T') commit $(git rev-parse --short HEAD)" >> "$OUT/results.txt"
for w in m1 m2 m3; do
  bash demo/stop_sim.sh > "$OUT/stop_before_$w.txt" 2>&1
  c0=$(date +%s); while [ "$(temp)" -ge 70 ] && [ $(( $(date +%s) - c0 )) -lt 600 ]; do sleep 10; done
  D=$OUT/$w; mkdir -p "$D"; W=river_moderate_aus_$w; startT=$(temp)
  QT_QPA_PLATFORM=offscreen AMMO_LOGS=$D WORLD=$W setsid bash demo/part1_fleet.sh > "$D/part1.out" 2>&1 &
  P=$!
  sleep 8
  AMMO_LOGS=$D WORLD=$W timeout 1900 bash demo/part1_timer.sh > "$D/timer.out" 2>&1 &
  T=$!
  t0=$(date +%s); last=$t0; prev=0; peak=0; res=
  while :; do
    sleep 10; now=$(date +%s); tc=$(temp); [ "$tc" -gt "$peak" ] && peak=$tc
    c=$(grep -oE 'Collected [0-9]+ mushroom\(s\)' "$D/part1_sim.log" 2>/dev/null | awk '{s+=$2} END{print s+0}')
    [ "$c" -gt "$prev" ] && { prev=$c; last=$now; echo "$w $((now-t0)) s collected $c temp $tc" >> "$OUT/progress.txt"; }
    if grep -q 'mushrooms collected (' "$D/part1_sim.log" 2>/dev/null; then res=complete; sleep 40; break; fi
    [ $((now-t0)) -ge 1800 ] && { res=cap; break; }
    [ $((now-last)) -ge 600 ] && { res=stall; break; }
    kill -0 $P 2>/dev/null || { res=sim_exited; break; }
  done
  sleep 2
  echo "$w result=$res collected=$prev wall_since_launch=$(( $(date +%s) - t0 )) start_temp=$startT peak_temp=$peak timer=$(tail -1 "$D/timer.out" | tr -d '\n')" >> "$OUT/results.txt"
  kill $T 2>/dev/null
  bash demo/stop_sim.sh > "$OUT/stop_after_$w.txt" 2>&1
  wait $P 2>/dev/null
done
echo "end $(date '+%F %T')" >> "$OUT/results.txt"
