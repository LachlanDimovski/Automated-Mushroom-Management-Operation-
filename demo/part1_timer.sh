#!/bin/bash
# Run beside part1_fleet.sh: wall time from the sim clock starting to "All N mushrooms collected".
D=$(cd "$(dirname "$0")" && pwd); source "$D/env.sh" || exit 1
W=${1:-${WORLD:-river_moderate_aus_dt2}}
st() { timeout 20 ign topic -e -n 1 -t /world/$W/stats 2>/dev/null | python3 -c "import sys,re;m=re.search(r'sim_time \{\s*(?:sec: (\d+))?',sys.stdin.read());print(int(m.group(1) or 0) if m else 0)"; }
until [ "$(st)" -gt 0 ] 2>/dev/null; do sleep 2; done; t0=$(date +%s); echo "sim clock started $(date +%T)"
until grep -q "mushrooms collected (" "$LOGS/part1_sim.log" 2>/dev/null; do sleep 3; done
echo "$(grep -o 'All [0-9]* mushrooms collected' "$LOGS/part1_sim.log" | tail -1) after $(( $(date +%s) - t0 )) s from sim start, sim $(st) s"
