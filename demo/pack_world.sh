#!/bin/bash
# Maintainer: packs the river demo worlds and only the models they use into ammo_river_worlds.tar.gz,
# the asset attached to the GitHub release. bash demo/pack_world.sh [OUT_DIR]
set -e
F=${AMMO_FOREST:-$(cd "$(dirname "$0")/../forest_worlds" && pwd)}
OUT=$(cd "${1:-.}" && pwd)/ammo_river_worlds.tar.gz
cd "$F"
W=$(ls worlds/river_moderate_aus{,_dt2,_m1,_m2,_m3}.{sdf,meta.json})
M=$(cat $(echo "$W" | grep "sdf$") | grep -o 'model://[A-Za-z0-9_]*' | sort -u | sed 's#model://##' |
    while read m; do ls -d models/*/$m world_assets/$m 2>/dev/null || true; done)
tar czf "$OUT" $W $M
echo "$OUT ($(du -h "$OUT" | cut -f1))"
