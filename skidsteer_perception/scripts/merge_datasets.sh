#!/usr/bin/env bash
# Combine two or more captured datasets into one training set.
#
#   ./merge_datasets.sh <output_dir> <input_dir> [input_dir ...]
#   ./merge_datasets.sh ~/mushroom_sim_combined ~/mushroom_sim_dataset ~/mushroom_sim_dataset_air
#
# Refuses on a filename collision rather than overwriting: two frames with the
# same name means one set silently replaces the other's labels, and the only
# symptom is a model that trains fine and detects nothing.
set -euo pipefail

[ $# -ge 2 ] || { echo "usage: $0 <output_dir> <input_dir> [input_dir ...]"; exit 1; }

OUT="$1"; shift

rm -rf "$OUT"
for split in train val; do mkdir -p "$OUT/images/$split" "$OUT/labels/$split"; done

for src in "$@"; do
    [ -d "$src/images/train" ] || { echo "not a dataset: $src"; exit 1; }
    for split in train val; do
        for img in "$src/images/$split"/*.jpg; do
            [ -e "$img" ] || continue
            stem=$(basename "$img" .jpg)
            if [ -e "$OUT/images/$split/$stem.jpg" ]; then
                echo "collision: $stem already present. Recapture with a"
                echo "different platform name, or rename before merging."
                exit 1
            fi
            cp "$img" "$OUT/images/$split/"
            cp "$src/labels/$split/$stem.txt" "$OUT/labels/$split/"
        done
    done
    echo "  merged $(basename "$src")"
done

cp "$1/data.yaml" "$OUT/data.yaml"

echo
for split in train val; do
    printf "%-6s %s imgs / %s labels\n" "$split" \
        "$(ls "$OUT/images/$split" | wc -l)" "$(ls "$OUT/labels/$split" | wc -l)"
done
echo "-> $OUT"
