#!/bin/bash
# Installs the demo river worlds and their models (about 130 MB) into forest_worlds/ from the GitHub release.
#   bash demo/get_world.sh                 download with the GitHub CLI (gh auth login first; the repo is private)
#   bash demo/get_world.sh FILE.tar.gz     use a copy downloaded from the repo's Releases page
# The alternative is building every world from source with forest_worlds/setup.sh (internet, Blender >= 4.2).
set -e
F=$(cd "$(dirname "$0")/../forest_worlds" && pwd)
NAME=ammo_river_worlds.tar.gz
if [ -n "$1" ]; then
    T=$1
else
    D=$(mktemp -d)
    gh release download --repo "${AMMO_REPO:-LachlanDimovski/AMMO-Mushroom-Fleet}" --pattern $NAME --dir "$D"
    T=$D/$NAME
fi
tar xzf "$T" -C "$F"
ls "$F"/worlds/river_moderate_aus*.sdf
echo "Worlds installed in $F/worlds"
