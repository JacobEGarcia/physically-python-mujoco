#!/usr/bin/env bash
# Fetch just the Unitree G1 model from google-deepmind/mujoco_menagerie.
set -e
cd "$(dirname "$0")"
if [ ! -d mujoco_menagerie ]; then
  git clone --depth 1 --filter=blob:none --sparse https://github.com/google-deepmind/mujoco_menagerie.git
  (cd mujoco_menagerie && git sparse-checkout set unitree_g1)
fi
ln -sfn ../mujoco_menagerie/unitree_g1/assets scene/assets
echo "Menagerie Unitree G1 model ready."
