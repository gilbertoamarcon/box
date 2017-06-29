#!/bin/bash

# Small Box
./purge-model.sh small_box
./create-model-box.sh small_box crate.png 0.38 0.38 0.25 0 1 1

# Big Box
./purge-model.sh big_box
./create-model-box.sh big_box crate.png 0.457 0.457 0.457 1 1 1

# Low Walls
./purge-model.sh low_wall
./create-model-box.sh low_wall wall.jpg 8 0.1 0.5 1 0.125 0.5

# Low pillars
./purge-model.sh low_pillar
./create-model-box.sh low_pillar wall.jpg 0.1 0.1 0.5 1 5 0.5

# Floor
./purge-model.sh floor
./create-model-flat.sh floor floor.jpg 8 8 0.125 0.125
