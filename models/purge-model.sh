#!/bin/bash

# Purge a gazebo model

# Usage:
# ./purge-model.sh <model-name>

rm -Rf ~/.gazebo/models/$1/
