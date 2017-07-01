#!/bin/bash

# Create a box gazebo model

# Usage:
# Place the texture on the same directory as the script, then:
# ./create-model-flat.sh <model-name> <texture> <w> <l> <h> <static> <texture scale w> <texture scale l>

mkdir ~/.gazebo/models/$1/
mkdir ~/.gazebo/models/$1/materials/
mkdir ~/.gazebo/models/$1/materials/scripts
mkdir ~/.gazebo/models/$1/materials/textures

# model.config 
cat > ~/.gazebo/models/$1/model.config <<- EOM
<?xml version="1.0"?>
<model>
    <name>$1</name>
    <version>1.0</version>
    <sdf version="1.4">model.sdf</sdf>
</model>
EOM

# model.sdf
cat > ~/.gazebo/models/$1/model.sdf <<- EOM
<?xml version="1.0" ?>
<sdf version="1.4">
    <model name="$1">
        <static>$6</static>
        <link name="link">
            <inertial>
                <mass>0.01</mass>
            </inertial>
            <collision name="collision">
                <geometry>
                    <box>
                        <size>$3 $4 $5</size>
                    </box>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>0.02</mu>
                            <mu2>0.02</mu2>
                        </ode>
                    </friction>
                </surface>
            </collision>
            <visual name="visual">
                <cast_shadows>1</cast_shadows>
                <geometry>
                    <box>
                        <size>$3 $4 $5</size>
                    </box>
                </geometry>
                <material>
                    <script>
                        <uri>model://$1/materials/scripts</uri>
                        <uri>model://$1/materials/textures</uri>
                        <name>$1</name>
                    </script>
                </material>
            </visual>
        </link>
    </model>
</sdf>
EOM

# script.material
cat > ~/.gazebo/models/$1/materials/scripts/script.material <<- EOM
material $1
{
        receive_shadows on
        technique
        {
                pass
                {
                        texture_unit
                        {
                                texture $2
                                filtering anistropic
                                max_anisotropy 16
                                scale $7 $8
                        }
                }
        }
}
EOM

# Texture
cp $2 ~/.gazebo/models/$1/materials/textures/

# script.material
cat <<- EOM
<model name='$1'>
    <include>
        <uri>model://$1</uri>
    </include>
    <pose>0 0 0 0 0 0</pose>
</model>
EOM

