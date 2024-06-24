# PRO-LAB

## Getting Started

### Installing

xhost +local:

Turtlebot Keyboard Control:
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

Auswahl des Turtlebots:
export TURTLEBOT3_MODEL=burger

# Git:

## Initial Clone:

git clone --recurse-submodules <Repository-URL> (git clone --recurse-submodules https://github.com/KevinEppacher/Probabilistic_Lab.git)

git submodule init

git submodule update

## Commit and Push:

git add .

git commit -m "Was geändert wurde"

git push

git checkout -b particle_filter

git push -u origin particle_filter

Terminal Commands:

roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch