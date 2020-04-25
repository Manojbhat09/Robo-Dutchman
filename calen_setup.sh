#!etc/bin/bash
source devel/setup.sh
catkin_make
tmux new-session -s calen
tmux new-window -d -n ArmPlanner
tmux new-window -d -n git
tmux new-window -d
tmux a calen
