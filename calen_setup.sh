#!etc/bin/bash
source devel/setup.sh
catkin_make
tmux new-seesion -s calen
tmux new-window -d -n exe
tmux new-window -d -n git
tmux new-window -d
tmux a calen
