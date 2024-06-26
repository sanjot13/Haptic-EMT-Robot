#!/bin/bash
if [ ! "$BASH_VERSION" ] ; then
    exec /bin/bash "$0" "$@"
fi

# launch redis server is not launched
if [ -z "$(pgrep redis-server)" ]; then
	redis-server &
	REDIS_PID=$!
fi

sleep 0.2

cd ./bin/panda_gripper_example

# Launch haply driver using tmux
tmux new-session -d -s haply "python3 resources/haply_driver_with_force_grid.py"
# tmux new-session -d -s haply "python3 resources/haply_driver.py"

sleep 1.0

# Launch plotter using tmux
tmux new-session -d -s plotter "python3 resources/PlotAll.py"

sleep 1.0

# launch controller
./controller_stiffness_map 1&
CONTROLLER_MAIN_PID=$!

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

function ctrl_c() {
    kill -2 $CONTROLLER_MAIN_PID
}

sleep 1

# Wait for Controller main program to quit and stop redis
wait $CONTROLLER_MAIN_PID
if [ ! -z "$REDIS_PID" ]; then
	kill -2 $REDIS_PID
fi

# Once Controller main program dies, kill all other python scripts
tmux send-keys -t plotter C-c
tmux send-keys -t haply C-c

# Close all other tmux sessions
tmux kill-session -t plotter
tmux kill-session -t haply

# sleep for everything to close
sleep 0.5
