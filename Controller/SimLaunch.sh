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

# Launch interfaces server using tmux
tmux new-session -d -s plotter "python3 resources/PlotAll.py"

sleep 1.0

# launch controller
# ./controller_panda_gripper 1&

# launch force controller
# ./force_controller_z_panda_gripper 1&

# launch automated force controller
# ./force_controller_z_automated 1&
# ./force_controller_z_automated_prototype 1&

# launch joint test controller
./joint_test_controller

# moment testing controller
# ./plate_controller_panda_gripper 1&

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

# Once Controller main program dies, kill plotter
tmux send-keys -t plotter C-c

# Close the tmux session
tmux kill-session -t plotter

# sleep for everything to close
sleep 0.5
