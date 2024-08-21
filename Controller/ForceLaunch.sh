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

sleep 1.5

# Launch interfaces server using tmux
tmux new-session -d -s force_sensor "sudo python3 resources/bota_driver.py enx00e04c6856c6"
tmux send-keys -t force_sensor "kensalisbury" Enter

sleep 2.0

# launch controller
# force controller z
./force_controller_z_panda_gripper 1&

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
tmux send-keys -t force_sensor C-c

# Close the tmux session
tmux kill-session -t plotter
tmux kill-session -t force_sensor C-c

# Running stiffness calculation after controller has closed
cd resources
python3 calculateK.py

# sleep for everything to close
sleep 0.5
