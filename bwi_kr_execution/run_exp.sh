#!/bin/bash
#xterm -e roscore &
sleep 5
konsole -e "roslaunch bwi_kr_execution environment_2d_krr2014.launch" &
sleep 15
konsole -e "roslaunch bwi_kr_execution bwi_kr_execution_simulation.launch --screen" &
sleep 15
konsole -e "rosrun bwi_tasks bwi_back_and_forth_node --screen" &



