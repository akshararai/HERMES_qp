#!/bin/bash
xterm -leftbar -geometry 90x12+0+298 -bg thistle -fg black -title xmotor -e xmotor &
sleep 0.5
xterm -leftbar -geometry 90x8+0+164 -bg LightYellow -fg black -title xopengl -e xopengl &
sleep 0.5
xterm -leftbar -geometry 90x8+0+30 -bg LightGray -fg black -title xsimulation -e xsimulation &
sleep 0.5
#valgrind --leak-check=full --track-origins=yes --log-file=$LAB_ROOT/valgrind_log --max-stackframe=3873320 xtask
gdb xtask

