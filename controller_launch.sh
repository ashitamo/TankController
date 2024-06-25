#!/usr/bin/env bash
#export python3=/home/hsun91chen/anaconda3/envs/py_ros/bin/python
#export python3 = python3
python3 heartbeam.py &
python3 sub_stall.py &
python3 sub_throttle.py &
python3 sub_steer.py &
