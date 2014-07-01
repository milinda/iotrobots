#! /bin/bash

python takeoff.py &
sleep 10
killall -9 python
python controller.py