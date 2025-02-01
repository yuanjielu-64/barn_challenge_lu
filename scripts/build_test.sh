#!/bin/bash

killall -9 rosmaster

killall gzclient

killall gzserver
 

WORLD_IDX=$1

python run_.py --world_idx $WORLD_IDX --gui false

