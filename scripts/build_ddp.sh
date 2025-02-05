#!/bin/bash

killall -9 rosmaster

killall gzclient

killall gzserver

WORLD_IDX=$1

for j in {1..5} ; do    
    python run_ddp.py --world_idx $WORLD_IDX --out "out_ddp_v=1.5_ddp"
    sleep 2
done

