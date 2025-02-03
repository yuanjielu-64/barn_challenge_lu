#!/bin/bash

killall -9 rosmaster

killall gzclient

killall gzserver

for i in {271..1} ; do
    for j in {1..3} ; do            
        python run_ddp.py --world_idx $i --out "out_ddp_v=2_ddp"
        sleep 2
    done
done
