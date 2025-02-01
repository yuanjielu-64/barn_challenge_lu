#!/bin/bash

killall -9 rosmaster

killall gzclient

killall gzserver

for i in {299..1} ; do
    for j in {1..1} ; do            
        python run_ddp.py --world_idx $i --out "out_ddp_v=1.5_ddp"
        sleep 2
    done
done

for i in {299..1} ; do
    for j in {1..1} ; do            
        python run_ddp.py --world_idx $i --out "out_ddp_v=1.5_ddp"
        sleep 2
    done


for i in {299..1} ; do
    for j in {1..1} ; do            
        python run_ddp.py --world_idx $i --out "out_ddp_v=1.5_ddp"
        sleep 2
    done
done