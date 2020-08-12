# (31,1)
for f in ../benchmark/32x32_obst204/map_32by32_obst204_agents10_ex*; do
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents10_ex54.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex18.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex19.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex27.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex3.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex30.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex33.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex5.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex52.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex55.yaml" ]; then
        continue
    fi
        if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex58.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex61.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex70.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex72.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex73.yaml" ]; then
        continue
    fi
    # for new goal (31,1) (this point is not reachable)
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex74.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex76.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex84.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex88.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex92.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex94.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex96.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents30_ex10.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents30_ex12.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents30_ex13.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents30_ex15.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents30_ex20.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents30_ex29.yaml" ]; then
        continue
    fi
    if [ "$f" == "../benchmark/32x32_obst204/map_32by32_obst204_agents30_ex32.yaml" ]; then
        continue
    fi
    echo $f;
    # for t in {1..10}; do  
    #     ./cbs -i $f --approach pruning --agentNumber 3 --timeStep "$t" --x 14 --y 13; 
    # done
    ./cbs -i $f --approach pruning --agentNumber 3 --timeStep 5 --x 14 --y 13;
    done > my_results.txt
# (14,13) : 44 , 86