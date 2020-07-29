for f in ../benchmark/32x32_obst204/map_32by32_obst204_agents20_ex9*; do
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
    echo $f;  
    ./cbs -i $f --outputFileBase /home/hadar/My_Project_MAPF/debug/outputFileBase.yaml --outputFileNew /home/hadar/My_Project_MAPF/debug/outputFileNew.yaml --secondOutputFileBase /home/hadar/My_Project_MAPF/debug/secondOutputFileBase.yaml --secondOutputFileNew /home/hadar/My_Project_MAPF/debug/secondOutputFileNew.yaml --approach pruning --agentNumber 3 --timeStep 5 --x 1 --y 31; done > my_results.txt
