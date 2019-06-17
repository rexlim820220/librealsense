#!/bin/bash
PATH="$PATH":/home/user/bin
echo Start.
#cat 654.pcd > d.pcd
#exec pointcloud.cpp and generate d55.pcd
#n="grep "POINTS\s+\K\w+" d55.pcd"
n=1449239
sed -i 's/736780/'"$n"'/g' e504.pcd
more +12 e55.pcd >> e504.pcd
echo Done.
