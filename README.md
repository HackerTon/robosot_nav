# robosot_nav
Tune dwaplannerros, amcl and costmap parameter for turtlebot3 burger in robosot competition.

## Instruction
Replace YOURMAP with something like /home/ubuntu/map.yaml
1) roslaunch rosot_nav turtlebot3_navigation.launch map_file:=YOURMAP
2) rosrun rosot_nav point
3) rosrun rosot_nav navigation

### TODO
- Push ball to boal

#### citation
@misc{zheng2017ros,
    title={ROS Navigation Tuning Guide},
    author={Kaiyu Zheng},
    year={2017},
    eprint={1706.09068},
    archivePrefix={arXiv},
    primaryClass={cs.RO}
}