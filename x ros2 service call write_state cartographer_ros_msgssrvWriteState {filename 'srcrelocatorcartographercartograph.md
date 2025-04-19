```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: 'src/relocator/cartographer/cartographer_config/pbstream/trajectory.pbstream', include_unfinished_submaps: true}"

ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=src/relocator/cartographer/cartographer_config/map/map.yaml

ros2 run nav2_map_server map_saver_cli -t map -f src/relocator/cartographer/cartographer_config/map/map
```

