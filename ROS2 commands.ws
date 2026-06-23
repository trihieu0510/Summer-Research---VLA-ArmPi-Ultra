#Moving the servo
ros2 topic pub --once /servo_controller servo_controller_msgs/msg/ServosPosition "{duration: 1.0, position_unit: 'deg', position: [{id: 4, position: 100.0}]}"
ros2 topic list                    # what's being published?
ros2 topic echo <topic>            # see live messages
ros2 topic hz <topic>              # is it actually publishing, and how fast?
ros2 node list                     # what nodes are running?
ros2 service list                  # what services are callable?
ros2 launch <pkg> <file.launch.py> # start a system

ros2 topic list                            # all topics
ros2 topic list -t                         # with message types
ros2 topic info <topic>                    # who publishes, who subscribes
ros2 topic echo <topic>                    # stream every message (Ctrl+C to stop)
ros2 topic echo <topic> --once             # just one message
ros2 topic echo <topic> --no-arr           # hide huge arrays (for images)
ros2 topic hz <topic>                      # publishing rate
ros2 topic bw <topic>                      # bandwidth in bytes/sec
ros2 topic pub --once <topic> <type> "<yaml>"   # publish a one-shot message
ros2 topic pub --rate 1 <topic> <type> "<yaml>" # publish repeatedly at 1 Hz

ros2 service list                                  # all services
ros2 service list -t                               # with types
ros2 service type <service>                        # just the type
ros2 service call <service> <type> "<yaml>"        # call it

ros2 service call /object_sorting/enter std_srvs/srv/Trigger "{}"
ros2 service call /object_sorting/set_target interfaces/srv/SetStringBool "{data_str: 'red', data_bool: true}"

ros2 node list                            # running nodes
ros2 node info <node>                     # its pubs, subs, services, params

ros2 launch <pkg> <file.launch.py>                          # default args
ros2 launch <pkg> <file.launch.py> arg:=value               # override an arg
ros2 launch <pkg> <file.launch.py> display:=false           # your usual
ros2 launch -s <pkg> <file.launch.py>                       # show all available args

ros2 run <pkg> <executable>                                          # vanilla
ros2 run <pkg> <executable> --ros-args -p name:=value                # set a parameter
ros2 run <pkg> <executable> --ros-args -r /old_topic:=/new_topic     # remap a topic

ros2 pkg list                              # all installed packages
ros2 pkg prefix <pkg>                      # install path
ros2 pkg executables <pkg>                 # what can you `ros2 run`?
ros2 pkg executables                       # all executables across all packages

ros2 interface show <type>                 # show a message/service definition
ros2 interface list                        # all known interfaces
ros2 interface package <pkg>               

ros2 param list                                   # all params on all nodes
ros2 param list <node>                            # just that node
ros2 param get <node> <param>                     # read a value
ros2 param set <node> <param> <value>             # change it live (no restart)
ros2 param dump <node>                            # dump all to YAML# interfaces from a package

ros2 bag record <topic1> <topic2> ...      # save messages to disk
ros2 bag record -a                          # record everything
ros2 bag info <bag_dir>                    # inspect a recorded bag
ros2 bag play <bag_dir>                    # replay

ros2 doctor                                # check ROS2 install / network health
ros2 doctor --report                       # detailed version
ros2 daemon stop && ros2 daemon start      # nuke the discovery daemon

ros2 topic hz /depth_cam/rgb/image_raw
ros2 topic echo /servo_controller --once         # one message
ros2 topic echo /depth_cam/rgb/image_raw --no-arr --once   # skip the image data
ros2 interface show std_srvs/srv/Trigger
ros2 service call /object_sorting/enter std_srvs/srv/Trigger "{}"
ros2 pkg executables app | grep object_sorting
ros2 launch -s example color_sorting_node.launch.py
