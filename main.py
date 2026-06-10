#Moving the servo
ros2 topic pub --once /servo_controller servo_controller_msgs/msg/ServosPosition "{duration: 1.0, position_unit: 'deg', position: [{id: 4, position: 100.0}]}"
