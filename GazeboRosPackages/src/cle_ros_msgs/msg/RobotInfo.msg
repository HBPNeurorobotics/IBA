string robot_id                 #
string robot_model              # if robot is custom, it says the name of the custom model template,
                                # if robot is not custom,it is the path to the template SDF
bool is_custom                  # If true, robot_rel_path points to a custom zip (optional)
                                # Otherwise, it points to a template SDF
cle_ros_msgs/Pose pose          # Euler representation of the robot pose (optional)
                                # Defaults to <0,0,0,0,0,0> or first robot's pose
