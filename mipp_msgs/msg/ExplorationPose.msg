# Message containing a possible exploration pose and the related information gain
float64 gain                # Value of the pose, usually derived from how much information is gained at the pose
geometry_msgs/Pose pose     # The pose of the robot to receive the afformentioned gain
int64 pose_rank