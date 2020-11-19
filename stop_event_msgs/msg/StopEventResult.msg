#StopEventPublisher
int64 stop_event
bool expected
string description
time event_time
time stop_time
duration stop_duration
int64 active_goal_id # What was executing when stop happened

# Trajectory data for the event
string[] joint_names
trajectory_msgs/JointTrajectoryPoint event_state
trajectory_msgs/JointTrajectoryPoint final_state
