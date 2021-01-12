uint8 id

geometry_msgs/Point position
float64 yaw
float64 euc_dist_travelled
float64 yaw_dist_travelled

int8 FSM_STATE_INIT=-1
int8 FSM_STATE_IDLE=0
int8 FSM_STATE_PLANNING=1
int8 FSM_STATE_MOVING=2
int8 FSM_STATE_RECOVERING=3
int8 FSM_STATE_DONE=4
int8 fsm_state

uint8 COM_STATE_WORKING=0
uint8 COM_STATE_RANGE_BROKEN=1
uint8 COM_STATE_LOS_BROKEN=2
uint8 COM_STATE_RANGE_LOS_BROKEN=3
uint8 com_state