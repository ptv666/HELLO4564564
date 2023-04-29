#当前机器人状态
hit_spider/hexapod_Base_Pose base_Pose_Now
int8[6] support_State_Now
int8[6] faultLeg_State_Now
hit_spider/FeetPosition feetPositionNow

#下一步机器人状态
hit_spider/hexapod_Base_Pose base_Pose_Next
int8[6] support_State_Next
int8[6] faultLeg_State_Next
hit_spider/FeetPosition feetPositionNext

#移动方向
geometry_msgs/Point move_Direction

std_msgs/String remarks