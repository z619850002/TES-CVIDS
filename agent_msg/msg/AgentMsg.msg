Header header
int16 seq
geometry_msgs/Point position_imu
geometry_msgs/Quaternion orientation_imu
geometry_msgs/Point tic
geometry_msgs/Quaternion ric
#map point.
geometry_msgs/Point32[] point_3d
geometry_msgs/Point32[] point_uv
int32[] point_id
int64[] point_des
#2d points.
geometry_msgs/Point32[] feature_2d
int64[] feature_des
int32 publish_index




# points refer to window features, which contains 3D postion
# feature refer to fast feature in whole image
# point_2d point_uv feature_uv can be removed in the future

# to do list: distance resample
# tic ric when loading and saving
