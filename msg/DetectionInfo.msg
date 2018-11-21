
Header header

# All arrays should have the same size
std_msgs/Int32[] ids
std_msgs/Int32[] widths
std_msgs/Int32[] heights
std_msgs/String[] filePaths
std_msgs/Int32[] inliers
std_msgs/Int32[] outliers
# 3x3 homography matrix: [h11, h12, h13, h21, h22, h23, h31, h32, h33] (h31 = dx and h32 = dy, see QTransform)
std_msgs/Float32MultiArray[] homographies
