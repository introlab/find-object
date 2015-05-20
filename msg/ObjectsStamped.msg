# objects format: 
# [ObjectId1, objectWidth, objectHeight, h11, h12, h13, h21, h22, h23, h31, h32, h33, ObjectId2...] 
# where h## is a 3x3 homography matrix (h31 = dx and h32 = dy, see QTransform)
Header header
std_msgs/Float32MultiArray objects 