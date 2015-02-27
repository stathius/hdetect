# A message to contain all the info and the points of each laser cluster of the scan
# clusters : the actual points of the cluster
# cog : the center of gravity of each cluster
# features: the extracted features for each cluster
# cog_projected : if image is projected onto image
# image_projected : if cog is projected onto image
# detection_laser_prob : the laser scan probability
# detection_camera_prob : the laser scan probability
# detection_fusion_prob : the laser scan probability
# label: annotation for the label

sensor_msgs/PointCloud clusters
geometry_msgs/Point32 cog
std_msgs/Float32MultiArray features

bool cog_projected
bool crop_projected

int8 detection_label
float32 detection_laser_prob
float32 detection_camera_prob
float32 detection_fusion_prob

int8 label
