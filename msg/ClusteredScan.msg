# A message to contain all the info and the points of each laser cluster of the scan
# clusters : the actual points of the cluster
# cogs	 : the center of gravity of each cluster
# features: the extracted features for each cluster
# label: annotation for the label
# fusion : if the cluster can be fused with camera information (if the bounding box is within the image)
# projected : if cluster is projected onto image
# laser : the laser scan  --> laser scan and image are saved here to have everything in place 
#         and send it over network more easily in the future
# image : the image capture

Header header
sensor_msgs/LaserScan scan
sensor_msgs/Image image
sensor_msgs/PointCloud[] clusters
geometry_msgs/Point32[] cogs
std_msgs/Float32MultiArray[] features
int8[] labels #annotation labels
bool[] fusion
bool[] projected
uint16 nclusters
uint16 nfeatures
int8[] detection_labels 
float32[] detection_probs
