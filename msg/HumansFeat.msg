# Message to send information about Humans detected

# id : The idenfification of the human
# x : Human x position in the /laser_top frame
# y : Human y position in the /laser_top frame
# distance : Distance between the human and the robot
# detectiontime : Starting human time detection

int16 id

float32 x
float32 y

float32 velx
float32 vely

float64 detectiontime
