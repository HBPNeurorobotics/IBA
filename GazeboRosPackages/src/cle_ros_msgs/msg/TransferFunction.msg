# This message type contains structured information on transfer functions
string name
string code
int32 type
int32 ROBOT2NEURON=1
int32 NEURON2ROBOT=2
int32 NEURONMONITOR=3
cle_ros_msgs/Device[] devices
cle_ros_msgs/Topic[] topics
cle_ros_msgs/Variable[] variables