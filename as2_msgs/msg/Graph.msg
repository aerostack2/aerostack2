# Graph message definition
# 
# if distance is nan, nodes not connected
# if distance is 0, nodes at the same position
# else distance in meters between nodes

as2_msgs/Node[] nodes       # Nodes in graph
float64[] distances         # Relationship matrix between nodes