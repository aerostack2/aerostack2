# Geofence defined by an id, alert that generates in case of event and a polygon 
# that defines the fence.

int32 id                            # geofence id
int32 alert                         # alert severity
geometry_msgs/Polygon polygon       # fence polygon
