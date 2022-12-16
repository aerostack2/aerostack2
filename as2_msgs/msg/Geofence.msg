# Geofence defined by an id, alert that generates in case of event and a polygon 
# that defines the fence.

int8 id                            # geofence id
int8 alert                         # alert severity
string type                        # exclusion or inclusion
geometry_msgs/Polygon polygon       # fence polygon
