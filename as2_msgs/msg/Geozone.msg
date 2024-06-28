# GeoStructure defined by an id, alert that generates in case of event and a polygon 
# that defines the area.

int8 id                            # geofence id
int8 alert                         # alert generated
string type                        # geofence or geocage
string data_type                   # cartesian or gps
geometry_msgs/Polygon polygon      # fence polygon
float32 z_up                       # height limit up
float32 z_down                     # height limit bottom
