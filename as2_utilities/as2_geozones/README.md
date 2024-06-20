# Geozones

Geozones node for Aerostack2.

Params:
- config_file: Polygons that define geofences are defined here. (IMPORTANT) Remember that the order in which points are given defines how the polygon is built, this means that n point will be conected to n+1 point and so on. Last point will be connected to the first point.
Config file parameters:

    - **id** (int): geozone id 
    - **alert** (int): generated alert
    - **type** (string): geofence or geocage. Geofence will generate alert on entering the area, geocage will generate alert on exiting the area. 
    - **data_type** (string): gps or cartesian. 
    - **polygon** (list(2DPoint)): List of 2D points that forms the polygon. Should at least be 3. Order matters. 
    - **z_up** (float): up limit, always in cartesian.
    - **z_down** (float): down limit, always in cartesian.

Example config file:

    {
        "geozones": [
            {
                "id": 1,
                "alert": 1,
                "type": "geofence",
                "data_type": "gps",
                "polygon": [
                    [
                        52.172046,
                        4.416790
                    ],
                    [
                        52.172046,
                        4.415912
                    ],
                    [
                        52.171506,
                        4.415912
                    ],
                    [
                        52.171506,
                        4.416789
                    ]
                ],
                "z_up": 100.0,
                "z_down": 90.0
            }
        ]
    }

- Interfaces:

    Available Services:

    - ```/set_geozone```: Set a geoestructure vía message.
    - ```/get_geozone```: Get a list of all geoestructures available

    Published topics:
    - ```/alert_event```: Publish the defined alert message when a geoestructure event is triggered.
