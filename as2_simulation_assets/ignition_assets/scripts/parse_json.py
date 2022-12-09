#!/usr/bin/python3

import json
import argparse


def get_battery_capacity(flight_time):
    # UAV specific, sets flight time

    # calculate battery capacity from time
    # capacity (Ah) = flight time (in hours) * load (watts) / voltage
    # assume constant voltage for battery to keep things simple for now.
    battery_capacity = (float(flight_time) / 60) * 6.6 / 12.694
    return battery_capacity


def get_drone(drone):
    model = 'none'
    if 'model' in drone:
        model = drone['model']

    name = 'none'
    if 'name' in drone:
        name = drone['name']

    x, y, z = 0, 0, 0
    if 'xyz' in drone:
        x, y, z = drone['xyz']

    roll, pitch, yaw = 0, 0, 0
    if 'rpy' in drone:    
        roll, pitch, yaw = drone['rpy']

    capacity = 0
    if "flight_time" in drone:
        flight_time = drone['flight_time']
        capacity = get_battery_capacity(flight_time)

    payload = ""
    if 'payload' in drone:
        for sensor_name, sensor in drone['payload'].items():
            if 'sensor' not in sensor:
                continue
            sensor_type = sensor['sensor']

            x_s, y_s, z_s = 0, 0, 0
            if 'xyz' in sensor:
                x_s, y_s, z_s = sensor['xyz']

            roll_s, pitch_s, yaw_s = 0, 0, 0
            if 'rpy' in sensor:
                roll_s, pitch_s, yaw_s = sensor['rpy']

            payload += f":{sensor_name}:{sensor_type}:{x_s}:{y_s}:{z_s}:{roll_s}:{pitch_s}:{yaw_s}"
        
    return f"{model}:{name}:{x}:{y}:{z}:{yaw}:{capacity}{payload}"

def main(filepath):
    try:
        json_data = open(filepath)
        data = json.load(json_data)
        json_data.close()
    except FileNotFoundError:
        print("File not found.")
        exit(-1)

    world = 'none'
    if 'world' in data:
        world = data['world']

    drones = "none:none:0:0:0:0"
    if 'drones' in data:
        drones = data['drones']
    else:
        return world, drones

    drone_ = []
    for drone in drones:
        drone_ += [get_drone(drone)]
    
    return world, drone_

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Tool to parse simluation configuration file'
    )

    parser.add_argument('filepath', type=str, help='Filepath to config file')
    args = parser.parse_args()

    world, drones = main(args.filepath)
    print(world)
    for drone in drones:
        print(drone)
