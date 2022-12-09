#!/usr/bin/python3

import json
import argparse

def get_x_drone(filepath, n_drone):
    json_data = open(filepath)
    data = json.load(json_data)
    json_data.close()
    model=data[str(n_drone)]['model']
    x=data[str(n_drone)]['pose'][0]
    y=data[str(n_drone)]['pose'][1]
    z=data[str(n_drone)]['pose'][2]
    yaw=data[str(n_drone)]['pose'][3]
    return f"{model}:{x}:{y}:{z}:{yaw}"

def get_world(filepath):
    json_data = open(filepath)
    data = json.load(json_data)
    json_data.close()
    return data['world']

# DEPRECATED
def get_num_drones(filepath):
    json_data = open(filepath)
    data = json.load(json_data)
    json_data.close()
    return data['num_drones']

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Tool to parse simluation configuration file'
    )

    parser.add_argument('filepath', type=str, help='Filepath to config file')
    
    args = parser.parse_args()
    
    try:
        world = get_world(args.filepath)
    except (KeyError, FileNotFoundError):
        world="none"

    print(world)

    # num = get_num_drones(args.filepath)
    # print(num)

    i = 0
    while True:
        try:
            drone = get_x_drone(args.filepath, i)
            print(drone)
        except (KeyError, FileNotFoundError):
            break
        i += 1