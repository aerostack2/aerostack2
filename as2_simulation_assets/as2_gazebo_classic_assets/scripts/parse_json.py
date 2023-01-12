#!/usr/bin/python3

import json
import argparse

def get_drone(drone):
    try:
        model = drone['model']
    except KeyError as ex:
        if ex.args[0] != 'model':
            raise KeyError
        model = 'none'

    try:
        name = drone['name']
    except KeyError as ex:
        if ex.args[0] != 'name':
            raise KeyError
        name = 'none'    

    try:
        x, y, z, yaw = drone['pose']
    except KeyError as ex:
        if ex.args[0] != 'pose':
            raise KeyError
        x, y, z, yaw = 0, 0, 0, 0
        
    return f"{model}:{name}:{x}:{y}:{z}:{yaw}"

def get_object(object):
    try:
        model = object['model']
    except KeyError as ex:
        if ex.args[0] != 'model':
            raise KeyError
        model = 'none'

    try:
        name = object['name']
    except KeyError as ex:
        if ex.args[0] != 'name':
            raise KeyError
        name = 'none'    

    try:
        x, y, z, yaw = object['pose']
    except KeyError as ex:
        if ex.args[0] != 'pose':
            raise KeyError
        x, y, z, yaw = 0, 0, 0, 0
        
    return f"{model}:{name}:{x}:{y}:{z}:{yaw}"

def get_world(filepath):
    json_data = open(filepath)
    data = json.load(json_data)
    json_data.close()
    return data['world']


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Tool to parse simluation configuration file'
    )

    parser.add_argument('filepath', type=str, help='Filepath to config file')
    
    args = parser.parse_args()
    
    try:
        json_data = open(args.filepath)
        data = json.load(json_data)
        json_data.close()
    except FileNotFoundError:
        print("File not found.")
        exit(-1)
    
    world_str_ = ""
    try:
        world_str_ = get_world(args.filepath)
    except (KeyError, FileNotFoundError):
        world_str_="none"
    print(f"{world_str_}")
    
    try:
        drones = data['drones']
    except KeyError as ex:
        if ex.args[0] != 'drones':
            raise KeyError
        print("[]")
        exit(0)
    
    drone_str_ = ""
    for drone in drones:
        drone_str_ += f"{get_drone(drone)};"
    print(f"{drone_str_}")
    
    try:
        objects = data['objects']
    except KeyError as ex:
        if ex.args[0] != 'objects':
            raise KeyError
        print("[]")
        exit(0)
    
    object_str_ = ""
    for object in objects:
        object_str_ += f"{get_object(object)};"
    print(f"{object_str_}")
