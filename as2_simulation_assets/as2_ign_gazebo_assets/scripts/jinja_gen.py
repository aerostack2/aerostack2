#!/usr/bin/env python3

"""Render assets file (SDF) from jinja template

:raises argparse.ArgumentTypeError: Not used
:raises FileNotFoundError: Template not found
:raises OverwriteForbidden: Overwrite done in forbidden location/file.
:return: Rendered template SDF
"""

import os
import argparse
import shutil
import jinja2


class OverwriteForbidden(Exception):
    """Overwrite not allowed"""


def get_namespace():
    """Get namespace"""
    return os.getenv('AEROSTACK2_SIMULATION_DRONE_ID', default='drone_sim')


def get_file_contents(filepath):
    """Get file content"""
    with open(filepath, 'rb') as file:
        return file.read()


def str2bool(value):
    """String to bool"""
    if value.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    if value.lower() in ('no', 'false', 'f', 'n', '0'):
        return False

    raise argparse.ArgumentTypeError('Boolean value expected.')


def get_sensors(sensors_array):
    """Get sensors from payload"""
    sensors = []
    while sensors_array and sensors_array[0]:
        name = sensors_array.pop(0)
        model = sensors_array.pop(0)
        pose, sensors_array = sensors_array[:6], sensors_array[6:]

        sensors.append({'name': name, 'model': model,
                        'pose': f'{pose[0]} {pose[1]} {pose[2]} {pose[3]} {pose[4]} {pose[5]}'})
    return sensors


def get_origin(origin_array):
    """Get GPS origin"""
    origin = {}

    use_origin = False
    if len(origin_array) == 3:
        origin["latitude"] = float(origin_array.pop(0))
        origin["longitude"] = float(origin_array.pop(0))
        origin["altitude"] = float(origin_array.pop(0))
        use_origin = True

    return origin, use_origin


def main():
    """Entrypoint"""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'filename', help="file that the sdf file should be generated from")
    parser.add_argument('env_dir')
    parser.add_argument('--output-file', help="sdf output file")
    parser.add_argument('--stdout', action='store_true',
                        default=False, help="dump to stdout instead of file")
    parser.add_argument('--namespace', default=get_namespace(),
                        help="Drone ROS namespace")
    parser.add_argument('--origin', default='',
                        help='Set world origin values: lat, lon, alt')
    parser.add_argument('--sensors', default='', help="Drone model sensors")
    parser.add_argument('--no-odom', action='store_false',
                        dest="odom", help="Disable odometry plugin on model")
    parser.add_argument('--battery', dest='bat_capacity', default=0.0,
                        help='Enable battery plugin on model with given capacity')
    args = parser.parse_args()
    env = jinja2.Environment(loader=jinja2.FileSystemLoader(args.env_dir))
    template = env.get_template(os.path.relpath(args.filename, args.env_dir))

    sensors = get_sensors(str(args.sensors).split(sep=' '))

    origin, use_origin = get_origin(str(args.origin).split(sep=' '))

    dict_ = {'namespace': args.namespace, 'sensors': sensors, 'odom_plugin': args.odom,
             'battery_plugin': bool(float(args.bat_capacity)), 'capacity': float(args.bat_capacity),
             'origin': origin, 'use_origin': use_origin}
    result = template.render(dict_)

    if args.stdout:
        print(f"{result=}")
    else:
        if args.output_file:
            filename_out = args.output_file
        else:
            if not args.filename.endswith('.sdf.jinja'):
                raise FileNotFoundError("ERROR: Output file can only be determined automatically " +
                                        "for input files with the .sdf.jinja extension")
            filename_out = args.filename.replace('.sdf.jinja', '.sdf')
            assert filename_out != args.filename, "Not allowed to overwrite template"

        # Overwrite protection mechanism: after generation, the file will be copied
        # to a "last_generated" file. In the next run, we can check whether the target
        # file is still unmodified.
        filename_out_last_generated = filename_out + '.last_generated'

        if os.path.exists(filename_out) and os.path.exists(filename_out_last_generated):
            # Check whether the target file is still unmodified.
            if get_file_contents(filename_out).strip() != get_file_contents(filename_out_last_generated).strip():
                raise OverwriteForbidden(f"ERROR: generation would overwrite changes to `{filename_out}`. " +
                                         f"Changes should only be made to the template file `{args.filename}`. " +
                                         f"Remove `{os.path.basename(filename_out)}` " +
                                         "(after extracting your changes) to disable this overwrite protection.")

        with open(filename_out, 'w', encoding='utf-8') as f_out:
            print(f'{args.filename} -> {filename_out}')
            f_out.write(result)

        # Copy the contents to a "last_generated" file for overwrite protection check next time.
        shutil.copy(filename_out, filename_out_last_generated)


if __name__ == "__main__":
    main()
