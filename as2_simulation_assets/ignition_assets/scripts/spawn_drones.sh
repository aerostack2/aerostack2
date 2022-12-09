#!/usr/bin/env bash

DEFAULT_UAV_MODEL="quadrotor_base"

function setup() {
	AS2_MODELS="${AEROSTACK2_PATH}/simulation/ignition_assets/models"
	AS2_WORLDS="${AEROSTACK2_PATH}/simulation/ignition_assets/worlds"

	export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$AS2_MODELS:$AS2_WORLDS
}

function get_path() {
	target=$1
	path_list=$2

	# Check all paths in path_list for specified model
	IFS_bak=$IFS
	IFS=":"
	for possible_path in ${path_list}; do
		if [ -z $possible_path ]; then
			continue
		fi
		# trim \r from path
		possible_path=$(echo $possible_path | tr -d '\r')
		if test -f "${possible_path}/${target}" ; then
			path=$possible_path
			break
		fi
	done
	IFS=$IFS_bak

	echo $path
}

function parse_config_script() {
	pathfile=$1

	DIR_SCRIPT="${0%/*}"

	array=()
	while read line; do
		array+=($line)
	done < <(${DIR_SCRIPT}/parse_json.py $pathfile)

	local -n world=$2
	local -n drones_array=$3

	world=${array[0]}
	drones_array=("${array[@]:1}")  # removed the 1st element

	if [[ ${#drones_array[@]} -eq 0 ]]; then
		model=${UAV_MODEL:="none"}
		x=${UAV_X:="0.0"}
		y=${UAV_Y:="0.0"}
		z=${UAV_Z:="0.0"}
		yaw=${UAV_YAW:="1.57"}
		drones_array="${model}:${x}:${y}:${z}:${yaw}"
	fi
}

function parse_drone_config() {
	drone_config=$1

	local -n array=$2
	IFS_bak=$IFS
	IFS=":"
	for val in ${drone_config}; do
		array+=($val)
	done
	IFS=$IFS_bak
}

function spawn_drone_model() {
    N=$1
    model=$2
    x=$3
    y=$4
    z=$5
    Y=$6
	sensors=${@:7}  # All next arguments
	
	N=${N:=0}
	model=${model:=""}
	x=${x:=0.0}
	y=${y:=$((3*${N}))}
	z=${z:=0.1}
	Y=${Y:=0.0}
	sensors=${sensors:=""}

	if [ "$model" == "" ] || [ "$model" == "none" ]; then
		echo "empty model, setting iris as default"
		model="$DEFAULT_UAV_MODEL"
	fi

	world_name=${world_path##*/}
	world_name=${world_name%.*}

	target="${model}/${model}.sdf"
	modelpath="$(get_path ${target} ${IGN_GAZEBO_RESOURCE_PATH})"
    DIR_SCRIPT="${0%/*}"
    python3 ${DIR_SCRIPT}/jinja_gen.py ${modelpath}/${target}.jinja ${modelpath}/.. --namespace "${AEROSTACK2_SIMULATION_DRONE_ID::-1}${N}" --sensors "${sensors}" --output-file /tmp/${model}_${N}.sdf

    ros2 run ros_gz_sim create -world ${world_name} -file /tmp/${model}_${N}.sdf -name ${AEROSTACK2_SIMULATION_DRONE_ID::-1}${N} -x $x -y $y -z $z -Y $Y
}

function spawn_drones() {
	drones=$1
	num_vehicles=${#drones[@]}

	n=0
	while [ $n -lt $num_vehicles ]; do
		local drone_array=()
		parse_drone_config ${drones[$n]} drone_array
		echo Spawn: ${drone_array[*]}

		spawn_drone_model $n ${drone_array[*]}
		n=$(($n + 1))
	done
}

# ------- MAIN -------

if [ "$#" -lt 0 ]; then
	echo "usage: $0 [config_path] "
	exit 1
fi

if [ ! -x "$(command -v ign)" ]; then
	echo "You need to have ignition gazebo simulator installed!"
	exit 1
fi

config_path="$1"

setup

# Parse config file
declare world_path drones
parse_config_script $config_path world_path drones
echo drones: ${drones[*]}
echo world_path: $world_path

spawn_drones $drones
