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
	if [[ $pathfile == "none" ]]; then
		return
	fi

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
		name=${UAV_NAME:="none"}
		x=${UAV_X:="0.0"}
		y=${UAV_Y:="0.0"}
		z=${UAV_Z:="0.0"}
		yaw=${UAV_YAW:="1.57"}
		drones_array="${model}:${name}:${x}:${y}:${z}:${yaw}"
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
    name=$3
    x=$4
    y=$5
    z=$6
    Y=$7
	capacity=$8
	sensors=${@:9}  # All next arguments
	
	N=${N:=0}
	model=${model:=""}
	name=${name:=""}
	x=${x:=0.0}
	y=${y:=$((3*${N}))}
	z=${z:=0.1}
	Y=${Y:=0.0}
	capacity=${capacity:=""}
	sensors=${sensors:=""}

	if [ "$model" == "" ] || [ "$model" == "none" ]; then
		echo "empty model, setting iris as default"
		model="$DEFAULT_UAV_MODEL"
	fi

	if [ "$name" == "" ] || [ "$name" == "none" ]; then
		name=${AEROSTACK2_SIMULATION_DRONE_ID::-1}${N}
		echo "empty name, setting ${name} as default"
	fi

	world_name=${world_path##*/}
	world_name=${world_name%.*}

	target="${model}/${model}.sdf"
	modelpath="$(get_path ${target} ${IGN_GAZEBO_RESOURCE_PATH})"
    DIR_SCRIPT="${0%/*}"
    python3 ${DIR_SCRIPT}/jinja_gen.py ${modelpath}/${target}.jinja ${modelpath}/.. --namespace "${name}" --sensors "${sensors}" --battery "${capacity}" --output-file /tmp/${model}_${N}.sdf

    ros2 run ros_gz_sim create -world ${world_name} -file /tmp/${model}_${N}.sdf -name "${name}" -x $x -y $y -z $z -Y $Y
}

function start_ign_server() {
    world=$1
	world=$(eval echo $world)

	# Check if ENV VAR is set
	if [ "$world" == "none" ] && [[ -n "$UAV_WORLD" ]]; then
		world="$UAV_WORLD"
	fi

	# Check if world file exist, else look for world
	if [[ -f $world ]]; then
		world_path="$world"
	else
		target="${world}.sdf"
		world_path="$(get_path ${target} ${IGN_GAZEBO_RESOURCE_PATH})"
	fi

	# Check if world_path exist, else empty
	if [[ -d $world_path ]]; then
		world_path="${world_path}/${target}"
	else
		echo "empty world, setting empty.sdf as default"
		world_path="empty.sdf"
	fi

    ign gazebo -s $run_on_start $verbose $world_path &
	SERVER_PID=$!
}

function start_ign_client() {
	ign gazebo -g >/dev/null 2>/dev/null &
	CLIENT_PID=$!
}

function spawn_drones() {
	drones=$1
	num_vehicles=${#drones[@]}

	drones=${drones:="none"}

	n=0
	while [ $n -lt $num_vehicles ]; do
		local drone_array=()
		parse_drone_config ${drones[$n]} drone_array
		echo Spawn: ${drone_array[*]}

		spawn_drone_model $n ${drone_array[*]}
		n=$(($n + 1))
	done
}

function create_world_bridges() {
	ros2 launch ignition_assets world_bridges.py
	BRIDGE_PID=$!
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
config_path=${config_path:="none"}

# RUN ON START
if [[ -n "$RUN_ON_START" ]]; then
	run_on_start="-r"
else
	run_on_start=""
fi

# VERBOSE OUTPUT
if [[ -n "$VERBOSE_SIM" ]]; then
	verbose="-v 4"
else
	verbose=""
fi

setup

# Parse config file
declare world_path drones
parse_config_script $config_path world_path drones
echo drones: ${drones[*]}
echo world_path: $world_path

start_ign_server $world_path
sleep 1

spawn_drones $drones

start_ign_client

# ZOMBIE NODE NOT KILLED PROPERLY IF NOT LAUNCHED LAST
create_world_bridges

kill -9 ${BRIDGE_PID}
kill -9 ${CLIENT_PID}
kill -9 ${SERVER_PID}