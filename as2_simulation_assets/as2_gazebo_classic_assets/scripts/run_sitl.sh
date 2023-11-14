#!/usr/bin/env bash

# TODO function variable into local variable
# TODO external MAYUSC internal minusc

function cleanup() {
	pkill -x px4  || true
	pkill gzclient
	pkill gzserver
	pkill -9 gz
}

function setup_gazebo() {
    [ -d "/usr/share/gazebo-7" ] && export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-7
    [ -d "/usr/share/gazebo-9" ] && export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-9
    [ -d "/usr/share/gazebo-11" ] && export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-11

    echo -e "GAZEBO_RESOURCE_PATH $GAZEBO_RESOURCE_PATH"
}

function parse_config_script() {
	pathfile=$1

	DIR_SCRIPT="${0%/*}"

	array=()
	while read line; do
		array+=($line)
	done < <(${DIR_SCRIPT}/parse_json.py $pathfile)

	local -n world=$2
	local -n drones_=$3
	local -n objects_=$4

    world=${array[0]}
	drones_=${array[1]}
	objects_=${array[2]}

	if [[ ${#drones_[@]} -eq 0 ]]; then
		model=${UAV_MODEL:="iris"}
		x=${UAV_X:="0.0"}
		y=${UAV_Y:="0.0"}
		z=${UAV_Z:="0.0"}
		yaw=${UAV_YAW:="1.57"}
		drones_="${model}:${x}:${y}:${z}:${yaw}"
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

function run_gzserver() {
	world=$1
	world=$(eval echo $world)

	if [ "$world" == "none" ] && [[ -n "$PX4_SITL_WORLD" ]]; then
		world="$PX4_SITL_WORLD"
	fi

	# Check if world file exist, else look for world
	if [[ -f $world ]]; then
		world_path=${world%/*}
		target=${world##/*/}
	else
		target="${world}.world"
		world_path="$(get_path ${target} ${GAZEBO_RESOURCE_PATH})"
	fi

	# Check if world_path exist, else empty
	if [[ -d $world_path ]]; then
		world_path="${world_path}/${target}"
	else
		echo "empty world, setting empty.world as default"
		world_path="${src_path}/Tools/simulation/gazebo-classic/sitl_gazebo/worlds/empty.world"
	fi

	# To use gazebo_ros ROS2 plugins
	if [[ -n "$ROS_VERSION" ]] && [ "$ROS_VERSION" == "2" ]; then
		ros_args="-s libgazebo_ros_init.so -s libgazebo_ros_factory.so --ros-args --params-file ${AS2_GZ_ASSETS_SCRIPT_PATH}/config.yaml"
	else
		ros_args=""
	fi

	echo "Starting gazebo server"
	echo "gzserver ${verbose} ${world_path} ${ros_args}"
	gzserver $verbose $world_path $ros_args &
	SIM_PID=$!
}

function run_gzclient() {
	px4_follow_mode=$1
	# Disable follow mode
	if [[ $px4_follow_mode -eq 1 ]]; then
		# FIXME: follow_mode not working
		follow_mode_="--gui-client-plugin libgazebo_user_camera_plugin.so"
	else
		follow_mode_=""
	fi

	# gzserver needs to be running to avoid a race. Since the launch
	# is putting it into the background we need to avoid it by backing off
	sleep 3

	echo "Starting gazebo client"
	nice -n 20 gzclient $verbose $follow_mode_  # &  # TODO fails on headless
	GUI_PID=$!
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

function spawn_model() {
	N=$1
    model=$2
    name=$3
    x=$4
    y=$5
    z=$6
    Y=$7
	
	N=${N:=0}
	model=${model:=""}
	name=${name:=""}
	x=${x:=0.0}
	y=${y:=$((3*${N}))}
	z=${z:=0.1}
	Y=${Y:=0.0}

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
	modelpath="$(get_path ${target} ${GAZEBO_MODEL_PATH})"
	DIR_SCRIPT="${0%/*}"

	python3 ${DIR_SCRIPT}/jinja_gen.py ${modelpath}/${target}.jinja ${modelpath}/.. --mavlink_tcp_port $((4560+${N})) --mavlink_udp_port $((14560+${N})) --mavlink_id $((1+${N})) --gst_udp_port $((5600+${N})) --video_uri $((5600+${N})) --mavlink_cam_udp_port $((14530+${N})) --output-file /tmp/${model}_${N}.sdf

	gz model $verbose --spawn-file="/tmp/${model}_${N}.sdf" --model-name=${model}_${N} -x ${x} -y ${y} -z ${z} -Y ${Y} 2>&1
}

function spawn_object() {
	N=$1
    model=$2
    name=$3
    x=$4
    y=$5
    z=$6
    Y=$7
	
	N=${N:=0}
	model=${model:=""}
	name=${name:=""}
	x=${x:=0.0}
	y=${y:=$((3*${N}))}
	z=${z:=0.1}
	Y=${Y:=0.0}

	if [ "$model" == "" ] || [ "$model" == "none" ]; then
		echo "empty object model"
		return
	fi

	if [ "$name" == "" ] || [ "$name" == "none" ]; then
		name=${model}
		echo "empty name, setting ${name} as default"
	fi

	world_name=${world_path##*/}
	world_name=${world_name%.*}

	target="${model}/model.sdf"
	modelpath="$(get_path ${target} ${GAZEBO_MODEL_PATH})"
	DIR_SCRIPT="${0%/*}"

	gz model $verbose --spawn-file="${modelpath}/${target}" --model-name=${name} -x ${x} -y ${y} -z ${z} -Y ${Y} 2>&1
}

function run_sitl() {
	N=$1
	N=${N:=0}
	vehicle=$2
	vehicle=${vehicle:=""}

	NO_PXH=1
	# To disable user input
	if [[ -n "$NO_PXH" ]]; then
		no_pxh=-d
	else
		no_pxh=""
	fi

	# FIXME: VEHICLE --> IRIS
	vehicle=iris
	if [[ -n "$vehicle" ]]; then
		export PX4_SIM_MODEL=gazebo-classic_${vehicle}
	else
		export PX4_SIM_MODEL=gazebo-classic_iris
	fi

	working_dir="$build_path/tmp/sitl_${N}"
	mkdir -p "$working_dir"
	pushd "$working_dir" >/dev/null

	sitl_command="\"$sitl_bin\" -i $N $no_pxh \"$build_path\"/etc -s etc/init.d-posix/rcS -w $working_dir $headless"
	test_test___="\"$sitl_bin\" -i $N $no_pxh \"$build_path\"/etc -s etc/init.d-posix/rcS -w sitl_${MODEL}_${N} >out.log 2>err.log &"
	# TODO verbose

	echo SITL COMMAND: $sitl_command
	eval $sitl_command

	popd >/dev/null
}

function spawn_drones() {
	drones=$1
	num_vehicles=${#drones[@]}

	n=0
	while [ $n -lt $num_vehicles ]; do
		local drone_array=()
		parse_drone_config ${drones[$n]} drone_array
		echo Spawn: ${drone_array[*]}

		spawn_model $n ${drone_array[*]}
		run_sitl $n $VEHICLE
		n=$(($n + 1))
	done
}

function spawn_objects() {
	objects=$1
	num_objects=${#objects[@]}

	n=0
	while [ $n -lt $num_objects ]; do
		local objects_array=()
		parse_drone_config ${objects[$n]} objects_array
		echo Spawn: ${objects_array[*]}

		spawn_object $n ${objects_array[*]}
		n=$(($n + 1))
	done
}

set -e

if [ "$#" -lt 4 ]; then
	echo "usage: $0 sitl_bin config_path src_path build_path"
	exit 1
fi

if [ ! -x "$(command -v gazebo)" ]; then
	echo "You need to have gazebo simulator installed!"
	exit 1
fi

sitl_bin="$1"
config_path="$2"
src_path="$3"
build_path="$4"

echo SITL ARGS

echo sitl_bin: $sitl_bin
echo config: $config_path
echo src_path: $src_path
echo build_path: $build_path

# Parse config file
declare world_path drones_str objects_str
parse_config_script $config_path world_path drones_str objects_str

declare drones objects
IFS=';' read -r -a drones <<< "$drones_str"
IFS=';' read -r -a objects <<< "$objects_str"

echo world_path: $world_path
echo drones: $drones
echo objects: $objects

# Follow mode only available with one drone
if [[ ${#drones[@]} -gt 1 ]]; then
	follow_mode=""
else
	follow_mode=$PX4_FOLLOW_MODE
fi

# To disable user input
if [[ -n "$VERBOSE_SIM" ]]; then
	verbose="--verbose"
else
	verbose=""
fi

# To run PX4 in foregroud
if [[ -n "$HEADLESS" ]]; then
	headless=""
else
	headless="&"
fi

# kill process names that might stil
# be running from last time
echo "killing running instances"
trap "cleanup" SIGINT SIGTERM EXIT

sleep 1

source "${build_path}/build_gazebo-classic/setup.sh"
source "$src_path/Tools/simulation/gazebo-classic/setup_gazebo.bash" "${src_path}" "${build_path}"
setup_gazebo

run_gzserver $world_path
sleep 2

# Do not exit on failure now from here on because we want the complete cleanup
set +e
spawn_drones $drones
spawn_objects $objects

if [[ -n "$HEADLESS" ]]; then
	echo "not running gazebo gui"
else
	run_gzclient $follow_mode
fi

kill -9 $SIM_PID
if [[ ! -n "$HEADLESS" ]]; then
	kill -9 $GUI_PID
fi
cleanup
