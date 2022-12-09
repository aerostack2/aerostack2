#!/bin/bash

export package_list=();
export verbose=0;

function list_packages(){
    packages_cmakelist=$(find $AEROSTACK2_PATH -name "CMakeLists.txt"| sed -e 's/\/CMakeLists.txt//g' | sort -u)
    
    for package in $packages_cmakelist; do
        add_package=1
        # check if packages_cmakelist path includes a package.xml file and doesnt have a COLCON_IGNORE file in any of its parents folders
        if [ -f $package/package.xml ]; then
            path=$package
            # while path is not root
            while [ "$path" != "$AEROSTACK2_WORKSPACE" ]; do
                # check if COLCON_IGNORE file exists in path
                if [ -f $path/COLCON_IGNORE ]; then
                    add_package=0
                    break
                fi
                path=$(dirname $path)
            done
            if [ $add_package -eq 1 ]; then
                if grep -q "ament" $package/package.xml; then
                    project_name=$(grep "<name>" $package/package.xml | sed -e 's/<name>//g' | sed -e 's/<\/name>//g' | tr -d '\n'| tr -d ' '| tr -d '\t' | tr -d '\r')
                    package_list+=($project_name%%%$package)
                fi
            fi
        fi
    done
    
    #sort package_list by the first element separated by %%%
    package_list=($(printf '%s\n' "${package_list[@]}"|sort))
    
    echo -e "\n${yellowColour}[*]${endColour}${greenColour} List of packages:\n${endColour}"
    for i in "${!package_list[@]}"; do
        name=${package_list[$i]%%%*}
        path=${package_list[$i]##*%}
        if [ $verbose -eq 0 ]; then
            echo -e "${greenColour}[$i]${endColour}${grayColour} ${name}${endColour}"
        else
            echo -e "${greenColour}[$i]${endColour}${grayColour} ${name}${endColour} -> ${path}"
        fi
    done
}

function list_projects(){
  project_list={};
  if [ -d "$AEROSTACK2_PROJECTS" ]; then
  project_list=$(find "$AEROSTACK2_PROJECTS" -maxdepth 1 -mindepth 1 -type d | sort -u)
  project_list=($(printf '%s\n' "${project_list[@]}"|sort))
  fi 
  echo -e "\n${yellowColour}[*]${endColour}${greenColour} List of projects:\n${endColour}"
  for i in "${!project_list[@]}"; do
    name=${project_list[$i]##*/}
    path=${project_list[$i]##*%}
    if [ $verbose -eq 0 ]; then
      echo -e "${greenColour}[$i]${endColour}${grayColour} ${name}${endColour}"
    else
      echo -e "${greenColour}[$i]${endColour}${grayColour} ${name}${endColour} -> ${path}"
    fi
  done

}

function usage(){
    echo -e "\nUtil for listing ROS2 packages inside Aerostack 2\n"
    echo -e "\t${redColour}[-h , --help ] ${endColour}  Show this help"
    echo -e "\t${redColour}[-v , --verbose ]${endColour} Verbose output showing the paths of the packages"
    echo -e "\t${redColour}[-p , --plain ] ${endColour}  Disable colour output"
    echo -e "\t${redColour}[--list-format ] ${endColour}  List packages in a different format"
    echo ""
    exit 0
}

plain=0
list_format=0
list_projects=0

for opt in "${OPTS_ARGS[@]}"; do
    # check if the option ends with spaces and remove them
    opt="${opt%\ }"
    case $opt in
        -h | --help )
            usage
            exit 1
        ;;
        -v | --verbose)
        verbose=1;;
        -p | --plain)
        plain=1;;
        --list-format)
            list_format=1
        plain=1;;
        --projects)
            list_projects=1;;
        -* | --* )
            echo "invalid option: $opt"
            usage
            exit 1
        ;;
    esac;
done

# if plain then substitute all colors with empty string
if [ $plain -eq 1 ]; then
    greenColour=""
    endColour=""
    redColour=""
    blueColour=""
    yellowColour=""
    purpleColour=""
    turquoiseColour=""
    grayColour=""
fi

cmd="list_packages"
if [ $list_projects -eq 1 ]; then
    cmd="list_projects"
fi

if [ $list_format -eq 1 ]; then
    if [ $verbose -eq 0 ]; then
        $cmd | grep -vE 'List|Arg' | awk '{print($2)}' | sed -r '/^\s*$/d'| sort -u | tr '\n' ' '
    else
        $cmd | grep -vE 'List|Arg' | awk '{print($2,$4)}' | sed -r '/^\s*$/d'| sort -u | tr '\n' ' '
    fi
else
    $cmd
fi
