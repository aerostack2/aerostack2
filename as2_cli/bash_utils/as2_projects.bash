#!/bin/bash
PROJECTS_URL="https://raw.githubusercontent.com/aerostack2/aerostack2/main/installers/projects.repos"

verbose=0
function check_init(){
  # echo -e "\n${yellowColour}[*]${endColour}${grayColour} Checking internet connection...\n${endColour}"
  if [ -z "$AEROSTACK2_PROJECTS" ]; then
    echo -e "\n${redColour}[!]${endColour}${grayColour} AEROSTACK2_PATH environment variable is not set.\n${endColour}"
    exit 1
  fi

  ping -c 1 -W 1 google.com > /dev/null 2>&1
  if [ $? -eq 0 ]; then
    data=$(curl -s $PROJECTS_URL | grep -E 'projects' -A 3 | tr -d " ") 
    if [ -z "$data" ]
    then
      echo -e "\n${redColour}[!]${endColour}${grayColour} No projects config file can be found.\n${endColour}"
      exit 1
    fi
    projects=$(cat <<< $data | grep  'projects'| awk 'NF{print $NF}' FS='/'| tr -d ':')
    urls=$(cat <<< $data | grep  'url' | sed 's/url://g')
    versions=$(cat <<< $data | grep  'version' | awk 'NF{print $NF}' FS=':')
    n_projects=$(cat <<< $projects | wc -l)
    n_url=$(cat <<< $urls | wc -l)
    n_version=$(cat <<< $versions | wc -l)

    if [[ ! $n_projects -eq $n_url  || ! $n_projects -eq $n_version ]] ; then
      echo "Error: projects.repos file is not correct"
      exit 1
    fi
    
  else
    echo -e "\n${redColour}[-]${endColour}${grayColour} Internet connection is not working.\n${endColour}"
    exit 1
  fi
}

function list_projects(){
  echo -e "\n${yellowColour}[*]${endColour}${grayColour} Listing projects...\n${endColour}"
    for i in $(seq 1 $n_projects); do
        project=$(cat <<< $projects | awk 'NR=='$i'{print $1}')
        url=$(cat <<< $urls | awk 'NR=='$i'{print $1}')
        version=$(cat <<< $versions | awk 'NR=='$i'{print $1}')
        if [ $verbose -eq 0 ]; then
          echo -e "$blueColour$i$endColour: $project $yellowColour[$version]$endColour"
        else 
          echo -e "$blueColour$i$endColour: $project $yellowColour[$version]$endColour -> $url" #, url : $url"
        fi
    done
  echo ""
}

function helpPanel(){
  echo -e "\nUtil for installing Aerostack2 projects from github\n"
  echo -e "\t${redColour}[-l]${endColour}  List available projects"
  echo -e "\t${redColour}[-h]${endColour}  Show this help"
  echo -e "\t${redColour}[-i]${endColour}  Install projects by id"
  echo -e "\t${redColour}[-n]${endColour}  Install projects by name"
  echo -e "\t${redColour}[-v]${endColour}  Verbose mode"
  echo ""
  exit 0
}

function check_if_project_exists(){
  project=$1
  if [ -d "$AEROSTACK2_PROJECTS$project" ]; then
    return 1
  fi
  return 0
}

function install_project(){
  project_id=$1
  project=$(cat <<< $projects | head -n $project_id | tail -n 1)
  url=$(cat <<< $urls | head -n $project_id | tail -n 1)
  version=$(cat <<< $versions | head -n $project_id | tail -n 1)

  if [[ $project_id -gt $n_projects || $project_id -lt 1 ]]; then
    echo -e "${redColour}[!]${endColour}${grayColour} Project id $project_id is not available.${endColour}"
    exit 1
  else
    echo -e "\n${yellowColour}[*]${endColour} Installing project ${grayColour}$project${endColour} ...\n"
    check_if_project_exists $project
    if [ $? -eq 1 ]; then
      echo -e "${redColour}[!]${endColour}${grayColour} Project $project already exists.${endColour}\n"
      return 1
    fi
    cmd="git clone --branch $version $url $AEROSTACK2_PROJECTS$project"
    echo $cmd
    $cmd
  fi
}

function install_by_id(){
  echo -e "\n${yellowColour}[*]${endColour}${grayColour} Installing projects by id...\n${endColour}"
  ids=$(cat <<< $1 | tr ',' ' ')
  for i in $ids; do
    if [[ $i -gt $n_projects || $i -lt 1 ]]; then
      echo -e "${redColour}[!]${endColour}${grayColour} Project id $i is not available.${endColour}"
      continue 
      # exit 1
    else
      install_project $i
    fi
  done
}


function install_by_name(){
  echo -e "\n${yellowColour}[*]${endColour}${grayColour} Installing projects by name...\n${endColour}"
  names=$(cat <<< $1 | tr ',' ' ')
  # compare each name with projects
  for i in $names; do
    project_found=0
    for j in $(seq 1 $n_projects); do
      project=$(cat <<< $projects | head -n $j | tail -n 1)
      if [[ $i == $project ]]; then
        # echo -e "${yellowColour}[*]${endColour}${grayColour} Project $i is available.${endColour}"
        project_found=1
        install_project $j
        break
      fi
    done
    if [[ $project_found -eq 0 ]]; then
      echo -e "${redColour}[!]${endColour}${grayColour} Project $i is not available.${endColour}"
    fi
  done
}

check_init
parameter_enable=0

while getopts "vli:n:h" arg; do
		case $arg in
      v) verbose=1;;
			l) list_projects ; let parameter_enable+=2;;
      i) install_by_id $OPTARG ;let parameter_enable+=1;;
      n) install_by_name $OPTARG ;let parameter_enable+=1;;
			h) helpPanel;;
		esac
	done

if [ $parameter_enable -eq 0 ]; then
  echo -e "\n${redColour}[*]${endColour}${grayColour} No option specified${endColour}"
  helpPanel
fi


