#!/bin/bash

usage() {
    echo "  options:"
    echo "      -n: select drones namespace to launch, values are comma separated. By default, it will get all drones from config file"
    echo "      -m: select mode (hokuyo, optitrack, gazebo). Default is hokuyo"
}

# Initialize variables with default values
tb2_namespace_comma=""
mode="hokuyo"  # Default mode

# Arg parser
while getopts "n:m:" opt; do
  case ${opt} in
    n )
      tb2_namespace_comma="${OPTARG}"
      ;;
    m )
      if [[ "${OPTARG}" =~ ^(hokuyo|optitrack|gazebo|teleop|keyboard)$ ]]; then
        mode="${OPTARG}"
      else
        echo "Invalid mode: ${OPTARG}. Valid options are: hokuyo, optitrack, gazebo, keyboard" >&2
        exit 1
      fi
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      echo "Option -$OPTARG requires an argument" >&2
      usage
      exit 1
      ;;
  esac
done

# If no drone namespaces are provided, use just tb2_unizar
if [ -z "$tb2_namespace_comma" ]; then
  tb2_namespace_comma="tb2_unizar"
fi
IFS=',' read -r -a tb2_namespaces <<< "$tb2_namespace_comma"

# Set the tmuxinator YAML file based on the selected mode
tmuxinator_yaml="tmuxinator/launch_turtlebot_${mode}.yaml"

# Launch tb2 for each drone namespace
for namespace in ${tb2_namespaces[@]}; do
  base_launch="false"
  if [[ ${namespace} == ${tb2_namespaces[0]} ]]; then
    base_launch="true"
  fi
  eval "tmuxinator start -n ${namespace} -p ${tmuxinator_yaml} \
    tb_namespace=${namespace} \
    wait"

  sleep 0.1 # Wait for tmuxinator to finish

done

# Attach to tmux session
tmux attach-session -t ${tb2_namespaces[0]}