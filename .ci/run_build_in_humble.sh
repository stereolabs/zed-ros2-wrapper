#!/bin/bash
pwd_path="$(pwd)"
if [[ ${pwd_path:${#pwd_path}-3} == ".ci" ]] ; then cd .. && pwd_path="$(pwd)"; fi
ttk="---> "
root_path=${pwd_path}
repo_name=${PWD##*/}

echo "${ttk} Root repository folder: ${root_path}"
echo "${ttk} Repository name: ${repo_name}"

sudocmd=""
if  [[ ! $(uname) == "MINGW"* ]]; then
	LINUX_OS=1
    if  [[ ! ${CI_RUNNER_TAGS} == *"docker-builder"* ]]; then
	    sudocmd="sudo "
    fi
fi

${sudocmd} chmod +x .ci/*.sh

#the . command.sh syntaxe allows env var to be accessible cross-scripts (needed for timers)

# Check Ubuntu version
ubuntu=$(lsb_release -r)
echo "${ttk} Ubuntu $ubuntu"
ver=$(cut -f2 <<< "$ubuntu")
echo "${ttk} Version: $ver"

# Build the node
cd "${pwd_path}"
if [[ $ver == "20.04" ]]; then 
	echo "${ttk} Build ROS2 Humble from the source."    
    . .ci/build_humble_src.sh    
fi
if [[ $ver == "22.04" ]]; then 
	echo "${ttk} Install ROS2 Humble from the binaries."
    . .ci/build_humble_bin.sh
fi
if [ $? -ne 0 ]; then echo "${ttk} ROS2 Node build failed" > "$pwd_path/failure.txt" ; cat "$pwd_path/failure.txt" ; exit 1 ; fi
