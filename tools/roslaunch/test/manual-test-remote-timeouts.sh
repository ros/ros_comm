#!/usr/bin/env bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

if [ $# -lt 5 ] || [ "$1" == "--help" ] || [ "$1" == "-h" ]; then
    echo "Usage: manual-test-remote-timeouts.sh sigint_timeout sigterm_timeout address env_loader user [roslaunch_timeout]"
    echo "Run this script set up to connect to a remote machine (or your own one, if you have self-ssh enabled), and after a while, break the program with Ctrl-C."
    echo "Observe, if SIGINT and SIGTERM are issued approximately after the time you have given in sigint_timeout and sigterm_timeout"
    echo "Make sure the remote machine also has the same version of ros_comm as this one!"
    exit 1
fi

sigint=$1
sigterm=$2
address=$3
env_loader=$4
user=$5
if [ $# -gt 6 ]; then
    timeout=$6
else
    timeout=10.0
fi

bold="\033[1m"
normal="\033[0m"
echo -e "${bold}A while after you see '... done launching nodes', break the program with Ctrl-C."
echo -e "Observe, if SIGINT and SIGTERM are issued approximately after ${sigint} and ${sigterm} seconds"
echo -e "Make sure the remote machine also has the same version of ros_comm as this one!${normal}"

sleep 5

"${THIS_DIR}/../scripts/roslaunch" --sigint-timeout ${sigint} --sigterm-timeout ${sigterm} \
    "${THIS_DIR}/xml/manual-test-remote-timeouts.launch" \
    address:=${address} env_loader:=${env_loader} user:=${user} timeout:=${timeout}