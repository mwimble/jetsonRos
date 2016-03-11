#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Usage: env.sh COMMANDS"
  /bin/echo "Calling env.sh without arguments is not supported anymore. Instead spawn a subshell and source a setup file manually."
  exit 1
fi

# ensure to not use different shell type which was set before
CATKIN_SHELL=sh

# source setup.sh from same directory as this file
#echo "dollar 0 is $0"
#export
_CATKIN_SETUP_DIR="/home/ubuntu/catkin_ws/devel"
. "/home/ubuntu/.bashrc"
. "/home/ubuntu/catkin_ws/devel/setup.sh"
exec "$@"
