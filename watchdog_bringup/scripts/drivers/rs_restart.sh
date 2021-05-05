#!/bin/sh
# this bash script adheres to linux exit codes
# see https://tldp.org/LDP/abs/html/exitcodes.html for further information

# connect via ssh to pi with realsense
# TODO(scm)

# restart realsense hub
#   install uhubctl then I put in /etc/rc.local to power cycle usb hub:
#   uhubctl -a cycle -l 1 -p 1-4
#   uhubctl -a cycle -l 2 -p 1-4
sudo uhubctl -h

exit 0
