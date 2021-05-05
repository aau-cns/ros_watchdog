#!/bin/sh
# this bash script adheres to linux exit codes
# see https://tldp.org/LDP/abs/html/exitcodes.html for further information
echo "[BASH - RS] checking sensor - Realsense"

# connect via ssh to pi with realsense
# TODO(scm)

RS_ID="8087:0b37"
RS_STATUS="$(lsusb | grep ${RS_ID})"
echo "[BASH - RS] status: ${RS_STATUS}"

case $RS_STATUS in
  "")
    echo "[BASH - RS] device cannot be found"
    exit 1
    ;;
esac

# checks successful
exit 0
