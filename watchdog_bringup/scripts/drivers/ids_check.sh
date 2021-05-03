#!/bin/bash
# this bash script adheres to linux exit codes
# see https://tldp.org/LDP/abs/html/exitcodes.html for further information

echo "[BASH] checking sensor - FAILURE"
exit 1

IDS_STATUS="$(systemctl status ueyeusbdrc)"
echo "$IDS_STATUS"

case $IDS_STATUS in
  "")
    echo "[BASH - IDS] Service cannot be found"
    exit 0
    ;;
esac

exit 0
