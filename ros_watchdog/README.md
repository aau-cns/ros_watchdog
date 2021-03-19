# ros_watchdog

The watchdog can observe both, topics and sensors. Those have to be specified in in INI-files.
These files again can be specified as rosparam which is used during initialization. 

The watchdog offers a service to get the current status and advertise a topic `/status` of type `SystemStatus` of the `autonomy_msgs` package.
It listens to the ROS service `/status_service`.

## License
This software is made available to the public to use (_source-available_), licensed under the terms of the BSD-2-Clause-License with no commercial use allowed, the full terms of which are made available in the `LICENSE` file. No license in patents is granted.


## Topic INI-file

```commandline
; watchdog_action
;    NONE = 0                    # -> OK
;    WARNING = 1                 # -> OK
;    ERROR = 2                   # -> ABORT
;    RESTART_ROSNODE= 3          # -> HOLD
;    RESTART_SENSOR = 4          # -> HOLD

[mav/imu]
rate = 100
watchdog_action = 2
timeout = 1
node_name=/mav
sensor_name=px4
```

Each section is defined via the full topic name and constains the minimum rate, the timeout at the start up phase of the node, the node name, the name of the related hardware sensor and which action the watchdog should perform.

Currently there are 5 actions: `NONE` does nothing, just some verbose information is displayed. `WARNING` will show a verbose waring info on rosout. `ERROR` will give an error message and changes the output state of the ros_watchdog.
`RESTART_ROSNODE` kills the rosnode. If it was lauchned using the `relaunch=yes` flag it will be restarted automatically. 
`RESTART_SENSOR` runs a script specified in the corresponding section of in the `sensor_cfg_file`.

## Node INI-file

```commandline
[/mav]
max_restart_attempts=0
restart_timeout=1
```

Each section is defined by the full node name and contains the number of allowed restart attempts and the desired timout after starting the node again.

## Sensor INI-file

```commandline
[ids_camera]
restart_script=../../restart_ids_camera.sh
restart_attempts=2

```

Each section is defined by the sensor name (Important must match with Topic  INI-file).
The `restart_script` entry specifies the script to be run, if the watchdog_action `RESTART_SENSOR` is active.
The `restart_attempts` specifies how many time the sensor can be restarted in a row.

