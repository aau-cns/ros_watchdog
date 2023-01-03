# CNS Flight Stack: ROS1 Watchdog

[![License](https://img.shields.io/badge/License-AAUCNS-336B81.svg)](./LICENSE) [![Paper](https://img.shields.io/badge/IEEEXplore-10.1109/LRA.2022.3196117-00629B.svg?logo=ieee)](https://doi.org/10.1109/LRA.2022.3196117) [![Release](https://img.shields.io/github/v/release/aau-cns/ros_watchdog?include_prereleases&logo=github)](https://github.com/aau-cns/ros_watchdog/releases)

The ros_watchdog observers ROS1 topics and nodes, as well as system-defined drivers. Its settings are specified in INI-
files, which can be specified as ROS1 parameters used during initalization.

Maintainer: [Martin Scheiber](mailto:martin.scheiber@aau.at)

## Credit
This code was written by the [Control of Networked System (CNS)](https://www.aau.at/en/smart-systems-technologies/control-of-networked-systems/), University of Klagenfurt, Klagenfurt, Austria.

## License
This software is made available to the public to use (_source-available_), licensed under the terms of the BSD-2-Clause-License with no commercial use allowed, the full terms of which are made available in the `LICENSE` file. No license in patents is granted.

### Usage for academic purposes
If you use this software in an academic research setting, please cite the
corresponding paper and consult the `LICENSE` file for a detailed explanation.

```latex
@article{cns_flightstack22,
    title        = {CNS Flight Stack for Reproducible, Customizable, and Fully Autonomous Applications},
    author       = {Scheiber, Martin and Fornasier, Alessandro and Jung, Roland and Böhm, Christoph and Dhakate, Rohit and Stewart, Christian and Steinbrener, Jan and Weiss, Stephan and Brommer, Christian},
    journal      = {IEEE Robotics and Automation Letters},
    volume       = {7},
    number       = {4},
    year         = {2022},
    doi          = {10.1109/LRA.2022.3196117},
    url          = {https://ieeexplore.ieee.org/document/9849131},
    pages        = {11283--11290}
}
```

---

## Getting Started

### Prerequesites
This package is part of the [CNS FlightStack] and thus depends on the other packages of the flight stack:
- [CNS FlightStack: Autonomy Engine]

Further the following libraries are required
- Python3
- ROS noetic

### Build

As this is a ROS package, please build it within the catkin environment with

```bash
catkin build watchdog_bringup # will automatically build ros_watchdog
```

## Usage

The intended usage is together with the [CNS FlightStack: Autonomy Engine], which will interact with the watchdog. Use the provided launchfile to start the watchdog

```bash
roslaunch watchdog_bringup watchdog.launch 
```

### Default Launchfile Parameters

| Launch parameter | description | default value |
|:----------------:|:-----------:|:-------------:|
| `node_name`           | name of the node to be launched | `watchdog` |
| `status_o`            | out topic where status is published | `<node_name>/status` |
| `status_service_o`    | in service where status can be requested | `<node_name>/status` |
| `start_service_o`     | in service where watchdog is activated | `<node_name>/start` |
| `entity_check_rate`   | rate at which entities are checked (in Hz) | `2.0` |
| `drivers_cfg_file`    | config file for drivers or other scripts | `<watchdog_bringup>/config/drivers.ini` |
| `topics_cfg_file`     | config file for ROS1 topics | `<watchdog_bringup>/config/topics.ini` |
| `nodes_cfg_file`      | config file for ROS1 nodes | `<watchdog_bringup>/config/nodes.ini` |
| `do_verbose`          | enable verbose flag | `false` |

### Usage without Autonomy Engine

If required the watchdog can be used without the [CNS FlightStack: Autonomy Engine]. 
The easiest way to do this is to start the watchdog with the provided service: 

```bash
rosservice call /watchdog/service/start "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
startup_time: 10.0" # any time >(2/<rate>) should be provided here 

# feedback as Status.msg
header: 
  seq: 1
  stamp: 
    secs: <current_secs>
    nsecs: <current_nsecs>
  frame_id: "ros_watchdog"
successful: True|False
status: 
  entity: ''
  type: 0
  status: 0|1|2|4|8
  name: "/watchdog/global"
  info: "
    Status ObserverKeys.TOPIC: <no entries to watch>|0|1|2|4|8
    Status ObserverKeys.NODE: <no entries to watch>|0|1|2|4|8
    Status ObserverKeys.DRIVER: <no entries to watch>|0|1|2|4|8"
entity_ids: [...]
```

The status can be one of the following and is defined in the [`Status.msg`](watchdog_msgs/msg/Status.msg)
- `0`: undefined condition
- `1`: nominal condition
- `2`: startup phase (while the service call is active only, i.e., the duration `startup_time`)
- `4`: inconvenient failure (but topic/node/driver is still active)
- `8`: severe failure (no communication of topic/node/driver)

## Architecture

Please refer to the [academic paper] for further insights of the ros_watchdog

## Known Issues

None at this point in time, please submit an issue request using the git interface if any issues arise.

## Package Layout

```[console]
/path/to/ros_watchdog$ tree -L 3 --noreport --charset unicode
.
|-- LICENSE
|-- README.md
|-- ros_watchdog
|   |-- CMakeLists.txt
|   |-- LICENCE
|   |-- nodes
|   |   `-- WatchdogNode.py
|   |-- package.xml
|   |-- README.md
|   |-- setup.py
|   `-- src
|       |-- observer
|       `-- watchdog
|-- watchdog_bringup
|   |-- CMakeLists.txt
|   |-- config
|   |   |-- drivers.ini
|   |   |-- nodes.ini
|   |   `-- topics.ini
|   |-- launch
|   |   `-- watchdog.launch
|   |-- package.xml
|   `-- scripts
|       `-- drivers
`-- watchdog_msgs
    |-- action
    |   `-- HandleError.action
    |-- CMakeLists.txt
    |-- LICENCE
    |-- msg
    |   |-- Action.msg
    |   |-- ActionStamped.msg
    |   |-- StatusChangesArray.msg
    |   |-- StatusChangesArrayStamped.msg
    |   |-- Status.msg
    |   `-- StatusStamped.msg
    |-- package.xml
    |-- README.md
    `-- srv
        |-- Reset.srv
        `-- Start.srv
```

---

Copyright (C) on changes 2021-2023 Roland Jung, Martin Scheiber, Alessandro Fornasier, and Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
You can contact the authors at [roland.jung@aau.at](mailto:roland.jung@aau.at?subject=[CNS%20Flight%20Stack]%20ros_watchdog%20package), [martin.scheiber@aau.at](mailto:martin.scheiber@aau.at?subject=[CNS%20Flight%20Stack]%20ros_watchdog%20package), [alessandro.fornasier@aau.at](mailto:alessandro.fornasier@aau.at?subject=[CNS%20Flight%20Stack]%20ros_watchdog%20package), [christian.brommer@aau.at](mailto:christian.brommer@aau.at?subject=[CNS%20Flight%20Stack]%20ros_watchdog%20package).

<!-- LINKS: -->
[CNS FlightStack]: https://github.com/aau-cns/flight_stack
[CNS FlightStack: Autonomy Engine]: https://github.com/aau-cns/autonomy_engine
[MavROS (CNS Version)]: https://github.com/aau-cns/mavros
[PX4Bridge (CNS Version)]: https://github.com/aau-cns/PX4-Autopilot
[academic paper]: https://ieeexplore.ieee.org/document/9849131