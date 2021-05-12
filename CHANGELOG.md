# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [UNRELEASED]
**BRANCH**: `test/autonomy_2105XX`
### Fixed
##### ros_watchdog
###### DriversObserver
- Script callback would yield intro wrong return value.
###### NodesObserver
- System call takes a long time to execute, use `rosnode` module instead.

**BRANCH**: `feat/actions`
### Added
##### watchdog_bringup
- Scripts to restart realsense HUB (WIP).

##### watchdog_msgs
- Added `Action` and `ActionStamped` message definition

### Changed
- Updated `.gitignore` to include PyCharm, ROS, Python ignores (see https://www.toptal.com/developers/gitignore)
##### ros_watchdog
###### Observer
- Updated `Node(s)Observer` to inherit from `Observer(s)` super class

### Deprecated
- `SensorObserver.py` was replaced by `DriversObserver.py`. Will be removed in future commit.

**[FEATURE 1]**: `feat/watchdog_splitting` (merged to `develop`)
### Added
- Added `watchdog_bringup` (metapackage for launching)

##### aaucns_ros_watchdog
- Added compile dependencies to other packages of this repository

##### ros_watchdog
###### General
- Made `ros_watchdog` a ROS python package
###### Watchdog Node
- Added Watchdog Node `nodes/WatchdogNode.py`
- Added topic `/watchdog/log` for delta status changes in assets
- Added heartbeat message to contain global (worst) status
- Added start service under `/watchdog/service/start`
###### Watchdog
- Added Watchdog class `src/watchdog/Watchdog.py`
###### Observer
- Added `DriverObserver` which observes drivers

### Changed
##### ros_watchdog
- Made `ros_watchdog` a ROS python package
- Moved Observer files into observer module in `src/observer`
- Moved configuration `.ini` files to `watchdog_bringup/config`
- Updated Watchdog Node with new message definitions for autonomy

##### watchdog_msgs
- Renamed `autonomy_msgs` to `watchdog_msgs`
- Made `watchdog_msgs` a separate ROS package

### Deprecated
- Launchfiles in `ros_watchdog`
- Config in `ros_watchdog`

### Documentation

### Fixed

### Removed
- Old definition of services in `ros_watchdog`

###

## [Original]

Original Version by @rojung.

[FEATURE 1]: https://gitlab.aau.at/aau-cns/ros_pkgs/aaucns_ros_watchdog/-/tree/eef00de426041d7740b41923108a834ad830ad55
[Unreleased]: https://gitlab.aau.at/aau-cns/ros_pkgs/aaucns_ros_watchdog/-/compare/develop...main
[Original]: https://gitlab.aau.at/aau-cns/ros_pkgs/aaucns_ros_watchdog/-/tree/master
