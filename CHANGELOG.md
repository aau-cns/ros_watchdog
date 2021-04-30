# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [UNRELEASED]
**BRANCH**: `feat/watchdog_splitting`
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

[Unreleased]: https://gitlab.aau.at/aau-cns/ros_pkgs/aaucns_ros_watchdog/-/compare/develop...main
[Original]: https://gitlab.aau.at/aau-cns/ros_pkgs/aaucns_ros_watchdog/-/tree/master
