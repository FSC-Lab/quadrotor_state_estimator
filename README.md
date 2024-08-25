# state_estimator_switcher

[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://GitHub.com/Naereen/StrapDown.js/graphs/commit-activity)

### A simple ROS Node that acts as an intermediate switcher between motion capture state estimation and gps state estimation from mavros.

- Used for switching between indoor and outdoor testing for a [GUI software](https://github.com/FSC-Lab/ground_station_gui)

## Authors

- Tommy Zhang
- HS Helson Go
- Dr. Longhao Qian

## How to use

Subscribes to:

- /mavros/local_position/odom
- /mocap/UAV0

Publishes to:

- /mavros/local_position/odom/UAV0
