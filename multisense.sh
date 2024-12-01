#!/bin/bash
sudo systemctl stop NetworkManager
source install/setup.bash
sudo ifconfig enx207bd2e29411 10.66.171.20
sudo ip link set down enx207bd2e29411;
sudo ip link set enx207bd2e29411 mtu 1500;
sudo ip link set up enx207bd2e29411;
sudo systemctl start NetworkManager
