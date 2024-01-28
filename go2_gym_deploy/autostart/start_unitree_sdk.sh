#!/bin/bash
sudo docker stop foxy_controller || true
sudo docker rm foxy_controller || true
sudo kill $(ps aux |grep lcm_position_go2 | awk '{print $2}')
cd ~/go2_gym/go2_gym_deploy/build/
yes "" | sudo ./lcm_position_go2 eth0 &