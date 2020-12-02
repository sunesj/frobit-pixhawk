#!/bin/bash

echo "Updating package list"
sudo apt update

echo "Updating packages"
sudo apt upgrade -y

echo "Installing dependencies"
sudo apt install -y git docker docker-compose python3-pip
pip3 install pyserial

echo "Granting sudo privileges to Docker"
sudo groupadd docker
sudo usermod -aG docker "$USER"
newgrp docker

echo "Cloning code from GitHub"
frobit_dir=$HOME/Frobit-Pixhawk
git clone https://github.com/sunesj/frobit-pixhawk.git "$frobit_dir"

echo "Configuring crontabs"
current_dir=$PWD
cd "$frobit_dir" || (echo "Unable to enter directory $frobit_dir. Aborting setup..." && exit)
temp_file=tmp_crontab_file
cmd_signal_alive=$frobit_dir/scripts/signal_alive.sh
cmd_check_status="/usr/bin/python3 $frobit_dir/scripts/check_status.py"
crontab -l > $temp_file
echo "@reboot $cmd_signal_alive" >> $temp_file
crontab $temp_file
# shellcheck disable=SC2024
sudo crontab -l > $temp_file
echo "@reboot @cmd_check_status" >> $temp_file
sudo crontab $temp_file
rm $temp_file
# shellcheck disable=SC2164
cd "$current_dir"

# Starts script to avoid a reboot
echo "Starting status scripts"
$cmd_signal_alive &
$cmd_check_status &

echo ""
echo "Setup done..."
echo ""
echo "Start Frobit by running these commands:"
echo ""
echo "  cd $frobit_dir"
echo "  docker-compose up"


