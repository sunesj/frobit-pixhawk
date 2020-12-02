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
src_dir=$HOME/Frobit-Pixhawk
git clone https://github.com/sunesj/frobit-pixhawk.git "$src_dir"
cd "$src_dir" || (echo "Unable to enter directory $src_dir. Aborting setup..." && exit)

echo "Configuring crontabs"
temp_file=tmp_crontab_file
crontab -l > $temp_file
echo "@reboot $src_dir/scripts/signal_alive.sh" >> $temp_file
crontab $temp_file
# shellcheck disable=SC2024
sudo crontab -l > $temp_file
echo "@reboot /usr/bin/python3 $src_dir/scripts/check_status.py" >> $temp_file
sudo crontab $temp_file
rm $temp_file

echo "Done with setup. Please reboot..."
