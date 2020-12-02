#!/bin/bash

if ! grep -q "dtoverlay=pwm-2chan" /boot/config.txt;
then
  echo "Enabling PWM chip"
  echo "dtoverlay=pwm-2chan" | sudo tee -a /boot/config.txt > /dev/null
  #sudo dtoverlay pwm-2chan # Enable overlay manually to avoid reboot
fi

echo "Updating packages"
sudo apt update && sudo apt upgrade -y

echo "Installing dependencies"
sudo apt install -y git python3-pip docker
pip3 install pyserial

echo "Cloning code from GitHub"
frobit_dir=$HOME/Frobit-Pixhawk
git clone https://github.com/sunesj/frobit-pixhawk.git "$frobit_dir"

echo "Configuring crontabs"
current_dir=$PWD
cd "$frobit_dir" || (echo "Unable to enter directory $frobit_dir. Aborting setup..." && exit)
temp_file=tmp_crontab_file
cmd_signal_alive=$frobit_dir/scripts/signal_alive.sh
cmd_check_status="/usr/bin/python3 $frobit_dir/scripts/check_status.py"
crontab -l >$temp_file
echo "@reboot $cmd_signal_alive" >> $temp_file
crontab $temp_file
# shellcheck disable=SC2024
sudo crontab -l >$temp_file
echo "@reboot @cmd_check_status" >> $temp_file
sudo crontab $temp_file
rm $temp_file
# shellcheck disable=SC2164
cd "$current_dir"

# Starts script to avoid a reboot
#echo "Starting status scripts"
#$cmd_signal_alive &
#$cmd_check_status &

echo ""
echo "Please reboot and then run:"
echo "  source $frobit_dir/finish_setup.sh"
echo ""



