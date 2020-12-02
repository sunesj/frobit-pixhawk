#!/bin/bash

if ! grep -q "dtoverlay=pwm-2chan" /boot/config.txt; then
  echo "Enabling PWM chip"
  echo "dtoverlay=pwm-2chan" | sudo tee -a /boot/config.txt >/dev/null
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
temp_file=$frobit_dir/tmp_crontab_file
crontab -l >"$temp_file"
echo "@reboot $frobit_dir/scripts/signal_alive.sh" >>"$temp_file"
crontab "$temp_file"
sudo crontab -l >"$temp_file"
echo "@reboot /usr/bin/python3 $frobit_dir/scripts/check_status.py" >>"$temp_file"
sudo crontab "$temp_file"
rm "$temp_file"

echo ""
echo "Please reboot and then run:"
echo "  source $frobit_dir/scripts/finish_setup.sh"
echo ""
