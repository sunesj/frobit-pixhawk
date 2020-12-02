#!/bin/bash

frobit_dir=$HOME/Frobit-Pixhawk

echo "Configuring docker and docker-compose"
sudo systemctl start docker
sudo apt install -y docker-compose
sudo groupadd docker
sudo usermod -aG docker "$USER"

echo "Removing dependencies that are no longer needed"
sudo apt autoremove -y

echo ""
echo "Setup done..."
echo ""
echo "Start Frobit by running:"
echo "  cd $frobit_dir"
echo "  docker-compose up"
echo ""

# Load docker group
newgrp docker
