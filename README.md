# Frobit-Pixhawk controller
This project contains code to for the [Frobit mobile robot](http://frobomind.org/web/doku.php) developed by SDU Robotics. The code enables the robot to be controlled by a Pixhawk Cube flight controller running the PX4 flight stack.

The targeted version of the Frobit is controlled by a Raspberry Pi 3B with a custom hat that communicates with the Frobomind controller.

The sections below describe how to set up the hardware and software.

## Hardware configuration
Coming soon...

## Preparing the Raspberry Pi
See [instructions](https://github.com/sunesj/frobit-pixhawk/blob/master/preparing_rpi.md) for setting up the RPi with a clean installation of Raspberry Pi OS (Raspbian Buster).

## Installing and Running the Frobit-Pixhawk Controller
Once the RPi is prepared, and you are connected to it through SSH, the Frobit-Pixhawk code and its dependencies must be installed and configured. Convenience scripts are provided for this ([setup.sh](https://github.com/sunesj/frobit-pixhawk/blob/master/scripts/setup.sh) and [finish_setup.sh](https://github.com/sunesj/frobit-pixhawk/blob/master/scripts/finish_setup.sh)). To begin the installation simply download and run [setup.sh](https://github.com/sunesj/frobit-pixhawk/blob/master/scripts/setup.sh).
```
curl -fsSL https://raw.githubusercontent.com/sunesj/frobit-pixhawk/master/scripts/setup.sh | bash
```
Once the script is done it will prompt you to reboot and run [finish_setup.sh](https://github.com/sunesj/frobit-pixhawk/blob/master/scripts/finish_setup.sh). Once the RPi is rebooted, run the script to finish the installation.
```
bash "$HOME"/Frobit-Pixhawk/scripts/finish_setup.sh
```

## Running the Frobit-Pixhawk Controller
The Frobit-Pixhawk Controller is implemented in a series of docker containers. To start the controller run `docker-compose up` from the `Frobit-Pixhawk` directory.
```
cd "$HOME"/Frobit-Pixhawk
docker-compose up
```
This will automatically download and build the necessary Docker images and start the Docker containers. The containers are configured to automatically start after a reboot.
