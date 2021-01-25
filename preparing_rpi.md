## Preparing the Raspberry Pi
The goal of these instructions is to configure the RPi in a way that is a similar as possible to the standard configuration of the RPi as provided by SDU Robotics. The setup instructions are intended to configure the RPi in a headless state and have been tested from a pc running Ubuntu 18.04.

#### Write Raspberry Pi OS image to SD-card
This is easiest done using RPi Imager. To install RPi Imager run:
```
snap install rpi-imager
```
Run RPi Imager:
```
rpi-imager
```
Select **Raspberry Pi OS Lite (32-BIT)**, and the intended SD-card and click write.

Once the image has been written, the SD-card must be mounted on the pc for the remaining configuration. If this is not done automatically then pull out the SD-card and insert it again. The SD-card should be mounted at `/media/"$USER"/`.

#### Enabling SSH
SSH must be enabled to allow for connecting to the RPi using SSH. This is done by placing a file named `SSH` in the `/boot/` directory.
```
touch /media/"$USER"/boot/SSH
```

#### Enabling PWM
To use the PWM chip on the RPi it must first be enabled. This will be done automatically by the installation script, but should you wish to manually enable the PWM chip it can be done by adding the line `dtoverlay=pwm-2chan` to `/boot/config.txt`.

#### Setting the hostname
The default hostname of the pi is `raspberrypi`. To align this with the existing Frobit naming convention, the hostname should be changed to `frobit00`. This is done by exchanging `raspberrypi` for `frobit00` in `/etc/hostname` and `/etc/hosts`.
```
DIR=/media/"$USER"/rootfs/etc
sed 's/raspberrypi/frobit00/g' $DIR/hostname | sudo tee $DIR/hostname
sed 's/raspberrypi/frobit00/g' $DIR/hosts | sudo tee $DIR/hosts
```
Note: The default username on the RPi is **pi**. We will not change this.

#### Providing internet connection for the RPi
The RPi needs an internet connection to download dependencies during the setup. This can either be done by [sharing your computers internet connection through ethernet](https://www.crookm.com/journal/2018/sharing-wifi-connection-over-ethernet/), or by [configuring the RPi to connect to WiFi](https://www.raspberrypi.org/documentation/configuration/wireless/headless.md).

#### Connecting to the RPi
Install the SD-card in the RPi and power it on. After a while the RPi is ready to connect with SSH.
```
ssh pi@frobit00.local
```
The password is **raspberry** (we will change this in a moment).

#### Changing Password on the RPi
Once connected to user **pi** on the RPi through SSH, you can change its password to **pi**.
```
sudo passwd pi
```
