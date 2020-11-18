#!/bin/python

import commands
import time

base_path = '/sys/class'
pwm_period = 50000
sleep_period = 0.5


def command(c):
    status, output = commands.getstatusoutput(c)
    if status:
        print(c + ' failed with ' + str(output))
    return output


def setup(cmd, path):
    c = 'echo ' + str(cmd) + ' > ' + str(base_path) + '/' + str(path)
    command(c)


def get(path):
    c = 'cat ' + str(base_path) + '/' + str(path)
    return command(c)


def exportGPIO(gpio):
    setup(gpio, 'gpio/export')
    time.sleep(sleep_period)


def unexportGPIO(gpio):
    setup(gpio, 'gpio/unexport')
    time.sleep(sleep_period)


def setDirection(gpio, direction):
    setup(str(direction), 'gpio/gpio' + str(gpio) + '/direction')
    time.sleep(sleep_period)


def getDirection(gpio):
    direction = get('gpio/gpio' + str(gpio) + '/direction')
    time.sleep(sleep_period)
    return direction


def setValue(gpio, value):
    setup(str(value), 'gpio/gpio' + str(gpio) + '/value')


def getValue(gpio):
    return get('gpio/gpio' + str(gpio) + '/value')


def setPullUp(gpio):
    c = 'raspi-gpio set ' + str(gpio) + ' pu'
    command(c)


def setPullDown(gpio):
    c = 'raspi-gpio set ' + str(gpio) + ' pd'
    command(c)


def setPullNone(gpio):
    c = 'raspi-gpio set ' + str(gpio) + ' pn'
    command(c)


def exportPWM(channel):
    setup(channel, 'pwm/pwmchip0/export')
    time.sleep(sleep_period)


def unexportPWM(channel):
    setup(channel, 'pwm/pwmchip0/unexport')
    time.sleep(sleep_period)


def setPWMDuty(channel, duty_cycle):
    duty = pwm_period * duty_cycle / 100.0
    setup(int(duty), 'pwm/pwmchip0/pwm' + str(channel) + '/duty_cycle')


def enablePWM(channel):
    setup('1', 'pwm/pwmchip0/pwm' + str(channel) + '/enable')


def disablePWM(channel):
    setup('0', 'pwm/pwmchip0/pwm' + str(channel) + '/enable')


def setPWMperiod(channel):
    setup(int(pwm_period), 'pwm/pwmchip0/pwm' + str(channel) + '/period')
