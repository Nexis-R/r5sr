#!/bin/sh

SCRIPT_DIR=$(dirname "$0")

# udev rulesのインストール
sudo cp "$SCRIPT_DIR/99-cameras.rules" /etc/udev/rules.d/99-cameras.rules
sudo cp "$SCRIPT_DIR/99-dynamixel-device.rules" /etc/udev/rules.d/99-dynamixel-device.rules
sudo cp "$SCRIPT_DIR/99-rplidars1.rules" /etc/udev/rules.d/99-rplidars1.rules
sudo cp "$SCRIPT_DIR/99-rt-9axisimu.rules" /etc/udev/rules.d/99-rt-9axisimu.rules
