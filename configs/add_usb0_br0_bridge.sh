#!/bin/bash


sudo nmcli con add type bridge ifname br0
sudo nmcli con add type bridge-slave ifname usb0 master br0
sudo nmcli connection modify bridge-br0 ipv4.method manual ipv4.addresses 192.168.186.3/24
