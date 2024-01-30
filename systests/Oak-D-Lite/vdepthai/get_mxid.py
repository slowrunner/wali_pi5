#!/bin/env python3

# FILE:  get_mxid.py

# Will print the Oak device Manufacturer's Serial Number

# My Oak-D-Lite (From Kickstarter campaign) reports 184430101175A41200


import depthai as dai
with dai.Device() as device:
    print(device.getMxId())
