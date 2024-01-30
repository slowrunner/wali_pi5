#!/usr/bin/env python3

# import cv2
import depthai as dai
# import time
# import math

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
imu = pipeline.create(dai.node.IMU)
xlinkOut = pipeline.create(dai.node.XLinkOut)

xlinkOut.setStreamName("imu")


# Pipeline is defined, now we can connect to the device
try: 
    device = dai.Device(pipeline) 
except Exception as e:
    print(e)
    with dai.Device() as device:
        print("Device MxID: ", device.getMxId())

