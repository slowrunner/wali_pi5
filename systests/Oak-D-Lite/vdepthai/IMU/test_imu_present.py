#!/usr/bin/env python3

import depthai as dai

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
imu = pipeline.create(dai.node.IMU)
xlinkOut = pipeline.create(dai.node.XLinkOut)

xlinkOut.setStreamName("imu")


# Pipeline is defined, now we can connect to the device
try: 
    device = dai.Device(pipeline) 
    print("IMU Detected")

except Exception as e:
    print(e)
    with dai.Device() as device:
        print("Device MxID: ", device.getMxId())

