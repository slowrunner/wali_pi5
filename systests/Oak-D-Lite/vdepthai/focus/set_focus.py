#!/usr/bin/env python3


# REF:  https://discuss.luxonis.com/d/3278-lens-position-conversion-to-cm/4

import argparse
from depthai_sdk.managers import EncodingManager, PipelineManager
from depthai_sdk import Previews
import depthai as dai


parser = argparse.ArgumentParser()
parser.add_argument("-af", "--af_range", nargs=2, type=int,
    help="set auto focus range in cm (min distance, max distance)", metavar=("cm_min", "cm_max"))
args = parser.parse_args()

pm = PipelineManager()
pipeline = pm.pipeline
cam_rgb = pm.createColorCam()

if args.af_range:
    xin_ctrl = pipeline.create(dai.node.XLinkIn)
    xin_ctrl.setStreamName("control")
    xin_ctrl.out.link(cam_rgb.inputControl)


def set_focus_range():
    """Convert closest cm values to lens position values and set auto focus range."""
    cm_lenspos_dict = {
        6: 250,
        8: 220,
        10: 190,
        12: 170,
        14: 160,
        16: 150,
        20: 140,
        25: 135,
        30: 130,
        40: 125,
        60: 120
    }

    closest_cm_min = min(cm_lenspos_dict.keys(), key=lambda k: abs(k - args.af_range[0]))
    closest_cm_max = min(cm_lenspos_dict.keys(), key=lambda k: abs(k - args.af_range[1]))

    lenspos_min = cm_lenspos_dict[closest_cm_max]  # minimum lens position = 0 (infinity)
    lenspos_max = cm_lenspos_dict[closest_cm_min]  # maximum lens position = 255 (macro)

    af_ctrl = dai.CameraControl().setAutoFocusLensRange(lenspos_min, lenspos_max)
    q_ctrl.send(af_ctrl)


with dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH) as device:

    if args.af_range:
        q_ctrl = device.getInputQueue(name="control")
        set_focus_range()

