from depthai_sdk import OakCamera

with OakCamera() as oak:
    color = oak.camera('color')
