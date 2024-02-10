from depthai_sdk import OakCamera

print(dir(OakCamera))

with OakCamera() as oak:
    color = oak.config_camera('color')
    stereo = oak.create_stereo()
    stereo.config_stereo(align=color)
    pcl = oak.create_pointcloud(stereo=stereo, colorize=color)
    oak.visualize(pcl, visualizer='depthai-viewer')
    oak.start(blocking=True)
