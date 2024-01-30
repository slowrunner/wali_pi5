# RGB Tiny YOLO v4 with Spatial Data (with and without visual display)

CODE: https://raw.githubusercontent.com/luxonis/depthai-python/main/examples/SpatialDetection/spatial_tiny_yolo.py  
BLOB: https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob  
REF: https://docs.luxonis.com/projects/api/en/latest/samples/SpatialDetection/spatial_tiny_yolo/  



* ------ Raspberry Pi5 Pi OS Bookworm installation -----


1) ./setup_USB_rules.sh
```
#/bin/bash

echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

echo -e "RULES LOADED - NOW DISCONNECT AND RECONNECT THE USB CABLE TO THE OAK-D-LITE"

```

2) Setup Models  
```
mkdir /home/pi/wali_pi5/oak_models
pushd /home/pi/wali_pi5/oak_models
wget https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob
popd
```
3) Setup Python3 venv  
```
cd /home/pi/wali_pi5/systests/Oak-D-Lite
mkdir Oak-D-Lite
cd Oak-D-Lite
python3 -m venv --system-site-packages vdepthai 
```

4) Install Dependencies
```
cd /home/pi/wali_pi5/systests/Oak-D-Lite/vdepthai/
git clone https://github.com/luxonis/depthai.git
source bin/activate
pip3 install opencv-python opencv-contrib-python python3-matplotlib
cd depthai
python3 install_requirements.py 
```

5) TRY IT  (via TigerVNC remote desktop)
```
cd /home/pi/wali_pi5/systests/Oak-D-Light/vdepthai
source bin/activate
cd depthai
(vdepthai) pi@WaLiPi5:~/wali_pi5/systests/Oak-D-Lite/vdepthai/depthai $ python3 depthai_demo.py
```
6) Install spatial_tiny_yolov4
```
cd /home/pi/wali_pi5/systests/Oak-D-Light/vdepthai/
mkdir spatial_tiny_yolo
cd spatial_tiny_yolo
wget https://raw.githubusercontent.com/luxonis/depthai-python/main/examples/SpatialDetection/spatial_tiny_yolo.py
cp spatial_tiny_yolo.py orig_spatial_tiny_yolo.py
```
- Modify orig_spatial_tiny_yolo.py for /home/pi/wali_pi5/oak_models mblobPath
```
# nnBlobPath = str((Path(__file__).parent / Path('../models/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob')).resolve().absolute())
nnBlobPath = "/home/pi/wali_pi5/oak_models/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob"
```
7) Tryit via TigerVNC Remote
```
python3 orig_spatial_tiny_yolo.py
or
./spatial_tiny_yolo.py -d
```

8) Try console only version:
```
./console_spatial_tiny_yolo.py
or 
./spatial_tiny_yolo.py   (without the -d)
```

* ----- DOCKER INSTALLATION ----
(Separate Docker Container from ROS 2 Humble Desktop "Plus" r2hdp tagged Docker image)





# ANALYSIS (Raspberry Pi 3B+ Legacy Pi OS 2021-12):

 - Base: Ave load 0.02 (0.2 w/VNC desktop) 41degC 5.8 Watts 0.49A at 11.8V

 - Returning rgb preview annotated with object boundary and spacial x,y,z data, and full depthmap display  
   13-19 FPS Ave RPi3B+ load 4.0 (peak 5.5) Temp: 65degC (no throttling, soft temp limit 0x8 flag set)  
   (displayed on remote desktop via VNC)  
   12.4 Watts (1.2A at 10.4V)  
   ~ 6.6 Watts for Oak-D-Lite and display load  
   ~ 4.0 ave load - 100% of four cores at maximum processor frequency  
   ~ 18degC processor temperature load (unaspirated heatsink)  

```

== Modified with image and depth display option

./spatial_tiny_yolo.py [-d] [--display] [-h] [--help]
optional arguments:
  -h, --help     show this help message and exit
  -d, --display  display annotated rgb images and depth map

== Modified for results only, no image display

./console_spatial_tiny_yolo.py  

```

Spatial Tiny-yolo (with optional image, objects, and depth display)  
  Performs inference on RGB camera and retrieves spatial location coordinates: x (h), y (v), z (range) relative to device  
  Using  tiny-yolo-v4 network on rgb camerafor object recognition and stereo depth network on left+right greyscale cameras 

```
# Tiny yolo v3/4 label texts n=80 classes
labelMap = [  
    "person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
    "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
    "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant",
    "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
    "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
    "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
    "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
    "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
    "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
    "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
    "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
    "teddy bear",     "hair drier", "toothbrush"
]

```

# ANALYSIS (Raspberry Pi 5B Bookworm Pi OS 2024-01-18)   

 - Base: Ave load 0.00 (0.05 w/TigerVNC desktop, Docker ROS2 Wali ) 57 degC 3.8 Watts 0.734A at 5.1V 6.9Gi avail/7.9Gi Total  

 - console_spatial_tiny_yolo.py  

   - Console output only of processed frame rate, object x,y,z (depth) and object classification (label)
   - 27-28 FPS  
   - uptime 1m load 0.22 max 5m load 0.08  6.8Gi avail  59 degC  freq 1.5  
   - 8W 1.57A at 5.09v  
   - Create3 battery:  (13.5W)  
      voltage: 14.5v    
      temperature: 38.4 C  
      current: -0.93A  
      percentage: 0.54  


- spatial_tiny_yolo.py -d  
   - RGB preview annotated with object boundary, spatial x,y, z (depth) data, with full depthmap display  
     (displayed on remote desktop via TigerVNC)  
  - 14 FPS
  - uptime 1m load max 3.09 6.5Gi avail 69C 2.4GHz  
  - Create3 battery:  (16.4W)  
     voltage: 14.38
     temperature: 38.25
     current: -1.141
     percentage: 0.49


  
