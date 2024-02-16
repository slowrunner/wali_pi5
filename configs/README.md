# Raspberry Pi5 USB0 "GADGET MODE' SETUP

REF: https://www.hardill.me.uk/wordpress/2023/12/23/pi5-usb-c-gadget/   

1) Ensure WaliP5 is using 5GHz WiFi  (reserved 5GHz IP on router)  
2) Setup USB0 network on WaliPi5  
    - /boot/firmware/config.txt - add to end of file  
```
# enable USB0 networking
dtoverlay=dwc2
```
    - /boot/firmware/cmdline.txt  - add to end of line  
```
 modules-load=dwc2
```
    - /etc/modules  - add to end of file  
```
libcomposite
```
    - create /etc/network/interfaces.d/usb0  REF: https://forums.raspberrypi.com/viewtopic.php?t=357938  
```
allow hotplug usb0
auto usb0
interface usb0 inet static
    address 192.168.186.3/24
```
    - Update bootloader:  
```
 $ sudo rpi-update
```

    - sudo cp configs/usr_local_sbin.usb-gadget.sh /usr/local/sbin/usb-gadget.sh  
      (note I commented out bridge-slave-usb1)  

    - sudo cp configs/lib_systemd_system.usbgadget.service /lib/systemd/system/usbgadget.service  

    - source configs/add_usb0_br0_bridge.sh  

    - sudo apt install dnsmasq  

    - sudo cp configs/etc_dnsmasq_d.br0 /etc/dnsmasq.d/br0 

    - reboot  

    - ifconfig to see if it worked:
```
br0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        inet 192.168.186.3  netmask 255.255.255.0  broadcast 192.168.186.255

docker0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        inet xxx.xx.x.xx  netmask 255.255.0.0  broadcast xxx

usb0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether xx:xx:xx  txqueuelen 1000  (Ethernet)

wlan0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet xx.0.0.xxx  netmask 255.255.255.0  broadcast xx.0.0.xxx
```
