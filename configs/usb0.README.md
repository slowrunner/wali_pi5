# Set up usb0 networking on Raspberry Pi 5 USB-C connector to connect with Create3

REF: https://forums.raspberrypi.com/viewtopic.php?p=2195159#p2195179  
(For a never booted system REF: https://forums.raspberrypi.com/viewtopic.php?t=364247 )  

1) sudo nano /etc/network/interfaces.d/g_ether  

```
auto usb0 
allow-hotplug usb0 
iface usb0 inet static
        address 192.168.186.3
        netmask 255.255.0.0

auto usb0.1
allow-hotplug usb0.1
iface usb0.1 inet dhcp

EOF
```

2) sudo apt update && sudo apt upgrade -y

3) add to /boot/firmware/config.txt  
```
dtoverlay2,dr_mode=peripheral
```

4) add to end of /boot/firmware/cmdline.tx  
```
modules-load=dwc2,g_ether
```

5) reboot  

6) ifconfig  
```
docker0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        inet 172.17.0.1  netmask 255.255.0.0  broadcast 172.17.255.255
        ether 02:42:33:ed:b6:4a  txqueuelen 0  (Ethernet)

eth0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether d8:3a:dd:b6:36:a1  txqueuelen 1000  (Ethernet)

lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        loop  txqueuelen 1000  (Local Loopback)

usb0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.186.3  netmask 255.255.0.0  broadcast 192.168.255.255
        ether ea:78:3a:fa:b6:c5  txqueuelen 1000  (Ethernet)

usb0.1: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        ether ea:78:3a:fa:b6:c5  txqueuelen 1000  (Ethernet)

wlan0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet x.x.x.wifi  netmask 255.255.255.0  broadcast x.x.x.255
        ether d8:3a:dd:b6:36:a2  txqueuelen 1000  (Ethernet)s 0
```
