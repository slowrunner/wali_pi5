# Piper-tts in Docker

This directory builds a test docker container with Ubuntu, the alsa utils aplay/arecord, and piper-tts  

The voice model is in /home/pi/wali_pi5/c3ws/models/piper-tts  


To build the test container:  
```
./build_audio_Docker.sh
```

To run the built container:  
```
./run_docker_audio.sh
```

To test aplay in the audio Docker container:  
```
./test_aplay.sh
```

To test piper-tts in the audio Docker container:  
```
./piper.sh "hello"
```

When done, list the Docker images:  
```
$ docker image list
REPOSITORY   TAG       IMAGE ID       CREATED          SIZE
audio        0.0.1     901e8c157048   28 minutes ago   470MB
<none>       <none>    f771fc0e9a3c   50 minutes ago   199MB
r2hdp        latest    8c2c8df3c265   7 days ago       3.44GB
```

Delete the Docker audio image(s) and any current "none" repo images:   
```
$ docker image rm 901e8c157048
Untagged: audio:0.0.1
Deleted: sha256:901e8c15704862b2439fc25e7c073de44442de107cd9a7dd711e282f122365f1

docker image rm f771fc0e9a3c
Deleted: sha256:f771fc0e9a3c383b13dc9b0bf882bbb3ca959da622d8c164ec367b16acd96d4e
```

Check that only r2hdp image is left:  
```
$ docker image list
REPOSITORY   TAG       IMAGE ID       CREATED      SIZE
r2hdp        latest    8c2c8df3c265   7 days ago   3.44GB
```

Prune the docker build cache:  
```
$ docker buildx prune
WARNING! This will remove all dangling build cache. Are you sure you want to continue? [y/N] y
ID						RECLAIMABLE	SIZE		LAST ACCESSED
a9pmvmit8bdhs0a2whjuewckz               	true 		223.9MB   	40 minutes ago
gmqe27wclw5v5l63jgrouivev*              	true 	301B      	40 minutes ago
cxd1dg9kspdk7fvvrlcolz4jc*              	true 	1.045kB   	7 days ago
yh6gvtak67mvfjwhnepqvofdx*              	true 	1.334kB   	7 days ago
zj0u49z0op8t08b7zzz3tnhfy               	true 	177MB     	42 minutes ago
jtnk9nnxn4cr7ni0fvao8lo0q*              	true 	0B        	7 days ago
if0f2o805jgf838nbhgxp2qyq               	true 	129.6MB   	About an hour ago
owpn0u3h67dx86c8tokmudsek*              	true 	0B        	40 minutes ago
044ztyjqujzqnhuryxmbkymbk               	true 	177MB     	40 minutes ago
Total:	707.4MB
```


Check how much space Docker is taking:  
```
sudo du -sh /var/lib/docker
```

Check how much free space is on the SDcard:  
```
df -h /
```


