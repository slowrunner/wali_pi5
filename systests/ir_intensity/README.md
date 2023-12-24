# Create3 IR Intensity To Distance Function  

The Create3 has seven IR "obstacle" sensors which return a non-linear IR intensity value.  
The returned intensity varies with the obstacle's surface reflectivity and incidence angle,  
as well as sensor to sensor differences.  

The Create3_ir_dist.dist_ir_reading(sensor,reading) function returns:  
- Estimated distance (for particular IR sensor)  
- within the limits of 15mm to 400mm  
- -99.999 outside of the distance limits

The estimates are based on:  
- Average of three IR intensity readings  
- For each of 7 IR intensity sensors:   
  - "side_left"   
  - "left"  
  - "front_left"  
  - "front_center_left"  
  - "front_center_right"   
  - "front_right"  
  - "right"  
- At 7 distances: 0.4m 0.3m 0.2m 0.1m 0.05m 0.025m 0.015m  
- Linear interpolation between the reference readings for an individual sensor
