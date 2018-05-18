# WEEDINATOR
Code for microprocessor modules controlling the WEEDINATOR agricultural robot and manufacturing files.  All code compiles using the Arduino IDE and is in .ino format.

## Processors

The main processor is a 3 core TC275 running at 200 MHz whose main function is to control the various motors with step pulses and direction HIGH / LOW. On the I2C bus it is the MASTER.

The second processor is a MEGA 2560 which is used mainly for good compatibility with existing arduino modules and code. Its function is to recieve data from 

* GNSS module (Ublox C94 M8M)
* FONA GPRS module
* [PIXY](http://cmucam.org/projects/cmucam5/wiki) object recognition camera
* Digital compass (9DOF Razor IMU MO)<a href="#1"><sup>1</sup></a>
  
## Steering

* `Heading` is the direction that the machine is actually travelling in
* `Bearing` is the direction that it needs to go in.

Subtracting one from the other gives the direction and amount that the steering needs to turn.



<hr>
<a name="1"><sup>1</sup></a> The compass has issues with revolving magnets and large pieces of moving steel so may be of limited use. Currently the default is that the machine drives directly forwards which is overidden when it recieves data through fix.heading() which itself is overridden if the camera recognises a pre-programmed object.
