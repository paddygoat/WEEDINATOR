<img src="https://github.com/paddygoat/WEEDINATOR/blob/master/images/WEEDINATOR%202018.png" />

# WEEDINATOR
Code for microprocessor modules controlling the WEEDINATOR agricultural robot and manufacturing files.  All code compiles using the Arduino IDE and is in .ino format.

## Processors

#### Primary

A 3 core TC275 running at 200 MHz controls the various motors with step pulses and direction HIGH / LOW. On the I2C bus it is the MASTER.

#### Secondary

A MEGA 2560 is used mainly for good compatibility with existing arduino modules and code. Its function is to receive data from 

* GNSS module ([Ublox C94 NEO-M8M](https://www.u-blox.com/en/product/neo-m8-series) Base & Rover)
* GSM module ([Adafruit FONA 800H GPRS](https://www.adafruit.com/product/1946))
* Object recognition camera ([PIXY](http://cmucam.org/projects/cmucam5/wiki) )
* Digital compass ([Sparkfun 9DOF Razor IMU MO](https://www.sparkfun.com/products/14001), see [Issue #10](https://github.com/paddygoat/WEEDINATOR/issues/10))
  
## Steering

* `Heading` is the direction that the machine is actually travelling in
* `Bearing` is the direction that it needs to go in.

Subtracting one from the other gives the direction and amount that the steering needs to turn.
