# WEEDINATOR
Code for microprocessor modules controlling the WEEDINATOR agricultural robot.

The main processor is a 3 core TC275 running at 200 MHz whose main function is to control the various motors with step pulses and direction HIGH / LOW. On the I2C bus it is the MASTER.

The second processor is a MEGA 2560 which is used mainly for good compatibility with existing arduino modules and code. It's function is to recieve data from the GNSS module, FONA GPRS module, a PIXIE object recognition camera and a digital compass. The compass has issues with revolving magnets and large pieces of moving steel so may be of limited use. Currently the default is that the machine drives directly forwards which is overidden when it recieves data through fix.heading() which itself is overridden if the camera recognises a pre-programmed object.

'Heading' is the direction that the machine is actually travelling in and 'Bearing' is the direction that it needs to go in. Subtracting one from the other gives the direction and amount that the steering needs to turn.

All code compiles using the Arduino IDE and is in .ino format.

Work to do: 
* TC275 needs to display waypoint number and FONA GNSS coordinates.
* Mega needs to download the next waypoint in advance so that the machine does not stop and think at each waypoint.
* Torque / current balancing needs to be over ridden when going downhill.
