# WEEDINATOR
Code for microprocessor modules controlling the WEEDINATOR agricultural robot.

The main processor is a 3 core TC275 running at 200 MHz whose main function is to control the various motors with step pulses and direction HIGH / LOW. On the I2C bus it is the MASTER.

The second processor is a MEGA 2560 which is used mainly for good compatibility with existing arduino modules and code.

The third processor is a NANO which just about manages to recieve data from an Adafruit FONA module. The data comprises sets of coordinates which are used to create navigational headings for the machine. Currently, this data has to travel over to the TC275, then to the MEGA 2560 for creating a heading in degrees which then goes back to the TC275 for comparison with the bearing, also in degrees. One is subtracted from the other to make the machine's steering change automatically.

It would be really great to get rid of the NANO!
