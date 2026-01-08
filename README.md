# I2C Hacks

A repo for storing sample programs for testing and experimenting with using I2C sensors over VGA and HDMI ports of laptops.

The first one is a sample program a bno055 9-axis IMU sensor over I2C used to test the validity of using VGA and HDMI ports as GPIO. (Can be adapted to use for other sensors as well.)



On Linux, you can use `i2cdetect -l` to look for I2C buses that Linux has registered on your machine (like one for your HDMI port and VGA port).

You can then use `i2cdetect -r -y <BUS_NUMBER>` to get an output of what addresses are currently in-use which is the quick check to see if the wiring of your sensor is done correctly.


After that, you can run this program with `sudo python3 main.py` (after installing the required python libraries) to get a visualization of the bno055 sensor as it moves.