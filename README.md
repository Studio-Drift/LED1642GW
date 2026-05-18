# LED1642GW
An Arduino compatible LED1642GW LED driver library for ESP32 that uses direct register manipulation of the output registers to generate the required Clk, Data and Latch signals.

The library uses the LCD DMA interface to feed the output to the drivers with a 20MHz clock speed.

It can currently update 2000 16 bit values in under 2ms. This update takes approx 700us for the CPU.

The only drawback of using the LCD DMA is that one GPIO needs to be sacrificed to route all the unused signals to. By default pin 1 is used, but this can be any unused gpio that is floating.

The driver requires a config register and an output enable register to be set correctly in order for the LEDs to be powered. To allow for asynchronous power-up, the controller periodically sends these settings along with the LED data.

If the Drivers do not have a hardware PWM Clock connected, the Library can provide a 10MHz PWM reference clock using the built-in I2S hardware. This does however limit the PWM frequency to just 150Hz in 16-bit mode. It is advised to use a higher frequency hardware solution.

Originally created and maintained by Pim Swinkels.
Please retain attribution to the original author in forks and redistributions.
