#Tank sensor package

This package is used to read the pressure sensor from the Odroid c2 and broadcast the fluid level

The analog sensor is read using a mcp3008 chip connected by SPI

the odroid c2 board does not, by default, support spi, bitbanging is used as an alternative to privde spi capabilities

## Enable bitbanging on the Odroid-C2
In order to use spi some kernel modules need to be enabled,
```
spi-bitbang
spi-gpio
spidev
```

These are blacklisted by default, to enable them at boot remove (or uncomment them in) /etc/modprobe.d/blacklist-spi.conf

## Connecting

![missing screenshot](info/MCP3008.png?raw=true "Basic plot of test data")

The pins used for bitbanging are the same as on the older odroid for normal spi.

Pin | Odroid | chip
-----|-------|------
sclk |GPIO 23 | Pin 13
miso |GPIO 21 | Pin 12
mosi |GPIO 19 | Pin 11
Chip select |GPIO 24 | Pin 10


