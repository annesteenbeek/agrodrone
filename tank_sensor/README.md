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
The pins used for bitbanging are the same as on the older odroid for normal spi.
```
sclk -> 23
miso -> 21
mosi -> 19
Chip select -> 24
```

