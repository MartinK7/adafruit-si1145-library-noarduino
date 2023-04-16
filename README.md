# Adafruit SI1145 Library [![Build Status](https://github.com/adafruit/Adafruit_SI1145_Library/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_SI1145_Library/actions)

This is the Adafruit Arduino library for the  Si1145 UV/IR/Visible Light Sensor

Tested and works great with the Adafruit SI1145 Breakout Board
[<img src="assets/board.jpg?raw=true" width="500px">](https://www.adafruit.com/products/1777)

These sensors use I2C to communicate, 2 pins are required to interface

Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!

# Contributing

Contributions are welcome!

# Original source code

Written by Limor Fried for Adafruit Industries.
BSD license, check license.txt for more information
All text above must be included in any redistribution

# Noarduino
This forked project does not rely on arduino libraries. Just implement these functions, example:

```C
void si1145_delay(uint32_t delay_ms)
{
	vTaskDelay(delay_ms / portTICK_PERIOD_MS);
}

int8_t si1145_i2c_init(si1145_t *si1145)
{
	return 0;
}

void si1145_i2c_write_then_read(si1145_t *si1145, const uint8_t *buffer_tx, int buffer_tx_count, uint8_t *buffer_rx, int buffer_rx_count)
{
	uint8_t buffer[8];
	if(buffer_tx_count > sizeof(buffer)) {
		return;// -1;
	}
	memcpy(&buffer[0], buffer_tx, buffer_tx_count);

	if(i2c_master_transaction(si1145->dev_i2c, si1145->addr, I2C_DIRECTION_WRITE, buffer, buffer_tx_count)) {
		return;// -1;
	}

	if(i2c_master_transaction(si1145->dev_i2c, si1145->addr, I2C_DIRECTION_READ, buffer_rx, buffer_rx_count)) {
		return;// -1;
	}

	return;// 0;
}

void si1145_i2c_write(si1145_t *si1145, const uint8_t *buffer_tx, int buffer_tx_count)
{
	uint8_t buffer[8];
	if(buffer_tx_count > sizeof(buffer)) {
		return;// -1;
	}
	memcpy(&buffer[0], buffer_tx, buffer_tx_count);

	if(i2c_master_transaction(si1145->dev_i2c, si1145->addr, I2C_DIRECTION_WRITE, buffer, buffer_tx_count)) {
		return;// -1;
	}

	return;// 0;
}
```
