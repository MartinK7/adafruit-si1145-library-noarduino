/***************************************************
  This is a library for the Si1145 UV/IR/Visible Light Sensor

  Designed specifically to work with the Si1145 sensor in the
  adafruit shop
  ----> https://www.adafruit.com/products/1777

  These sensors use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_SI1145.h"

static uint8_t si1145_writeParam(si1145_t *si1145, uint8_t p, uint8_t v);
static uint8_t si1145_read8(si1145_t *si1145, uint8_t reg);
static uint16_t si1145_read16(si1145_t *si1145, uint8_t a);
static void si1145_write8(si1145_t *si1145, uint8_t reg, uint8_t val);

/**
 * @brief Destructor
 *
 */
void si1145_deinit(si1145_t *si1145)
{
}


/**
 * @brief Initize the driver, supplying both a `dev_i2c` bus and I2C address
 *
 * @param addr The I2C address of the Sensor
 * @param pBus Pointer to the `dev_i2c` instance to use
 * @return bool true: success false: failure to initize the sensor
 */
bool si1145_init(si1145_t *si1145, uint8_t addr, void *dev_i2c)
{
	if(!si1145) {
		return false;
	}

	si1145->dev_i2c = dev_i2c;
	si1145->addr = addr;

	if(si1145_i2c_init(si1145)) {
		return false;
	}

	uint8_t id = si1145_read8(si1145, SI1145_REG_PARTID);
	if(id != 0x45) {
		return false; // look for SI1145
	}

	si1145_reset(si1145);

	/***********************************/
	// enable UVindex measurement coefficients!
	si1145_write8(si1145, SI1145_REG_UCOEFF0, 0x29);
	si1145_write8(si1145, SI1145_REG_UCOEFF1, 0x89);
	si1145_write8(si1145, SI1145_REG_UCOEFF2, 0x02);
	si1145_write8(si1145, SI1145_REG_UCOEFF3, 0x00);

	// enable UV sensor
	si1145_writeParam(si1145, SI1145_PARAM_CHLIST,
	SI1145_PARAM_CHLIST_ENUV | SI1145_PARAM_CHLIST_ENALSIR |
	SI1145_PARAM_CHLIST_ENALSVIS | SI1145_PARAM_CHLIST_ENPS1);
	// enable interrupt on every sample
	si1145_write8(si1145, SI1145_REG_INTCFG, SI1145_REG_INTCFG_INTOE);
	si1145_write8(si1145, SI1145_REG_IRQEN, SI1145_REG_IRQEN_ALSEVERYSAMPLE);

	/****************************** Prox Sense 1 */

	// program LED current
	si1145_write8(si1145, SI1145_REG_PSLED21, 0x03); // 20mA for LED 1 only
	si1145_writeParam(si1145, SI1145_PARAM_PS1ADCMUX, SI1145_PARAM_ADCMUX_LARGEIR);
	// prox sensor #1 uses LED #1
	si1145_writeParam(si1145, SI1145_PARAM_PSLED12SEL, SI1145_PARAM_PSLED12SEL_PS1LED1);
	// fastest clocks, clock div 1
	si1145_writeParam(si1145, SI1145_PARAM_PSADCGAIN, 0);
	// take 511 clocks to measure
	si1145_writeParam(si1145, SI1145_PARAM_PSADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
	// in prox mode, high range
	si1145_writeParam(si1145, SI1145_PARAM_PSADCMISC,
	SI1145_PARAM_PSADCMISC_RANGE | SI1145_PARAM_PSADCMISC_PSMODE);

	si1145_writeParam(si1145, SI1145_PARAM_ALSIRADCMUX, SI1145_PARAM_ADCMUX_SMALLIR);
	// fastest clocks, clock div 1
	si1145_writeParam(si1145, SI1145_PARAM_ALSIRADCGAIN, 0);
	// take 511 clocks to measure
	si1145_writeParam(si1145, SI1145_PARAM_ALSIRADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
	// in high range mode
	si1145_writeParam(si1145, SI1145_PARAM_ALSIRADCMISC, SI1145_PARAM_ALSIRADCMISC_RANGE);

	// fastest clocks, clock div 1
	si1145_writeParam(si1145, SI1145_PARAM_ALSVISADCGAIN, 0);
	// take 511 clocks to measure
	si1145_writeParam(si1145, SI1145_PARAM_ALSVISADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
	// in high range mode (not normal signal)
	si1145_writeParam(si1145, SI1145_PARAM_ALSVISADCMISC, SI1145_PARAM_ALSVISADCMISC_VISRANGE);

	/************************/

	// measurement rate for auto
	si1145_write8(si1145, SI1145_REG_MEASRATE0, 0xFF); // 255 * 31.25uS = 8ms

	// auto run
	si1145_write8(si1145, SI1145_REG_COMMAND, SI1145_PSALS_AUTO);

	return true;
}
/**
 * @brief Reset the sensor's registers to an initial state
 *
 */
void si1145_reset(si1145_t *si1145)
{
	si1145_write8(si1145, SI1145_REG_MEASRATE0, 0);
	si1145_write8(si1145, SI1145_REG_MEASRATE1, 0);
	si1145_write8(si1145, SI1145_REG_IRQEN, 0);
	si1145_write8(si1145, SI1145_REG_IRQMODE1, 0);
	si1145_write8(si1145, SI1145_REG_IRQMODE2, 0);
	si1145_write8(si1145, SI1145_REG_INTCFG, 0);
	si1145_write8(si1145, SI1145_REG_IRQSTAT, 0xFF);

	si1145_write8(si1145, SI1145_REG_COMMAND, SI1145_RESET);
	si1145_delay(10);
	si1145_write8(si1145, SI1145_REG_HWKEY, 0x17);

	si1145_delay(10);
}

//////////////////////////////////////////////////////

/**
 * @brief Get the current UV reading
 *
 * @return uint16_t The the UV index * 100 (divide by 100 to get the index)
 */
uint16_t si1145_readUV(si1145_t *si1145) { return si1145_read16(si1145, 0x2C); }

/**
 * @brief Get the Visible & IR light levels
 *
 * @return uint16_t The Visible & IR light levels
 */
uint16_t si1145_readVisible(si1145_t *si1145) { return si1145_read16(si1145, 0x22); }

// returns IR light levels
/**
 * @brief Get the Infrared light level
 *
 * @return uint16_t The Infrared light level
 */
uint16_t si1145_readIR(si1145_t *si1145) { return si1145_read16(si1145, 0x24); }

// returns "Proximity" - assumes an IR LED is attached to LED
/**
 * @brief Gets the Proximity measurement - **Requires an attached IR LED**
 *
 * @return uint16_t The proximity measurement
 */
uint16_t si1145_readProx(si1145_t *si1145) { return si1145_read16(si1145, 0x26); }

/*********************************************************************/

static uint8_t si1145_writeParam(si1145_t *si1145, uint8_t p, uint8_t v)
{
	si1145_write8(si1145, SI1145_REG_PARAMWR, v);
	si1145_write8(si1145, SI1145_REG_COMMAND, p | SI1145_PARAM_SET);
	return si1145_read8(si1145, SI1145_REG_PARAMRD);
}

uint8_t si1145_readParam(si1145_t *si1145, uint8_t p)
{
	si1145_write8(si1145, SI1145_REG_COMMAND, p | SI1145_PARAM_QUERY);
	return si1145_read8(si1145, SI1145_REG_PARAMRD);
}

/*********************************************************************/

static uint8_t si1145_read8(si1145_t *si1145, uint8_t reg)
{
	uint8_t buffer[1] = {reg};
	si1145_i2c_write_then_read(si1145, buffer, 1, buffer, 1);
	return buffer[0];
}

static uint16_t si1145_read16(si1145_t *si1145, uint8_t a)
{
	uint8_t buffer[2] = {a, 0};
	si1145_i2c_write_then_read(si1145, buffer, 1, buffer, 2);
	return ((uint16_t)buffer[0]) | ((uint16_t)buffer[1] << 8);
}

static void si1145_write8(si1145_t *si1145, uint8_t reg, uint8_t val)
{
	uint8_t buffer[2] = {reg, val};
	si1145_i2c_write(si1145, buffer, 2);
}
