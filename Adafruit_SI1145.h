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
#ifndef _SI1145_H_
#define _SI1145_H_

#include <stdint.h>
#include <stdbool.h>

/* COMMANDS */
#define SI1145_PARAM_QUERY 0x80
#define SI1145_PARAM_SET 0xA0
#define SI1145_NOP 0x0
#define SI1145_RESET 0x01
#define SI1145_BUSADDR 0x02
#define SI1145_PS_FORCE 0x05
#define SI1145_ALS_FORCE 0x06
#define SI1145_PSALS_FORCE 0x07
#define SI1145_PS_PAUSE 0x09
#define SI1145_ALS_PAUSE 0x0A
#define SI1145_PSALS_PAUSE 0xB
#define SI1145_PS_AUTO 0x0D
#define SI1145_ALS_AUTO 0x0E
#define SI1145_PSALS_AUTO 0x0F
#define SI1145_GET_CAL 0x12

/* Parameters */
#define SI1145_PARAM_I2CADDR 0x00
#define SI1145_PARAM_CHLIST 0x01
#define SI1145_PARAM_CHLIST_ENUV 0x80
#define SI1145_PARAM_CHLIST_ENAUX 0x40
#define SI1145_PARAM_CHLIST_ENALSIR 0x20
#define SI1145_PARAM_CHLIST_ENALSVIS 0x10
#define SI1145_PARAM_CHLIST_ENPS1 0x01
#define SI1145_PARAM_CHLIST_ENPS2 0x02
#define SI1145_PARAM_CHLIST_ENPS3 0x04

#define SI1145_PARAM_PSLED12SEL 0x02
#define SI1145_PARAM_PSLED12SEL_PS2NONE 0x00
#define SI1145_PARAM_PSLED12SEL_PS2LED1 0x10
#define SI1145_PARAM_PSLED12SEL_PS2LED2 0x20
#define SI1145_PARAM_PSLED12SEL_PS2LED3 0x40
#define SI1145_PARAM_PSLED12SEL_PS1NONE 0x00
#define SI1145_PARAM_PSLED12SEL_PS1LED1 0x01
#define SI1145_PARAM_PSLED12SEL_PS1LED2 0x02
#define SI1145_PARAM_PSLED12SEL_PS1LED3 0x04

#define SI1145_PARAM_PSLED3SEL 0x03
#define SI1145_PARAM_PSENCODE 0x05
#define SI1145_PARAM_ALSENCODE 0x06

#define SI1145_PARAM_PS1ADCMUX 0x07
#define SI1145_PARAM_PS2ADCMUX 0x08
#define SI1145_PARAM_PS3ADCMUX 0x09
#define SI1145_PARAM_PSADCOUNTER 0x0A
#define SI1145_PARAM_PSADCGAIN 0x0B
#define SI1145_PARAM_PSADCMISC 0x0C
#define SI1145_PARAM_PSADCMISC_RANGE 0x20
#define SI1145_PARAM_PSADCMISC_PSMODE 0x04

#define SI1145_PARAM_ALSIRADCMUX 0x0E
#define SI1145_PARAM_AUXADCMUX 0x0F

#define SI1145_PARAM_ALSVISADCOUNTER 0x10
#define SI1145_PARAM_ALSVISADCGAIN 0x11
#define SI1145_PARAM_ALSVISADCMISC 0x12
#define SI1145_PARAM_ALSVISADCMISC_VISRANGE 0x20

#define SI1145_PARAM_ALSIRADCOUNTER 0x1D
#define SI1145_PARAM_ALSIRADCGAIN 0x1E
#define SI1145_PARAM_ALSIRADCMISC 0x1F
#define SI1145_PARAM_ALSIRADCMISC_RANGE 0x20

#define SI1145_PARAM_ADCCOUNTER_511CLK 0x70

#define SI1145_PARAM_ADCMUX_SMALLIR 0x00
#define SI1145_PARAM_ADCMUX_LARGEIR 0x03

/* REGISTERS */
#define SI1145_REG_PARTID 0x00
#define SI1145_REG_REVID 0x01
#define SI1145_REG_SEQID 0x02

#define SI1145_REG_INTCFG 0x03
#define SI1145_REG_INTCFG_INTOE 0x01
#define SI1145_REG_INTCFG_INTMODE 0x02

#define SI1145_REG_IRQEN 0x04
#define SI1145_REG_IRQEN_ALSEVERYSAMPLE 0x01
#define SI1145_REG_IRQEN_PS1EVERYSAMPLE 0x04
#define SI1145_REG_IRQEN_PS2EVERYSAMPLE 0x08
#define SI1145_REG_IRQEN_PS3EVERYSAMPLE 0x10

#define SI1145_REG_IRQMODE1 0x05
#define SI1145_REG_IRQMODE2 0x06

#define SI1145_REG_HWKEY 0x07
#define SI1145_REG_MEASRATE0 0x08
#define SI1145_REG_MEASRATE1 0x09
#define SI1145_REG_PSRATE 0x0A
#define SI1145_REG_PSLED21 0x0F
#define SI1145_REG_PSLED3 0x10
#define SI1145_REG_UCOEFF0 0x13
#define SI1145_REG_UCOEFF1 0x14
#define SI1145_REG_UCOEFF2 0x15
#define SI1145_REG_UCOEFF3 0x16
#define SI1145_REG_PARAMWR 0x17
#define SI1145_REG_COMMAND 0x18
#define SI1145_REG_RESPONSE 0x20
#define SI1145_REG_IRQSTAT 0x21
#define SI1145_REG_IRQSTAT_ALS 0x01

#define SI1145_REG_ALSVISDATA0 0x22
#define SI1145_REG_ALSVISDATA1 0x23
#define SI1145_REG_ALSIRDATA0 0x24
#define SI1145_REG_ALSIRDATA1 0x25
#define SI1145_REG_PS1DATA0 0x26
#define SI1145_REG_PS1DATA1 0x27
#define SI1145_REG_PS2DATA0 0x28
#define SI1145_REG_PS2DATA1 0x29
#define SI1145_REG_PS3DATA0 0x2A
#define SI1145_REG_PS3DATA1 0x2B
#define SI1145_REG_UVINDEX0 0x2C
#define SI1145_REG_UVINDEX1 0x2D
#define SI1145_REG_PARAMRD 0x2E
#define SI1145_REG_CHIPSTAT 0x30

#define SI1145_ADDR 0x60

typedef struct si1145 {
	void *dev_i2c;
	uint8_t addr;
} si1145_t;

bool si1145_init(si1145_t *si1145, uint8_t addr, void *dev_i2c);
void si1145_deinit(si1145_t *si1145);
void si1145_reset(si1145_t *si1145);

uint16_t si1145_readUV(si1145_t *si1145);
uint16_t si1145_readIR(si1145_t *si1145);
uint16_t si1145_readVisible(si1145_t *si1145);
uint16_t si1145_readProx(si1145_t *si1145);

extern void si1145_delay(uint32_t delay_seconds);
extern int8_t si1145_i2c_init(si1145_t *si1145);
extern void si1145_i2c_write_then_read(si1145_t *si1145, const uint8_t *buffer_tx, int buffer_tx_count, uint8_t *buffer_rx, int buffer_rx_count);
extern void si1145_i2c_write(si1145_t *si1145, const uint8_t *buffer_tx, int buffer_tx_count);

#endif
