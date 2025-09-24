#include "common/common.h"

struct calibration_vars {
	int32 t_fine;          // ref: 4.2.3 Compensation formulas (https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)
	uint16 dig_T1;
	int16 dig_T2;
	int16 dig_T3;
	uint16 dig_P1;
	int16 dig_P2;
	int16 dig_P3;
	int16 dig_P4;
	int16 dig_P5;
	int16 dig_P6;
	int16 dig_P7;
	int16 dig_P8;
	int16 dig_P9;
	uint8 dig_H1;
	int16 dig_H2;
	uint8 dig_H3;
	int16 dig_H4;
	int16 dig_H5;
	int8 dig_H6;
};

typedef struct {
	uint16 _spi_address;
	uint8 cs_pin;
	uint64 cooldown_ms;
	bool last_change; // must start at true / 1 / high
	struct calibration_vars calvars;
	HAL_PinOut step;
	HAL_SPI bus;
	HAL_Sleep sleep;
} BME280;

extern void cs_select(BME280* dev);
extern void cs_deselect(BME280* dev);
extern BME280 new_BME280(HAL_PinOut pin, uint8 _spi_address, uint8 cs_pin, uint64 cooldown_ms,
						 bool last_change, HAL_SPI bus, HAL_Sleep sleep);
extern int32 compensate_temp(BME280* dev, int32 adc_T);
extern uint32 compensate_pressure(BME280* dev, int32 adc_P);
extern uint32 compensate_humidity(BME280* dev, int32 adc_H);
extern void write_register(BME280* dev, uint8 reg, uint8 data);
extern void read_registers(BME280* dev, uint8 reg, uint8* buf, uint16 len);
extern void read_compensation_parameters(BME280* dev);
extern void bme280_read_raw(BME280* dev, int32* humidity, int32* pressure, int32* temperature);