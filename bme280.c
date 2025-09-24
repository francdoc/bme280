#include "common/common.h"
#include "bme280.h"

void cs_select(BME280* dev)
{
	dev->sleep(dev->cooldown_ms);
	bool next_change = !dev->last_change; // from high to low before reading/writing (active low)
	dev->step(next_change);
	dev->last_change = next_change;
	dev->sleep(dev->cooldown_ms);

	printf("CS Select - next_change: %d, last_change: %d\n", next_change, dev->last_change);
}

void cs_deselect(BME280* dev)
{
	dev->sleep(dev->cooldown_ms);
	bool next_change = !dev->last_change; // from low to high after reading/writing
	dev->step(next_change);
	dev->last_change = next_change;
	dev->sleep(dev->cooldown_ms);

	printf("CS Deselect - next_change: %d, last_change: %d\n", next_change, dev->last_change);
}

BME280 new_BME280(HAL_PinOut pin, uint8 _spi_address, uint8 cs_pin, uint64 cooldown_ms,
				  bool last_change, HAL_SPI bus, HAL_Sleep sleep)
{
	BME280 bme280;
	memset(&bme280, 0, sizeof(BME280)); // Initialize all members to 0

	// Initialize the device structure
	bme280._spi_address = _spi_address;
	bme280.cs_pin = cs_pin;
	bme280.cooldown_ms = cooldown_ms;
	bme280.last_change = last_change;
	bme280.sleep = sleep;
	bme280.step = pin;
	bme280.bus = bus;

	printf("Initialized new BME280 structure\n");

	return bme280;
}

int32 compensate_temp(BME280* dev, int32 adc_T)
{
	int32 var1, var2, T;
	var1 = ((((adc_T >> 3) - ((int32)dev->calvars.dig_T1 << 1))) * ((int32)dev->calvars.dig_T2)) >>
		   11;
	var2 = (((((adc_T >> 4) - ((int32)dev->calvars.dig_T1)) *
			  ((adc_T >> 4) - ((int32)dev->calvars.dig_T1))) >>
			 12) *
			((int32)dev->calvars.dig_T3)) >>
		   14;

	dev->calvars.t_fine = var1 + var2;
	T = (dev->calvars.t_fine * 5 + 128) >> 8;

	printf("Compensate Temp - adc_T: %d, var1: %d, var2: %d, t_fine: %d, T: %d\n", adc_T, var1,
		   var2, dev->calvars.t_fine, T);
	return T;
}

uint32 compensate_pressure(BME280* dev, int32 adc_P)
{
	int32 var1, var2;
	uint32 p;
	var1 = (((int32)dev->calvars.t_fine) >> 1) - (int32)64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32)dev->calvars.dig_P6);
	var2 = var2 + ((var1 * ((int32)dev->calvars.dig_P5)) << 1);
	var2 = (var2 >> 2) + (((int32)dev->calvars.dig_P4) << 16);
	var1 = (((dev->calvars.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
			((((int32)dev->calvars.dig_P2) * var1) >> 1)) >>
		   18;
	var1 = ((((32768 + var1)) * ((int32)dev->calvars.dig_P1)) >> 15);
	if (var1 == 0) {
		return 0;
	}

	p = (((uint32)(((int32)1048576) - adc_P) - (var2 >> 12))) * 3125;
	if (p < 0x80000000) {
		p = (p << 1) / ((uint32)var1);
	} else {
		p = (p / (uint32)var1) * 2;
	}

	var1 = (((int32)dev->calvars.dig_P9) * ((int32)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
	var2 = (((int32)(p >> 2)) * ((int32)dev->calvars.dig_P8)) >> 13;
	p = (uint32)((int32)p + ((var1 + var2 + dev->calvars.dig_P7) >> 4));

	printf("Compensate Pressure - adc_P: %d, var1: %d, var2: %d, p: %d\n", adc_P, var1, var2, p);
	return p;
}

uint32 compensate_humidity(BME280* dev, int32 adc_H)
{
	int32 v_x1_u32r;
	v_x1_u32r = (dev->calvars.t_fine - ((int32)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32)dev->calvars.dig_H4) << 20) -
					(((int32)dev->calvars.dig_H5) * v_x1_u32r)) +
				   ((int32)16384)) >>
				  15) *
				 (((((((v_x1_u32r * ((int32)dev->calvars.dig_H6)) >> 10) *
					  (((v_x1_u32r * ((int32)dev->calvars.dig_H3)) >> 11) + ((int32)32768))) >>
					 10) +
					((int32)2097152)) *
					   ((int32)dev->calvars.dig_H2) +
				   8192) >>
				  14));
	v_x1_u32r =
		(v_x1_u32r -
		 (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32)dev->calvars.dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

	printf("Compensate Humidity - adc_H: %d, v_x1_u32r: %d\n", adc_H, v_x1_u32r);
	return (uint32)(v_x1_u32r >> 12);
}

void write_register(BME280* dev, uint8 reg, uint8 data)
{
	uint8 buf[2];
	buf[0] = reg & 0x7f; // remove read bit as this is a write
	buf[1] = data;
	cs_select(dev);
	dev->bus.tx(buf, sizeof(buf), NULL, 0);
	cs_deselect(dev);
	sleep_ms(10);
}

void read_registers(BME280* dev, uint8 reg, uint8* buf, uint16 len)
{
	// For this particular device, we send the device the register we want to read
	// first, then subsequently read from the device. The register is auto incrementing
	// so we don't need to keep sending the register we want, just the first.
	reg |= dev->_spi_address;
	cs_select(dev);
	dev->bus.tx(&reg, sizeof(reg), NULL, 0);
	sleep_ms(10);
	dev->bus.tx(NULL, 0, buf, len);
	cs_deselect(dev);
	sleep_ms(10);
}

/* This function reads the manufacturing assigned compensation parameters from the device */
void read_compensation_parameters(BME280* dev)
{
	uint8 buffer[26];

	read_registers(dev, 0x88, buffer, 24);

	dev->calvars.dig_T1 = buffer[0] | (buffer[1] << 8);
	dev->calvars.dig_T2 = buffer[2] | (buffer[3] << 8);
	dev->calvars.dig_T3 = buffer[4] | (buffer[5] << 8);

	dev->calvars.dig_P1 = buffer[6] | (buffer[7] << 8);
	dev->calvars.dig_P2 = buffer[8] | (buffer[9] << 8);
	dev->calvars.dig_P3 = buffer[10] | (buffer[11] << 8);
	dev->calvars.dig_P4 = buffer[12] | (buffer[13] << 8);
	dev->calvars.dig_P5 = buffer[14] | (buffer[15] << 8);
	dev->calvars.dig_P6 = buffer[16] | (buffer[17] << 8);
	dev->calvars.dig_P7 = buffer[18] | (buffer[19] << 8);
	dev->calvars.dig_P8 = buffer[20] | (buffer[21] << 8);
	dev->calvars.dig_P9 = buffer[22] | (buffer[23] << 8);

	dev->calvars.dig_H1 = buffer[25];

	read_registers(dev, 0xE1, buffer, 8);

	dev->calvars.dig_H2 = buffer[0] | (buffer[1] << 8);
	dev->calvars.dig_H3 = (int8)buffer[2];
	dev->calvars.dig_H4 = buffer[3] << 4 | (buffer[4] & 0xf);
	dev->calvars.dig_H5 = (buffer[5] >> 4) | (buffer[6] << 4);
	dev->calvars.dig_H6 = (int8)buffer[7];

	printf("Compensation parameters:\n");
	printf("T1: %u, T2: %d, T3: %d\n", dev->calvars.dig_T1, dev->calvars.dig_T2,
		   dev->calvars.dig_T3);
	printf("P1: %u, P2: %d, P3: %d, P4: %d, P5: %d, P6: %d, P7: %d, P8: %d, P9: %d\n",
		   dev->calvars.dig_P1, dev->calvars.dig_P2, dev->calvars.dig_P3, dev->calvars.dig_P4,
		   dev->calvars.dig_P5, dev->calvars.dig_P6, dev->calvars.dig_P7, dev->calvars.dig_P8,
		   dev->calvars.dig_P9);
	printf("H1: %u, H2: %d, H3: %d, H4: %d, H5: %d, H6: %d\n", dev->calvars.dig_H1,
		   dev->calvars.dig_H2, dev->calvars.dig_H3, dev->calvars.dig_H4, dev->calvars.dig_H5,
		   dev->calvars.dig_H6);
}

void bme280_read_raw(BME280* dev, int32* humidity, int32* pressure, int32* temperature)
{
	uint8 buffer[8];

	read_registers(dev, 0xF7, buffer, 8);
	*pressure = ((uint32)buffer[0] << 12) | ((uint32)buffer[1] << 4) | (buffer[2] >> 4);
	*temperature = ((uint32)buffer[3] << 12) | ((uint32)buffer[4] << 4) | (buffer[5] >> 4);
	*humidity = (uint32)buffer[6] << 8 | buffer[7];

	printf("Raw readings - Temperature: %d, Pressure: %d, Humidity: %d\n", *temperature, *pressure,
		   *humidity);
}