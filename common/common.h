#pragma once

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

typedef int64_t int64;
typedef int32_t int32;
typedef int16_t int16;
typedef int8_t int8;

typedef uint64_t uint64;
typedef uint32_t uint32;
typedef uint16_t uint16;
typedef uint8_t uint8;

typedef ptrdiff_t isize;

#define byte uint8
#define true 1
#define false 0

typedef float float32;
typedef double float64;

typedef long double float128;

typedef int error;

typedef int64 unix_nano;

typedef unix_nano (*HAL_unix_nano)(void);

typedef struct {
	error (*tx)(byte* write_data, isize wlen, byte* read_data, isize rlen);
} HAL_SPI;

typedef struct {
	error (*tx)(uint16 i2c_address, byte* write_data, isize wlen, byte* read_data, isize rlen);
} HAL_I2C;

typedef void (*HAL_Sleep)(int64 nanoseconds);

typedef error (*HAL_Reader)(byte* buffer, isize buffer_len, isize* read_len);
typedef error (*HAL_Writer)(byte* data_start, isize data_len, isize* written_len);

typedef void (*HAL_PinOut)(bool);
typedef bool (*HAL_PinIn)(void);