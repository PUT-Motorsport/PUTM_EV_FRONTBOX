#pragma once

using adc_raw_t = uint16_t;

struct __attribute__((packed)) ADC1_data {
	adc_raw_t apps_2;
};

struct __attribute__((packed)) ADC2_data {
	adc_raw_t suspension_left;
	adc_raw_t suspension_right;
	adc_raw_t brake_pressure_front;
	adc_raw_t brake_pressure_rear;
	adc_raw_t apps_1;
};

constexpr auto ADC1_BUFFER_SIZE {sizeof(ADC1_data) / sizeof(adc_raw_t)};
constexpr auto ADC2_BUFFER_SIZE {sizeof(ADC2_data) / sizeof(adc_raw_t)};
