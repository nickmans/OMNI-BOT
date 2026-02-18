#include "battery_monitor.h"
#include "main.h"

extern ADC_HandleTypeDef hadc2;

static float s_last_voltage_v = 0.0f;

void BatteryMonitor_Init(void)
{
	s_last_voltage_v = 0.0f;
}

float BatteryMonitor_ReadVoltage_V(void)
{
	const uint32_t adc_full_scale = 65535u;
	const float vref = 3.3f;
	const float battery_divider_scale = 11.0f;

	if (HAL_ADC_Start(&hadc2) != HAL_OK)
	{
		return s_last_voltage_v;
	}

	if (HAL_ADC_PollForConversion(&hadc2, 5u) != HAL_OK)
	{
		(void)HAL_ADC_Stop(&hadc2);
		return s_last_voltage_v;
	}

	uint32_t raw = HAL_ADC_GetValue(&hadc2);
	(void)HAL_ADC_Stop(&hadc2);

	if (raw > adc_full_scale)
	{
		raw = adc_full_scale;
	}

	const float adc_pin_v = ((float)raw * vref) / (float)adc_full_scale;
	s_last_voltage_v = adc_pin_v * battery_divider_scale;
	return s_last_voltage_v;
}

