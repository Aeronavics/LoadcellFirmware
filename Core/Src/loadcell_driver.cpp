/*
 * loadcell_driver.cpp
 *
 *  Created on: Oct 25, 2024
 *      Author: jmorritt
 */

#include "loadcell_driver.hpp"

/**
  * @brief  Creates and returns a singleton instance of the Loadcell driver
  * @retval Singleton instance of the Loadcell driver
  */
Loadcell_driver& Loadcell_driver::get_driver(void)
{
	// Create a singleton for the driver.
		static Loadcell_driver singleton;

		// Return the driver.
		return singleton;
}

/**
  * @brief  Loadcell driver deconstructor
  * @retval None
  */
Loadcell_driver::~Loadcell_driver(void)
{
	return;
}

/**
  * @brief  Handles loadcell drivers unthrottled loop
  * @retval None
  */
void Loadcell_driver::sync_update_unthrottled(void)
{
	// If we're not in STATE_NORMAL, then we don't do anything yet.
	if (this_state != DRIVER_STATE_NORMAL)
	{
			return;
	}
	return;
}

/**
  * @brief  Handles loadcell drivers 100Hz loop
  * @retval None
  */
void Loadcell_driver::sync_update_100Hz(void)
{
	// If we're not in STATE_NORMAL, then we don't do anything yet.
	if (this_state != DRIVER_STATE_NORMAL)
	{
			return;
	}

	return;
}

/**
  * @brief  Handles loadcell drivers 10Hz loop
  * @note   Transmits loadcell status telemetry in this loop.
  * @retval None
  */
void Loadcell_driver::sync_update_10Hz(void)
{
	// If we're not in STATE_NORMAL, then we don't do anything yet.
	if (this_state != DRIVER_STATE_NORMAL)
	{
			return;
	}

	// Calculate weight moving averages
	calculate_weight();

	// Transmit loadcell telemetry
	transmit_telemetry();
	return;
}

/**
  * @brief  Handles loadcell drivers 1Hz loop
  * @note   1Hz loop handles all housekeeping.
  * @retval None
  */
void Loadcell_driver::sync_update_1Hz(void)
{
	// Run the driver housekeeping state machine.

	// If we don't specify a new state, assume we will just remain in the current state.
	Driver_state next_state = this_state;

	// Select behaviour depending on the current state.
	switch (this_state)
	{
		case DRIVER_STATE_UNKNOWN:
		{
			// If we don't know which state we are in, then we probably want to initialise.
			next_state = DRIVER_STATE_INIT;
			break;
		}
		case DRIVER_STATE_INIT:
		{
			MX_DMA_Init();
			MX_ADC1_Init();
			/* Intialise ADCs*/
			if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
			{
				/* Calibration Error */
				Error_Handler();
			}

			if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc_reading, 1) != HAL_OK)
			{
				/* Start Error */
				Error_Handler();
			}

//			Libcanard_module::get_driver().sendLog(impl_::LogLevel::Debug, "Loadcell Driver initialized");
			next_state = DRIVER_STATE_NORMAL;
			break;
		}
		case DRIVER_STATE_NORMAL:
		{
			break;
		}
		case DRIVER_STATE_ERROR:
		{
			// We'll attempt to reinitialise; that's about all we can hope for.
			next_state = DRIVER_STATE_INIT;
			break;
		}
		default:
		{
			// We shouldn't ever end up here.
			this_state = DRIVER_STATE_UNKNOWN;
			next_state = DRIVER_STATE_UNKNOWN;
			break;
		}
	}

	// Advance to the next state.
	prev_state = this_state;
	this_state = next_state;

	// All done.

	return;
}

/**
  * @brief  Handles incoming CAN messages
  * @param  transfer : The CAN message
  * @param  data_type_signature : CAN signature of the transfer
  * @param	data_type_id : CAN ID of the transfer
  * @param	inout_transfer_id : transfer id of the transfer
  * @param	priority : Priority of the transfer
  * @param  payload : CAN payload of the transfer
  * @param	payload_len : Length of the CAN payload
  * @note   Listens to the incoming can messages and processes them if required.
  * @retval None
  */
void Loadcell_driver::handle_rx_can(const CanardRxTransfer * transfer, uint64_t data_type_signature, uint16_t data_type_id, uint8_t* inout_transfer_id, uint8_t priority, const void* payload, uint16_t payload_len)
{

}

/**
  * @brief  Transmits Loadcell driver telemetry
  * @note   Tramsmits a com_aronavics_LoadcellInfo msg over CAN
  * 				after checking for errors and calculating values
  * @retval None
  */
void Loadcell_driver::transmit_telemetry(void)
{
	uint8_t transfer_id = 0;
	uint32_t length = 0;

	com_aeronavics_LoadcellInfo loadcell_info;

	loadcell_info.sensor_id = get_can_param_by_id(CAN_PARAM_IDX_SENSOR_ID)->value.integer_value;

	loadcell_info.weight = measured_weight;

	uint8_t loadcell_info_buffer[COM_AERONAVICS_LOADCELLINFO_MAX_SIZE];
	length = com_aeronavics_LoadcellInfo_encode(&loadcell_info, loadcell_info_buffer);
	driverhost_broadcast_can(
		nullptr,
		COM_AERONAVICS_LOADCELLINFO_SIGNATURE,
		COM_AERONAVICS_LOADCELLINFO_ID,
		&transfer_id,
		CAN_TRANSFER_PRIORITY_MEDIUM,
		&loadcell_info_buffer,
		length,
		this
	);

}

/**
  * @brief  Calculates the weight measured by the loadcell
  * @note   Calculates the moving average of the loadcell sensor.
  *					measured_weight is then updated to the calculated value.
  * @retval None
  */
void Loadcell_driver::calculate_weight(void)
{
	// get moving average of ADC readings
	float average_adc = calculate_average(loadcell_sensor.raw_values, TELEM_MOVING_AVERAGE_BINS);

	measured_weight = map_loadcell_weight(average_adc);
}

/**
  * @brief  Maps spray weight to percentage
  * @param  weight : Measured weight of the spray
  * @note   Calculates a percentage of spray remaining based on a given weight.
  *					The weight of the spray is assumed to be within MIN_SPRAY_WEGHT & MIN_SPRAY_WEGHT.
  *					The output percentage will be between MIN_SPRAY_PERCENT & MAX_SPRAY_PERCENT.
  * @retval A percentage of spray remaining
  */
float Loadcell_driver::map_loadcell_weight(float weight)
{
	float calculated_weight =  (weight - LOADCELL_ADC_MIN) * (LOADCELL_MEASURED_WEIGHT_MAX - LOADCELL_MEASURED_WEIGHT_MIN) / (LOADCELL_ADC_MAX - LOADCELL_ADC_MIN) + LOADCELL_MEASURED_WEIGHT_MIN;

	if (calculated_weight < LOADCELL_MEASURED_WEIGHT_MIN)
	{
		return LOADCELL_MEASURED_WEIGHT_MIN;
	}
	else
	{
		return calculated_weight;
	}
}

/**
  * @brief  Calculates the average of a given array
  * @param  array : Array to calculate average over
  * @param  size	: Size of the array
  * @retval uint16_t average of the array
  */
uint32_t Loadcell_driver::calculate_average(volatile uint32_t* array, uint8_t size)
{
    float temp_value = 0;
    for(uint8_t i = 0; i < size; i++){
    	temp_value += array[i];
    }
    return temp_value / size;
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : ADC handle
  * @note   Adds the ADC readings into their respective sensor structures
  *					and calls the functions to calculate the moving averages for each sensor.
  * @retval None
  */
void Loadcell_driver::adc_callback(ADC_HandleTypeDef *AdcHandle)
{
	loadcell_sensor.raw_values[loadcell_sensor.array_index%TELEM_MOVING_AVERAGE_BINS] = adc_reading;
	loadcell_sensor.array_index = ++loadcell_sensor.array_index % TELEM_MOVING_AVERAGE_BINS;
}

Loadcell_driver::Loadcell_driver(void)
{
    // Initialise the driver state machine.
    prev_state = DRIVER_STATE_UNKNOWN;
    // All done.
    return;
}

