/*
 * loadcell_driver.hpp
 *
 *  Created on: Oct 25, 2024
 *      Author: jmorritt
 */

#ifndef INC_LOADCELL_DRIVER_HPP_
#define INC_LOADCELL_DRIVER_HPP_

#include <stdint.h>

#include "driver_module.hpp"
//#include "libcanard_module.hpp"
#include "can_params.hpp"

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_adc.h"
#include "stm32l4xx_hal_adc_ex.h"

#include "adc.h"

#include "main.h"

#include <com.aeronavics.LoadcellInfo.h>


#define TELEM_MOVING_AVERAGE_BINS 			10

#define LOADCELL_ADC_MIN								100
#define LOADCELL_ADC_MAX								4000
#define LOADCELL_MEASURED_WEIGHT_MIN		0.0
#define LOADCELL_MEASURED_WEIGHT_MAX		7.5


typedef struct{
	uint8_t array_index;		//current index in array for moving average
	//uint16_t value;			//calculated value
	uint32_t raw_values[TELEM_MOVING_AVERAGE_BINS];	//array of raw values used to calculate a moving average
}telemBase_t;

class Loadcell_driver : public Driver_module
{
	public:
		void sync_update_unthrottled(void);
		void sync_update_100Hz(void);
		void sync_update_10Hz(void);
		void sync_update_1Hz(void);
		void handle_rx_can(const CanardRxTransfer * transfer, uint64_t data_type_signature, uint16_t data_type_id, uint8_t* inout_transfer_id, uint8_t priority, const void* payload, uint16_t payload_len);

		void adc_callback(ADC_HandleTypeDef *AdcHandle);

		static Loadcell_driver& get_driver(void);
		~Loadcell_driver(void);

	private:

		Driver_state prev_state;

		float measured_weight;

		volatile uint32_t adc_reading;
		volatile telemBase_t loadcell_sensor;

		Loadcell_driver(void);
		Loadcell_driver(Loadcell_driver const&);		// Poisoned.
		void operator =(Loadcell_driver const&);	// Poisoned.

		void transmit_telemetry(void);
		void calculate_weight(void);

		float map_loadcell_weight(float weight);
		uint32_t calculate_average(volatile uint32_t* array, uint8_t size);

};

#endif /* INC_LOADCELL_DRIVER_HPP_ */
