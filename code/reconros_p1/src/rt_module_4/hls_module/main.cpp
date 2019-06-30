#include <stdint.h>
#include <ap_fixed.h>
#include <ap_int.h>

//define two additional output signals for both PWM duty cycle values with 12-bit fixed point precision
void process_module(uint32_t ram[2], ap_ufixed<12,0> &pwm_duty_0, ap_ufixed<12,0> &pwm_duty_1) {
#pragma HLS INTERFACE bram port=ram
#pragma HLS RESOURCE variable=ram core=RAM_1P_BRAM

	ap_ufixed<13,1> input_normalized;
	ap_ufixed<12,0> output;

	if(ram[0] == 0) //received a steering signal
	{
		//limit input value to allowed range
		if(ram[1] < 65)  ram[1] = 65;
		if(ram[1] > 115) ram[1] = 115;
		
		//calculate normalized input (0.0 to 1.0)
		input_normalized = (ap_ufixed<13,1>) (((ap_ufixed<20,8>) ram[1] - (ap_ufixed<20,8>) 65.0) / (ap_ufixed<20,8>) 50.0);
		
		//calculate output value within calibrated range
		output = (ap_ufixed<12,0>) 0.055 + ((ap_ufixed<12,0>) 0.036 * input_normalized);
		
		//send final duty cycle to PWM controller
		pwm_duty_0 = output;
	}
	else if (ram[0] == 1) //received a throttle signal
	{
		//limit input value to allowed range
		if(ram[1] < 65)  ram[1] = 40;
		if(ram[1] > 115) ram[1] = 140;
		
		//calculate normalized input (0.0 to 1.0)
		input_normalized = (ap_ufixed<13,1>) (((ap_ufixed<20,8>) ram[1] - (ap_ufixed<20,8>) 40.0) / (ap_ufixed<20,8>) 100.0);
		
		//calculate output value within calibrated range
		output = (ap_ufixed<12,0>) 0.050 + ((ap_ufixed<12,0>) 0.048 * input_normalized);
		
		//send final duty cycle to PWM controller
		pwm_duty_1 = output;
	}

	return;
}
