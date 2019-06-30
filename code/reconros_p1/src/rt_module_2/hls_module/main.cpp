#include <stdint.h>

void process_module(uint32_t ram[1]) {
#pragma HLS INTERFACE bram port=ram
#pragma HLS RESOURCE variable=ram core=RAM_1P_BRAM

	//Read sonar sensor value from BRAM
	uint8_t sonar = (uint8_t) ram[0];
	
	//Write resulting state based on detected distance in cm
	if (sonar < 100)
	{
		ram[0] = 1;
	}
	else if (sonar > 120)
	{
		ram[0] = 2;
	}
	else
	{
		ram[0] = 0;
	}
  
	return;
}
