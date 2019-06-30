#include <stdint.h>

void process_module(uint32_t ram[15]) {
#pragma HLS INTERFACE bram port=ram
#pragma HLS RESOURCE variable=ram core=RAM_1P_BRAM

	uint32_t averaged_columns[15];
	uint32_t maxValue = 0;
    uint32_t result = 0;
	
	//buffer last 8 measurements in this array
	static uint32_t buffer[8][15];
	static int pointer = 0;
	
	//ringbuffer pointer wrap-around
	if((++pointer) >= 8)
	{
		pointer = 0;
	}
	
	//insert new measurement that is read from BRAM
	for(int s=0; s<15; s++)
	{
		buffer[pointer][s] = ram[s];
	}
	
	//average over measurements
	for(int i=0; i<15; i++)
	{
		averaged_columns[i] = 0;
		for(int j=0; j<8; j++)
		{
			averaged_columns[i] += buffer[j][i];
		}
		averaged_columns[i] /= 8;
	}
	
	//find column of maximum intensity
    for(int x=0; x<15; x++)
	{
		if(averaged_columns[x] > maxValue)
		{
			maxValue = averaged_columns[x];
			result = x;
		}
    }

	//only steer if white intensity exceeds a certain threshold
	if(maxValue > 10000)
	{
		ram[0] = result;
	}
	else
	{
		ram[0] = 7; //default (middle) steering angle
	}

	return;
}
