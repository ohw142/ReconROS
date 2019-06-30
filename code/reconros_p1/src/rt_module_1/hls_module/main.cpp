#include <stdint.h>

void process_module(uint32_t ram[1]) {
#pragma HLS INTERFACE bram port=ram
#pragma HLS RESOURCE variable=ram core=RAM_1P_BRAM

	//Read input sensor value from BRAM
	uint8_t leds = (uint8_t) ram[0];

	int8_t start= -1;
    int8_t stop = -1;
        
	//Find middle of detected white stripe
	for(int i = 0; i<8; i++) 
	{
		if(leds&(1<<i))
		{
            start = i;	
            break;
		}
	}
	
	for(int i = 7; i>=0; i--) 
	{
		if(leds&(1<<i))
		{
            stop = i;	
            break;
        }
	}
        
    int8_t middle=(start+stop+1)>>1;
	
	//Write output steering value based on stripe position
    if(middle<0)
	{
		ram[0]=0;
    }else if(middle<1)
	{
		ram[0]=1;
	}else if(middle<3)
	{
		ram[0]=2;
	}else if(middle<5)
	{
		ram[0]=3;
	}else if(middle<7)
	{
		ram[0]=4;
	}else
	{
		ram[0]=5;
	}

	return;
}
