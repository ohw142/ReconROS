#include <stdint.h>
#include <ap_fixed.h>
#include <ap_int.h>
#include <hls_stream.h>
#include <ap_axi_sdata.h>

#define DATA_WIDTH_IN 	24
#define DATA_WIDTH_OUT 	24

#define abs(value) ((value<0)?-value:value)

typedef ap_axiu<DATA_WIDTH_IN,1,1,1> data_in_t; //defines AXIS signals
typedef hls::stream<data_in_t> stream_in_t;

typedef ap_axiu<DATA_WIDTH_OUT,1,1,1> data_out_t; //defines AXIS signals
typedef hls::stream<data_out_t> stream_out_t;

using namespace std;

//This module implements basically the same functionality as module_2, but in the HSL color space to help with line detection
//It uses separate saturation and luminance thresholds, therefore it reads/writes one word more to BRAM
void process_module(uint32_t ram[32], stream_in_t& inStream, stream_out_t& outStream) {
#pragma HLS INTERFACE bram port=ram
#pragma HLS RESOURCE variable=ram core=RAM_1P_BRAM
#pragma HLS INTERFACE axis port=outStream
#pragma HLS INTERFACE axis port=inStream

  data_in_t dataIn;

  bool startFrame = false;
  bool endLine = false;

  uint32_t white_pixels[15] = {};
  #pragma HLS ARRAY_PARTITION variable=white_pixels complete dim=1

  uint32_t column_pixels = 1;
  uint32_t column = 0;
  int line = 0;

  //read in thresholds and column widths
  uint32_t sat_threshold = ram[0];
  uint32_t lum_threshold = ram[1];
  
  uint32_t columns[15] = {};
  for(int i = 0; i<15; i++)
  {
	#pragma HLS UNROLL
    columns[i] = ram[2+i];
  }

  ap_fixed<9,1> divider = 255;
  
  ap_fixed<9,1> ts = (sat_threshold & 0x000000ff) / divider;
  ap_fixed<9,1> tl = (lum_threshold & 0x000000ff) / divider;
	
  while(line < 720)
  {
  #pragma HLS pipeline
		
    data_out_t dataOut;
	
    //read next pixel
    inStream.read(dataIn);
    startFrame = dataIn.user;
    endLine = dataIn.last;
	
    if(startFrame) line = 0;
    if(endLine) line++;
    
	//perform color space conversion from RGB to HSL and count pixels that exceed configured thresholds
    ap_fixed<9,1> r,g,b;
    ap_fixed<9,1> cmax, cmin, delta;
    ap_fixed<9,1> s,l;

	uint8_t r_raw = (dataIn.data & 0xff0000) >> 16;
	uint8_t g_raw = (dataIn.data & 0x00ff00) >> 8;
	uint8_t b_raw = dataIn.data & 0x0000ff;
	
    r=r_raw / divider;
    g=g_raw / divider;
    b=b_raw / divider;
    
    if(r_raw>g_raw && r_raw>b_raw)
	{
      cmax = r;
    }else if (g_raw>r_raw && g_raw>b_raw)
	{
      cmax = g;
    }else
	{
      cmax = b;
    }

    if(r_raw<g_raw && r_raw<b_raw)
	{
      cmin = r;
    }else if (g_raw<r_raw && g_raw<b_raw)
	{
      cmin = g;
    }else
	{
      cmin = b;
    }
    delta = cmax-cmin;
    l=(cmax+cmin)/2;
    if (delta==0)
	{
      s=0;
    }else
	{
      s=delta/(1-abs(cmax+cmin-1));
    }
    
    if(s<ts && l>tl) 
	{
      white_pixels[column]++;
      dataIn.data = dataIn.data;
    }else
	{
      dataIn.data = 0x000000;
    }

    if(endLine)
	{
      column = 0;
      column_pixels = 0;
    } else 
	{
      if(column_pixels>=columns[column])
	  {
        column++;
      }
      column_pixels++;
    }
    
    //output stream
    dataOut.strb = -1;
    dataOut.keep = -1;
    dataOut.id = 0;
    dataOut.dest = 0;

    dataOut.user = dataIn.user;
    dataOut.last = dataIn.last;

    outStream.write(dataOut);
  }
  
  //write result to BRAM
  for (int i=0; i<15; i++)
  {
	#pragma HLS UNROLL
    ram[17+i] = white_pixels[i];
  }
	
  return;
}
