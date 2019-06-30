#define  _GNU_SOURCE

#include "reconos.h"
#include "reconos_app.h"
#include "mbox.h"
#include "timer.h"

#include "utils.h"
#include "lib/runtime/arch/arch.h"
#include "lib/runtime/private.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

#include "reconros_p1.h"

#define BLOCK_SIZE 2048

#define log(...) printf(__VA_ARGS__); fflush(stdout)

//array to hold information about each HWT slot
struct reconos_thread **reconos_hwts; 

static int reconfigure(char* filename,unsigned int partial){

	FILE *bitfile;
	unsigned int size;
	char *bitstream;

	bitfile = fopen(filename, "rb");
	if(!bitfile){
		log("Error opening bitfile %s\n",filename);
		return -1;
	}

	fseek(bitfile,0L,SEEK_END);
	size=ftell(bitfile);
	rewind(bitfile);

	bitstream = (char *)malloc(size*sizeof(char));
	if(!bitstream){
		log("Error allocating memory for bitstream %s\n",filename);
		return -1;
	}
	fread(bitstream, sizeof(char), size, bitfile);
	fclose(bitfile);

	int fd_partial = open("/sys/devices/soc0/amba/f8007000.devcfg/is_partial_bitstream", O_RDWR);
	if(fd_partial < 0){
		log("Failed to open xdevcfg attribute 'is_partial_bitstream' when configuring %s\n",filename);
		return -1;
	}

	char partial_flag[2];
	if(!partial) {
		strcpy(partial_flag,"0");
	}
	else {
		strcpy(partial_flag,"1");
	}
	write(fd_partial, partial_flag, 2);
	close(fd_partial);

	fd_partial = open("/dev/xdevcfg", O_RDWR);
	if(fd_partial < 0){
		log("Failed to open xdevcfg device when configuring %s\n",filename);
		return -1;
	}
	log("Opened xdevcfg. Configuring with %u bytes\n",size);
	write(fd_partial, bitstream, size);
	int fd_finish_flag = open("/sys/devices/soc0/amba/f8007000.devcfg/prog_done", O_RDWR);
	char finish_flag = '0';

	//wait until reconfiguration is finished
	while(finish_flag != '1'){
		read(fd_finish_flag,&finish_flag,1);
	}
	log("Reconfiguration with bitfile %s finished\n",filename);
	close(fd_partial);
	close(fd_finish_flag);

	return 0;

}

int HWT_init()
{
	//Do not load initial bitstream, since it is already loaded at boot (before the kernel module is loaded)
	/*
	if(reconfigure(bitstream,0) < 0){
		printf("Failed to load static bitfile of sortdemo\n");
		return -1;
	}
	*/

	//init ReconOS components and set clock frequency
	reconos_init();
	reconos_app_init();
	reconos_clock_threads_set(100000);

	//create HWTs (this spawns a delegate thread for each slot)
	int num_hwts = 4;
	log("creating %d hw-threads:", num_hwts);
	reconos_hwts = (struct reconos_thread **) malloc(num_hwts * sizeof(struct reconos_thread));
	if(!reconos_hwts){
		log("Could not allocate memory for %d hardware threads\n",num_hwts);
	}
	for (int i = 0; i < num_hwts; i++) {
		log(" %d", i);
		reconos_hwts[i]=reconos_thread_create_hwt_reconf();
	}
	log("\n");
	
	return 0;
}

uint32_t HWT_process_module_1 (uint32_t* data)
{
	mbox_put(resources_module_1_addr, (uint32_t)(data));
	//wait for HWT
	mbox_get(resources_module_1_ack);

	//As with all other modules, the first data element is only returned for convenience. The calling process can read the entire data block afterwards.
	return data[0]; 
}

uint32_t HWT_process_module_2 (uint32_t* data)
{
	mbox_put(resources_module_2_addr, (uint32_t)(data));
	//wait for HWT
	mbox_get(resources_module_2_ack);

	return data[0];
}

uint32_t HWT_process_module_3 (uint32_t* data)
{
	mbox_put(resources_module_3_addr, (uint32_t)(data));
	//wait for HWT
	mbox_get(resources_module_3_ack);

	return data[0];
}

uint32_t HWT_process_module_4 (uint32_t* data)
{
	mbox_put(resources_module_4_addr, (uint32_t)(data));
	//wait for HWT
	mbox_get(resources_module_4_ack);

	return data[0];
}

int HWT_reconfigure(int slot, char* bitstream)
{
	printf("Reconfiguring: Suspending thread\n");
	reconos_thread_suspend_block(reconos_hwts[slot]);

	printf("Reconfiguring: Loading partial bitstream\n");
	if(reconfigure(bitstream,1) < 0) return -1;

	printf("Reconfiguring: Resuming thread\n");
	reconos_thread_resume(reconos_hwts[slot],slot);
	
	return 0;
}

int HWT_shutdown()
{
	timer_cleanup();
	reconos_app_cleanup();
	reconos_cleanup();

	return 0;
}
