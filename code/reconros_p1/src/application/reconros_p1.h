#ifndef RECONROS_P1_H
#define RECONROS_P1_H

#ifdef __cplusplus
extern "C" {
#endif

int HWT_init();

uint32_t HWT_process_module_1 (uint32_t* data);
uint32_t HWT_process_module_2 (uint32_t* data);
uint32_t HWT_process_module_3 (uint32_t* data);
uint32_t HWT_process_module_4 (uint32_t* data);

int HWT_reconfigure(int slot, char* bitstream);
int HWT_shutdown();

#ifdef __cplusplus
}
#endif

#endif /* RECONROS_P1_H */