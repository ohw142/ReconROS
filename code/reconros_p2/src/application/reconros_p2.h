#ifndef RECONROS_P2_H
#define RECONROS_P2_H

#ifdef __cplusplus
extern "C" {
#endif

int HWT_init();

uint32_t HWT_start_module_1 (uint32_t* data);
void HWT_waiton_module_1 ();
uint32_t HWT_start_module_2 (uint32_t* data);
void HWT_waiton_module_2 ();
uint32_t HWT_start_module_3 (uint32_t* data);
void HWT_waiton_module_3 ();

int HWT_reconfigure(int slot, char* bitstream);
int HWT_shutdown();

#ifdef __cplusplus
}
#endif

#endif /* RECONROS_P2_H */