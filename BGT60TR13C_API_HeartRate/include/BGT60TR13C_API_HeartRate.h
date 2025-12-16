#ifndef BGT60TR13C_API_HEARTRATE_H
#define BGT60TR13C_API_HEARTRATE_H

#include "driver/spi_master.h"

void BGT60TR13C_Init(spi_host_device_t spi);
void Start_Measuring(void);
void Stop_Measuring(void);
#endif