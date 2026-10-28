#ifndef __AIRPUMP_H
#define __AIRPUMP_H

#define AIR_PUMP_CNT 4
#include "bsp_gpio.h"

typedef enum{

    AIRPUMP_OFF = 0,
    AIRPUMP_ON,

}AirpumpState_e;

typedef enum{

    EE_AIRPUMP,
    LEFT_AIRPUMP,
    RIGHT_AIRPUMP,

}Airpumpname_e;

#pragma pack(1)

typedef struct 
{
    GPIO_Init_Config_s gpio_init_config;
    AirpumpState_e state;
    Airpumpname_e name;

}airpump_init_Config_s;


typedef struct 
{  
    GPIOInstance *airpump_gpio_instance;
    AirpumpState_e state;
    Airpumpname_e name;

}AirPumpInstance;

#pragma pack()

AirPumpInstance *AirPumpInit(airpump_init_Config_s *config);
void AirPumpSetStatus(AirPumpInstance *airpump,AirpumpState_e state);
void AirPumpTask();





















#endif
