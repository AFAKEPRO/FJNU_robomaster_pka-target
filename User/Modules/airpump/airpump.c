#include "airpump.h"
#include "bsp_gpio.h"
#include "stdlib.h"


static uint8_t idx = 0; // register idx,是该文件的全局索引,在注册时使用
static AirPumpInstance *airpump_instance[AIR_PUMP_CNT] = {NULL}; // 会在control任务中遍历该指针数组进行气泵开关

AirPumpInstance *AirPumpInit(airpump_init_Config_s *config)
{
    AirPumpInstance *instance = (AirPumpInstance *)malloc(sizeof(AirPumpInstance));
    memset(instance,0,sizeof(AirPumpInstance));

    instance->name                 	= config->name;
    instance->state                	= config->state;
    instance->airpump_gpio_instance = GPIORegister(&config->gpio_init_config);
    
    airpump_instance[idx++] = instance;
    return instance;

}

void AirPumpSetStatus(AirPumpInstance *airpump,AirpumpState_e state)
{
    airpump->state = state;
}

uint8_t debug_air;
void AirPumpTask()
{
    AirPumpInstance *airpump;
    for(size_t i = 0;i<idx;++i)
    {
        airpump = airpump_instance[i];
        if(airpump->state == AIRPUMP_ON)
		    GPIOSet(airpump->airpump_gpio_instance);
	    else
		    GPIOReset(airpump->airpump_gpio_instance);

    }
  	debug_air=HAL_GPIO_ReadPin(airpump->airpump_gpio_instance->GPIOx,airpump->airpump_gpio_instance->GPIO_Pin);

}




