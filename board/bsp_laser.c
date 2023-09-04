#include "bsp_laser.h"
#include "main.h"

void laser_on(void)
{
    HAL_GPIO_WritePin(LAZER_GPIO_Port, LAZER_Pin, GPIO_PIN_SET);
}
void laser_off(void)
{
    HAL_GPIO_WritePin(LAZER_GPIO_Port, LAZER_Pin, GPIO_PIN_RESET);
}
