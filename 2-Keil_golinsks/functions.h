#include <stdint.h>
void PortF_Init(void);
void spin(void);
void PortL_Init(void);
void PortM_Init(void);
void ToF_Init(void);
void UART_Init(void);
void aquire_send_data(void);
void I2C_Init(void);
void PortG_Init(void);
void VL53L1X_XSHUT(void);
void SysTick_Wait_Faster(uint32_t delay);
