# OpenCMX

Dùng thu vi?n modbus
Khai báo các c?u hình t?i modbus_define.h
modbus_sticks d?t t?i ng?t SysTick_Handler
MODBUS = 1 b?t ch?c nang modbus
nh?n d? li?u modbus:incomming_modbus_serial(USART2->DR & (uint16_t)0x01FF) 
# Modbus library infomation ( 11-Jul-18)
http://www.simplymodbus.ca/index.html
This modbus was rebuilt on CCS drivers( C Compilers for Microchip PIC)
- Choose modbus mode : master of slave, most of cases is slave
- Data source ( slave )/destination (master): coils, inputs, hold_regs, input_regs
- get modbus data from uart data buffer by call modbus_slave_exe(rx2_data_buff, USART2_index) or modbus_master_exe(rx2_data_buff, USART2_index) in my code
- Some other informations : address of master / slave somewhere in "modbus_define.h"
# Timer library compare mode ( 12-Jul-18)
TIM2 Configuration: Output Compare Timing Mode
  /* Compute the prescaler value */
PrescalerValue = (uint16_t) (SystemCoreClock / 12000000) - 1;????
I don't really understand what is PrescalerValue, TIM_Period, TIM_Prescaler
But it runs!
---------
||||||||
---------