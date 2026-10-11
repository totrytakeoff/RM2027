set pagination off
set confirm off
target remote :3333

monitor reset halt
thbreak HAL_UARTEx_ReceiveToIdle_DMA
continue
return (HAL_StatusTypeDef)1
detach
