#ifndef TEST_CAN_H
#define TEST_CAN_H
#include <stdint.h>
typedef struct {int index;} CAN_HandleTypeDef;
typedef struct {uint32_t StdId;} CAN_TxHeaderTypeDef;
typedef struct CANInstance {
    CAN_HandleTypeDef *can_handle;CAN_TxHeaderTypeDef txconf;
    uint32_t tx_id;uint8_t tx_buff[8],rx_buff[8];uint32_t rx_id;uint8_t rx_len;
    void(*can_module_callback)(struct CANInstance*);void *id;
} CANInstance;
typedef struct {CAN_HandleTypeDef*can_handle;uint32_t tx_id,rx_id;void(*can_module_callback)(CANInstance*);void*id;} CAN_Init_Config_s;
CANInstance*CANRegister(CAN_Init_Config_s*config);
uint8_t CANTransmit(CANInstance*c,float timeout);
#endif
