/*
 * MAX30102.h
 *
 */

#ifndef APPLICATION_MAX30102_H_
#define APPLICATION_MAX30102_H_


struct _max30102_device
{
    uint32_t i2c_num;  //
    uint32_t inited;   //
    //uint32_t *ir_buffer;  
    //uint32_t *red_buffer; 
//    struct ringbuffer_s ir_ringbuffer;
//   struct ringbuffer_s red_ringbuffer;

    int32_t spo2;
    int32_t spo2_vaild;
    int32_t heart_rate;
    int32_t hr_vaild;

    int32_t temperature;
    int32_t read_num; 
};


//#define I2C_WRITE_ADDR 0xAE
//#define I2C_READ_ADDR 0xAF
extern uint8_t RingCount;  
extern uint8_t SetUpCount;
extern uint8_t ErrCount ;
extern uint32_t MaxDataBuff[2][2000];
extern int32_t Spo2Data;
extern int32_t HeartRate;
extern uint8_t HrErrCount;
#define I2C_MAX30102_ADDR 0x57
//register addresses   
#define REG_INTR_STATUS_1 0x00
#define REG_INTR_STATUS_2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA 0x07
#define REG_FIFO_CONFIG 0x08
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA 0x0C
#define REG_LED2_PA 0x0D
#define REG_PILOT_PA 0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR 0x1F
#define REG_TEMP_FRAC 0x20
#define REG_TEMP_CONFIG 0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID 0xFE
#define REG_PART_ID 0xFF

bool maxim_max30102_init(void);
bool maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led);
bool maxim_max30102_write_reg(uint8_t uch_addr, uint8_t uch_data);
bool maxim_max30102_read_reg(uint8_t uch_addr, uint8_t *puch_data);
bool maxim_max30102_reset(void);
bool maxim_max30102_wait_ready();
bool maxim_max30102_shutdown();


#if 0
extern int32_t maxim_max30102_read_temperature(void);
extern uint32_t MathSPO2Data(uint32_t dred,uint32_t dir);

void max30102_dev_start(void *notifycb);

void max30102_dev_ready_data();

void max30102_dev_data_prepare();

int32_t max30102_dev_spo2_get();

int32_t max30102_dev_hr_get();

void max30102_dev_power_save();

void max30102_dev_read_data();

extern void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid,
                                                   int32_t *pn_heart_rate, int8_t *pch_hr_valid);
#endif
#endif /* APPLICATION_MAX30102_H_ */
