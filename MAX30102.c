/*
 * MAX30102.c
 */
#include <string.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "I2C.h"

#include "cy_scb_i2c.h"
#include "MAX30102(6).h"



#define POOL_SIZE (2012)
#define BUFFER_SIZE  2000   //
#define MA4_SIZE  4
#define min(x,y) ((x) < (y) ? (x) : (y))
#define FS 100
//static uint32_t g_ir_pool[POOL_SIZE];
//static uint32_t g_red_pool[POOL_SIZE];

//uch_spo2_table is approximated as  -45.060*ratioAverage* ratioAverage + 30.354 *ratioAverage + 94.845 ;
const uint8_t uch_spo2_table[184] = { 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
                                      99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                                      100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
                                      97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
                                      90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
                                      80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
                                      66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
                                      49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
                                      28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
                                      3, 2, 1
                                    } ;
static  int32_t an_x[ BUFFER_SIZE]; //ir
static  int32_t an_y[ BUFFER_SIZE]; //red

static struct _max30102_device max30102_dev;

uint32_t MaxDataBuff[2][BUFFER_SIZE];
uint8_t RingCount = 0;
uint8_t SetUpCount = 0;
uint8_t ErrCount = 0 ;
uint8_t HrErrCount = 0;
int32_t Spo2Data = 0;   
int32_t HeartRate = 0;

bool maxim_max30102_wait_ready()
{
    //Semaphore_pend(_max301020_waitsemHandle, BIOS_WAIT_FOREVER);
    //while(PIN_getInputValue(MAX30102_INT_PIN)==1);
    return true;
}

static uint8_t PSOC_IIC_Write(uint8_t nPSOC_SlaveAddr, uint8_t *pPSOC_Data, uint8_t nPSOC_Len)
{
    int rval = 0;
    cy_stc_scb_i2c_master_xfer_config_t transfer;
    /* Configure write transaction */
    transfer.slaveAddress = nPSOC_SlaveAddr;
    transfer.buffer       = pPSOC_Data;
    transfer.bufferSize   = nPSOC_Len;
    transfer.xferPending  = true; /* Do not generate Stop condition at the end of transaction */
    /* Initiate write transaction.
    * The Start condition is generated to begin this transaction.
    */
    rval =  Cy_SCB_I2C_MasterWrite(I2C_HW, &transfer, &I2C_context);
    /* Wait for transaction completion */
    // while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(I2C_HW, &I2C_context)))
    // {
    // }
    printf("PSOC_IIC_Write %d \r\n",rval);  
    //printf("PSOC_IIC_Write state %d , ",rval)
    /* Process received data */
    return 0;
}

static uint8_t PSOC_IIC_Read(uint8_t nPSOC_SlaveAddr, uint8_t *pPSOC_Data, uint8_t nPSOC_Len)
{
    int rval = 0;

    cy_stc_scb_i2c_master_xfer_config_t transfer;
    /* Configure read transaction */
    transfer.slaveAddress = nPSOC_SlaveAddr;
    transfer.buffer       = pPSOC_Data;
    transfer.bufferSize   = nPSOC_Len;
    transfer.xferPending  = false; /* Generate Stop condition the end of transaction */
    /* Initiate read transaction.
    * The ReStart condition is generated to begin this transaction because
    * previous transaction was completed without Stop.
    */
    rval =  Cy_SCB_I2C_MasterRead(I2C_HW, &transfer, &I2C_context);
    /* Wait for transaction completion */
    // while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(I2C_HW, &I2C_context)))
    // {
    // }
    printf("PSOC_IIC_Read %d \r\n",rval);     //
    //printf("PSOC_IIC_Read state %d ,   ",rval);
    /* Process received data */

    return 0;
}

uint8_t PSOC_IIC_WriteOneByte(uint8_t nPSOC_SlaveAddr, uint8_t nPSOC_Cmd, uint8_t nPSOC_Data)
{
    uint8_t aPSOC_Data[2] = {0};

    aPSOC_Data[0] = nPSOC_Cmd;
    aPSOC_Data[1] = nPSOC_Data;

    if(PSOC_IIC_Write(nPSOC_SlaveAddr, aPSOC_Data, 2) == -1)
    {
        return -1;
    }

    return 0;
}

uint8_t PSOC_IIC_ReadOneByte(uint8_t nPSOC_SlaveAddr, uint8_t nPSOC_Cmd, uint8_t *pPSOC_Data)
{
    if(PSOC_IIC_Write(nPSOC_SlaveAddr, &nPSOC_Cmd, 1) == -1)
    {
        return -1;             //
    }

    if(PSOC_IIC_Read(nPSOC_SlaveAddr, pPSOC_Data, 1) == -1)
    {
        return -1;
    }

    return 0;
}

bool maxim_max30102_write_reg(uint8_t uch_addr, uint8_t uch_data)
/**
* \brief        Write a value to a MAX30102 register
* \par          Details
*               This function writes a value to a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[in]    uch_data    - register data
*
* \retval       true on success
*/
{
    return PSOC_IIC_WriteOneByte(I2C_MAX30102_ADDR,uch_addr,uch_data);
}

bool maxim_max30102_read_reg(uint8_t uch_addr, uint8_t *puch_data)
/**
* \brief        Read a MAX30102 register
* \par          Details
*               This function reads a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[out]   puch_data    - pointer that stores the register data
*
* \retval       true on success
*/
{
    return PSOC_IIC_ReadOneByte(I2C_MAX30102_ADDR,uch_addr,puch_data);
}

uint8_t PSOC_IIC_ReadBlock(uint8_t nPSOC_SlaveAddr, uint8_t nPSOC_Cmd, uint8_t *pPSOC_Data, uint8_t nPSOCLen)
{
    uint8_t nPSOC_H;
    uint8_t nPSOC_L;

    if(PSOC_IIC_Write(nPSOC_SlaveAddr, &nPSOC_Cmd, 1) == -1)
    {
        return -1;
    }

    if(PSOC_IIC_Read(nPSOC_SlaveAddr, pPSOC_Data, nPSOCLen) == -1)
    {
        return -1;
    }

    //order as highest to lowest
    nPSOC_L = pPSOC_Data[0];
    nPSOC_H = pPSOC_Data[1];
    pPSOC_Data[0] = nPSOC_H;
    pPSOC_Data[1] = nPSOC_L;

    return 0;
}

bool maxim_max30102_init(void)
/**
* \brief        Initialize the MAX30102
* \par          Details
*               This function initializes the MAX30102
*
* \param        None
*
* \retval       true on success
*/
{
    uint8_t uch_temp;
    if(maxim_max30102_write_reg(REG_INTR_ENABLE_1, 0xc0)) // INTR setting  
        return false;
    if(maxim_max30102_write_reg(REG_INTR_ENABLE_2, 0x02))  //
        return false;
    if(maxim_max30102_write_reg(REG_FIFO_WR_PTR, 0x00)) //FIFO_WR_PTR[4:0]
        return false;
    if(maxim_max30102_write_reg(REG_OVF_COUNTER, 0x00)) //OVF_COUNTER[4:0]
        return false;
    if(maxim_max30102_write_reg(REG_FIFO_RD_PTR, 0x00)) //FIFO_RD_PTR[4:0]
        return false;
    if(maxim_max30102_write_reg(REG_FIFO_CONFIG, 0x0f)) //0 avg=4=[0x4f]  sample avg = 8[=0x6f], fifo rollover=false, fifo almost full = 17
        return false;//edited avg=1
    if(maxim_max30102_write_reg(REG_MODE_CONFIG, 0x03))  //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
        return false;
    if(maxim_max30102_write_reg(REG_SPO2_CONFIG, 0x27)) // 0x2f SPO2_ADC range = 4096nA, SPO2 sample rate (50 Hz), LED pulseWidth (411uS)
        return false;//edited 4096na 100hz [0x47 3.24]

    if(maxim_max30102_write_reg(REG_LED1_PA, 0x1f))  //Choose value for ~ 4.5mA for LED1 //0.5 ma
        return false;
    if(maxim_max30102_write_reg(REG_LED2_PA, 0x1f))  // Choose value for ~ 4.5mA for LED2//0.5 ma
        return false;
//   if(!maxim_max30102_write_reg(REG_PILOT_PA, 0x7f))  // Choose value for ~ 25mA for Pilot LED
//        return false;
    if(maxim_max30102_write_reg(REG_TEMP_CONFIG, 0x00))  // temperature enable 0x01
           return false;
    maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp);
    maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp);
    return true;
}

int32_t maxim_max30102_read_temperature()
{
    int8_t temp=0;
    maxim_max30102_read_reg(REG_TEMP_INTR,(uint8_t*)&temp);
    maxim_max30102_write_reg(REG_TEMP_CONFIG, 0x1);
    return temp;
}

bool maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)

/**
* \brief        Read a set of samples from the MAX30102 FIFO register
* \par          Details
*               This function reads a set of samples from the MAX30102 FIFO register
*
* \param[out]   *pun_red_led   - pointer that stores the red LED reading data
* \param[out]   *pun_ir_led    - pointer that stores the IR LED reading data
*
* \retval       true on success
*/
{
    uint32_t un_temp[6]={0,0,0,0,0,0};
    uint8_t *p_temp=(uint8_t*)un_temp;
    uint8_t uch_temp;
 //   uint8_t num = 0;
 //   uint8_t i;
 //   uint8_t reg=0;
//    I2C_Transaction  trans;

    *pun_ir_led = 0;
    *pun_red_led = 0;

/*    maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp);
    maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp);
    maxim_max30102_read_reg(REG_OVF_COUNTER, &uch_temp);
    if(uch_temp!=0)
    {
        return false;
    }*/

//    if(!maxim_max30102_read_reg(I2C_MAX30102_ADDR,REG_FIFO_WR_PTR,&uch_temp))  return false;

 //   if(!maxim_max30102_read_reg(I2C_MAX30102_ADDR,REG_FIFO_RD_PTR,&uch_temp1))  return false;


    if(PSOC_IIC_ReadBlock(I2C_MAX30102_ADDR,REG_FIFO_DATA,p_temp,6))  return false;

    *pun_red_led=((p_temp[0]<<16)|(p_temp[1]<<8)|(p_temp[2]))&0x03FFFF;
    *pun_ir_led=((p_temp[3]<<16)|(p_temp[4]<<8)|(p_temp[5]))&0x03FFFF;


    return true;
}

bool maxim_max30102_reset()
/**
* \brief        Reset the MAX30102
* \par          Details
*               This function resets the MAX30102
*
* \param        None
*
* \retval       true on success
*/
{
    if(!maxim_max30102_write_reg(REG_MODE_CONFIG, 0x40))
        return false;
    else
        return true;
}

/*
 * 
 */
bool maxim_max30102_shutdown()
{
    if(!maxim_max30102_write_reg(REG_MODE_CONFIG, 0x80))
           return false;
       else
           return true;
}

