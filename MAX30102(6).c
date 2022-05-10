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

#if 0

void maxim_sort_ascend(int32_t  *pn_x, int32_t n_size)
/**
* \brief        Sort array
* \par          Details
*               Sort array in ascending order (insertion sort algorithm)
*
* \retval       None
*/
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++)
    {
        n_temp = pn_x[i];
        for (j = i; j > 0 && n_temp < pn_x[j - 1]; j--)
            pn_x[j] = pn_x[j - 1];
        pn_x[j] = n_temp;
    }
}
void maxim_peaks_above_min_height( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height )
/**
* \brief        Find peaks above n_min_height
* \par          Details
*               Find all peaks above MIN_HEIGHT
*
* \retval       None
*/
{
    int32_t i = 1, n_width, riseFound = 0, holdOff1 = 0, holdOff2 = 0, holdOffThresh = 4;
    *n_npks = 0;

    while (i < n_size - 1)
    {
        if (holdOff2 == 0)
        {
            if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i - 1])     // find left edge of potential peaks
            {
                riseFound = 1;
            }
            if (riseFound == 1)
            {
                if ((pn_x[i] < n_min_height) && (holdOff1 < holdOffThresh))     // if false edge
                {
                    riseFound = 0;
                    holdOff1 = 0;
                }
                else
                {
                    if (holdOff1 == holdOffThresh)
                    {
                        if ((pn_x[i] < n_min_height) && (pn_x[i - 1] >= n_min_height))
                        {
                            if ((*n_npks) < 15 )
                            {
                                pn_locs[(*n_npks)++] = i;   // peak is right edge
                            }
                            holdOff1 = 0;
                            riseFound = 0;
                            holdOff2 = 8;
                        }
                    }
                    else
                    {
                        holdOff1 = holdOff1 + 1;
                    }
                }
            }
        }
        else
        {
            holdOff2 = holdOff2 - 1;
        }
        i++;
    }
}

void maxim_sort_indices_descend(  int32_t  *pn_x, int32_t *pn_indx, int32_t n_size)
/**
* \brief        Sort indices
* \par          Details
*               Sort indices according to descending order (insertion sort algorithm)
*
* \retval       None
*/
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++)
    {
        n_temp = pn_indx[i];
        for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j - 1]]; j--)
            pn_indx[j] = pn_indx[j - 1];
        pn_indx[j] = n_temp;
    }
}


void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance)
/**
* \brief        Remove peaks
* \par          Details
*               Remove peaks separated by less than MIN_DISTANCE
*
* \retval       None
*/
{

    int32_t i, j, n_old_npks, n_dist;

    /* Order peaks from large to small */
    maxim_sort_indices_descend( pn_x, pn_locs, *pn_npks );

    for ( i = -1; i < *pn_npks; i++ )
    {
        n_old_npks = *pn_npks;
        *pn_npks = i + 1;
        for ( j = i + 1; j < n_old_npks; j++ )
        {
            n_dist =  pn_locs[j] - ( i == -1 ? -1 : pn_locs[i] ); // lag-zero peak of autocorr is at index -1
            if ( n_dist > n_min_distance || n_dist < -n_min_distance )
                pn_locs[(*pn_npks)++] = pn_locs[j];
        }
    }

    // Resort indices int32_to ascending order
    maxim_sort_ascend( pn_locs, *pn_npks );
}


void maxim_find_peaks( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num )
/**
* \brief        Find peaks
* \par          Details
*               Find at most MAX_NUM peaks above MIN_HEIGHT separated by at least MIN_DISTANCE
*
* \retval       None
*/
{
    maxim_peaks_above_min_height( pn_locs, n_npks, pn_x, n_size, n_min_height );
    maxim_remove_close_peaks( pn_locs, n_npks, pn_x, n_min_distance );
    *n_npks = min( *n_npks, n_max_num );
}

void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid,
        int32_t *pn_heart_rate, int8_t *pch_hr_valid)
/**
* \brief        Calculate the heart rate and SpO2 level
* \par          Details
*               By detecting  peaks of PPG cycle and corresponding AC/DC of red/infra-red signal, the an_ratio for the SPO2 is computed.
*               Since this algorithm is aiming for Arm M0/M3. formaula for SPO2 did not achieve the accuracy due to register overflow.
*               Thus, accurate SPO2 is precalculated and save longo uch_spo2_table[] per each an_ratio.
*
* \param[in]    *pun_ir_buffer           - IR sensor data buffer
* \param[in]    n_ir_buffer_length      - IR sensor data buffer length
* \param[in]    *pun_red_buffer          - Red sensor data buffer
* \param[out]    *pn_spo2                - Calculated SpO2 value
* \param[out]    *pch_spo2_valid         - 1 if the calculated SpO2 value is valid
* \param[out]    *pn_heart_rate          - Calculated heart rate value
* \param[out]    *pch_hr_valid           - 1 if the calculated heart rate value is valid
*
* \retval       None
*/
{
    uint32_t un_ir_mean, un_only_once ;
    int32_t k, n_i_ratio_count;
    int32_t i, s, m, n_exact_ir_valley_locs_count, n_middle_idx;
    int32_t n_th1, n_npks, n_c_min;
    int32_t an_ir_valley_locs[15] ;
    int32_t n_peak_interval_sum;

    int32_t n_y_ac, n_x_ac;
//    int32_t n_spo2_calc;
    float n_spo2_calc;
    int32_t n_y_dc_max, n_x_dc_max;
    int32_t n_y_dc_max_idx, n_x_dc_max_idx;
    int32_t an_ratio[5], n_ratio_average;
    int32_t n_nume, n_denom ;

    // calculates DC mean and subtract DC from ir
    un_ir_mean = 0;
    for (k = 0 ; k < n_ir_buffer_length ; k++ ) un_ir_mean += pun_ir_buffer[k] ;
    un_ir_mean = un_ir_mean / n_ir_buffer_length ;

    // remove DC and invert signal so that we can use peak detector as valley detector
    for (k = 0 ; k < n_ir_buffer_length ; k++ )
        an_x[k] = -1 * (pun_ir_buffer[k] - un_ir_mean) ;

    // 4 pt Moving Average
    for(k = 0; k < BUFFER_SIZE - MA4_SIZE; k++)
    {
        an_x[k] = ( an_x[k] + an_x[k + 1] + an_x[k + 2] + an_x[k + 3]) / (int)4;
    }
    // calculate threshold
    n_th1 = 0;
    for ( k = 0 ; k < BUFFER_SIZE ; k++)
    {
        n_th1 +=  an_x[k];
    }
    n_th1 =  n_th1 / ( BUFFER_SIZE);
    if( n_th1 < 30) n_th1 = 30; // min allowed
    if( n_th1 > 60) n_th1 = 60; // max allowed

    for ( k = 0 ; k < 15; k++) an_ir_valley_locs[k] = 0;
    // since we flipped signal, we use peak detector as vSalley detector
    maxim_find_peaks( an_ir_valley_locs, &n_npks, an_x, BUFFER_SIZE, n_th1, 4, 15 );//peak_height, peak_distance, max_num_peaks
    n_peak_interval_sum = 0;
    if (n_npks >= 2)
    {
        for (k = 1; k < n_npks; k++) n_peak_interval_sum += (an_ir_valley_locs[k] - an_ir_valley_locs[k - 1] ) ;
        n_peak_interval_sum = n_peak_interval_sum / (n_npks - 1);
        *pn_heart_rate = (int32_t)( (FS * 60) / n_peak_interval_sum );
        *pch_hr_valid  = 1;
    }
    else
    {
        *pn_heart_rate = -999; // unable to calculate because # of peaks are too small
        *pch_hr_valid  = 0;
    }

    //  load raw value again for SPO2 calculation : RED(=y) and IR(=X)
    for (k = 0 ; k < n_ir_buffer_length ; k++ )
    {
        an_x[k] =  pun_ir_buffer[k] ;
        an_y[k] =  pun_red_buffer[k] ;
    }

    // find precise min near an_ir_valley_locs
    n_exact_ir_valley_locs_count = n_npks;

    //using exact_ir_valley_locs , find ir-red DC andir-red AC for SPO2 calibration an_ratio
    //finding AC/DC maximum of raw

    n_ratio_average = 0;
    n_i_ratio_count = 0;
    for(k = 0; k < 5; k++) an_ratio[k] = 0;
    for (k = 0; k < n_exact_ir_valley_locs_count; k++)
    {
        if (an_ir_valley_locs[k] > BUFFER_SIZE )
        {
            *pn_spo2 =  -999 ; // do not use SPO2 since valley loc is out of range
            *pch_spo2_valid  = 0;
            return;
        }
    }
    // find max between two valley locations
    // and use an_ratio betwen AC compoent of Ir & Red and DC compoent of Ir & Red for SPO2
    for (k = 0; k < n_exact_ir_valley_locs_count - 1; k++)
    {
        n_y_dc_max = -16777216 ;
        n_x_dc_max = -16777216;
        if (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k] > 3)
        {
            for (i = an_ir_valley_locs[k]; i < an_ir_valley_locs[k + 1]; i++)
            {
                if (an_x[i] > n_x_dc_max)
                {
                    n_x_dc_max = an_x[i];
                    n_x_dc_max_idx = i;
                }
                if (an_y[i] > n_y_dc_max)
                {
                    n_y_dc_max = an_y[i];
                    n_y_dc_max_idx = i;
                }
            }
            n_y_ac = (an_y[an_ir_valley_locs[k + 1]] - an_y[an_ir_valley_locs[k] ] ) * (n_y_dc_max_idx - an_ir_valley_locs[k]); //red
            n_y_ac =  an_y[an_ir_valley_locs[k]] + n_y_ac / (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k])  ;
            n_y_ac =  an_y[n_y_dc_max_idx] - n_y_ac;   // subracting linear DC compoenents from raw
            n_x_ac = (an_x[an_ir_valley_locs[k + 1]] - an_x[an_ir_valley_locs[k] ] ) * (n_x_dc_max_idx - an_ir_valley_locs[k]); // ir
            n_x_ac =  an_x[an_ir_valley_locs[k]] + n_x_ac / (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k]);
            n_x_ac =  an_x[n_y_dc_max_idx] - n_x_ac;     // subracting linear DC compoenents from raw
            n_nume = ( n_y_ac * n_x_dc_max) >> 7 ; //prepare X100 to preserve floating value
            n_denom = ( n_x_ac * n_y_dc_max) >> 7;
            if (n_denom > 0  && n_i_ratio_count < 5 &&  n_nume != 0)
            {
                an_ratio[n_i_ratio_count] = (n_nume * 100) / n_denom ; //formular is ( n_y_ac *n_x_dc_max) / ( n_x_ac *n_y_dc_max) ;
                n_i_ratio_count++;
            }
        }
    }
    // choose median value since PPG signal may varies from beat to beat
    maxim_sort_ascend(an_ratio, n_i_ratio_count);
    n_middle_idx = n_i_ratio_count / 2;

    if (n_middle_idx > 1)
        n_ratio_average = ( an_ratio[n_middle_idx - 1] + an_ratio[n_middle_idx]) / 2; // use median
    else
        n_ratio_average = an_ratio[n_middle_idx ];

    if( n_ratio_average > 2 && n_ratio_average < 184)
    {
     //   n_spo2_calc = uch_spo2_table[n_ratio_average] ;
        n_spo2_calc=(float)(-45.060*n_ratio_average* n_ratio_average/10000.0 + 30.354 *n_ratio_average/100.0 + 94.845);
        *pn_spo2 = (int32_t)(n_spo2_calc*10) ;
        *pch_spo2_valid  = 1;//  float_SPO2 =  -45.060*n_ratio_average* n_ratio_average/10000 + 30.354 *n_ratio_average/100 + 94.845 ;  // for comparison with table
    }
    else
    {
        *pn_spo2 =  -999 ; // do not use SPO2 since signal an_ratio is out of range
        *pch_spo2_valid  = 0;
    }
}


/*
 * 
 * 
 */
void max30102_dev_start(void *notifycb)
{
 uint8_t uch_dummy=0xff;
 if(max30102_dev.inited==0)
   {
 //    maxim_max30102_i2c_init(notifycb);
 //    maxim_max30102_pin_init();
 //    ringbuffer_init(&max30102_dev.ir_ringbuffer,(uint8_t*)g_ir_pool,sizeof(g_ir_pool));
 //    ringbuffer_init(&max30102_dev.red_ringbuffer,(uint8_t*)g_red_pool,sizeof(g_red_pool));
     max30102_dev.inited=1;

#if 0
     UART_init();

        // Create a UART with data processing off.
     UART_Params_init(&uartParams);
     uartParams.writeDataMode = UART_DATA_BINARY;
     uartParams.readDataMode = UART_DATA_BINARY;
     uartParams.readReturnMode = UART_RETURN_FULL;
     uartParams.readEcho = UART_ECHO_OFF;
     uartParams.baudRate = 115200;

     // Open an instance of the UART drivers
     uart = UART_open(Board_UART0, &uartParams);
     if (uart == NULL)
         {
         while(1);
         }
#endif
   }

 maxim_max30102_reset();
 maxim_max30102_init();
 maxim_max30102_read_reg(0,&uch_dummy);
 //memset(max30102_dev.ir_buffer,0,sizeof(uint32_t)*FS*4);
 //memset(max30102_dev.red_buffer,0,sizeof(uint32_t)*FS*4);


 max30102_dev.heart_rate=0;
 max30102_dev.hr_vaild=0;
 max30102_dev.spo2=0;
 max30102_dev.spo2_vaild=0;
 max30102_dev.read_num=0;

// ringbuffer_reset(&max30102_dev.ir_ringbuffer);
// ringbuffer_reset(&max30102_dev.red_ringbuffer);
}


/*
 * 
 */
void max30102_dev_read_data()
{
    uint32_t ir=0;
    uint32_t red=0;
    if(!maxim_max30102_read_fifo(&red,&ir))
    {
        ir=0x3FFFFF;
        red=0x3FFFFF;
    }

  /*  ir|=(max30102_dev.read_num<<24);
    red|=(max30102_dev.read_num<<24);
    //UART_write(uart,&ir,4);
    //UART_write(uart,&red,4);
    if(ringbuffer_put(&max30102_dev.ir_ringbuffer,(uint8_t*)&ir,4)==0)
    {
        ir|=(max30102_dev.read_num<<24);
    }
    ringbuffer_put(&max30102_dev.red_ringbuffer,(uint8_t*)&red,4);// 
    max30102_dev.read_num++;
    if(max30102_dev.read_num==128)
        max30102_dev.read_num=0;*/
}


uint32_t MathSPO2Data(uint32_t dred,uint32_t dir)
{
    float R = 0;
    float DSPO2 = 0;
    R = (dred*1.0)/(dir*1.0);
    DSPO2 = (0.1484-0.018*R)*100/(0.133+0.01*R);
//    DSPO2 = 30.354*R-45.06*R*R+94.845;
    return (uint32_t)DSPO2;
}


/*
 * 
 */
void max30102_dev_power_save()
{
    maxim_max30102_shutdown();
}


uint32_t max30102_ir_wait_ready()
{
  //  return ringbuffer_data_len(&max30102_dev.ir_ringbuffer);
}

uint32_t max30102_red_wait_ready()
{
  //  return ringbuffer_data_len(&max30102_dev.red_ringbuffer);
}

/*
 * 
 */
void max30102_ir_read(uint8_t *ir,uint32_t count)
{
 //  ringbuffer_get(&max30102_dev.ir_ringbuffer,ir,count);
}

void max30102_red_read(uint8_t *red,uint32_t count)
{
 //   ringbuffer_get(&max30102_dev.red_ringbuffer,red,count);
}
#endif

#if 0
/*
 */
void max30102_dev_ready_data()
{
    uint32_t blen=FS*4;
    uint32_t i=0;

    for(i=0;i<blen;i++)
    {
        if(maxim_max30102_wait_ready())
            maxim_max30102_read_fifo(&max30102_dev.red_buffer[i],&max30102_dev.ir_buffer[i]);
    }

  // maxim_heart_rate_and_oxygen_saturation(max30102_dev.ir_buffer, blen, max30102_dev.red_buffer, &max30102_dev.spo2,
 //                                          (int8_t*)&max30102_dev.spo2_vaild, &max30102_dev.heart_rate, (int8_t*)&max30102_dev.hr_vaild);
}

/*
 */
void max30102_dev_data_prepare()
{
  uint32_t blen=FS*4;
  uint32_t i=0;

  for(i=100;i<blen;i++)
  {
      max30102_dev.red_buffer[i-100]=max30102_dev.red_buffer[i];
      max30102_dev.ir_buffer[i-100]=max30102_dev.ir_buffer[i];
  }

  for(i=300;i<400;i++)
  {
    if(maxim_max30102_wait_ready())
       maxim_max30102_read_fifo(&max30102_dev.red_buffer[i],&max30102_dev.ir_buffer[i]);
  }

 // maxim_heart_rate_and_oxygen_saturation(max30102_dev.ir_buffer, blen, max30102_dev.red_buffer, &max30102_dev.spo2,
 //                                            (int8_t *)&max30102_dev.spo2_vaild, &max30102_dev.heart_rate, (int8_t *)&max30102_dev.hr_vaild);
}

/*
 */
int32_t max30102_dev_spo2_get()
{
    if(max30102_dev.spo2_vaild)
        return max30102_dev.spo2;
    return 0;
}




#endif
