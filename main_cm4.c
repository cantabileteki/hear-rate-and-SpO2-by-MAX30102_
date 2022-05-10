/* WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "cy_device_headers.h"
#include <stdlib.h>
#include "stdio.h"



uint32_t aun_ir_buffer[2000];    //ir data buffer
uint32_t aun_red_buffer[2000];    //red data buffer
int32_t n_ir_buffer_length;    //data buffer length 
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid


 //BLE：
void generic_EventHandler(uint32_t event, void *eventParameter)
{
    switch(event)
    {
        case CY_BLE_EVT_STACK_ON:
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
        {
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            break;
        }
        case CY_BLE_EVT_GATT_CONNECT_IND://
        {
            Cy_GPIO_Read(P5_1_PORT,P5_1_NUM);
            break;
        }
        case CY_BLE_EVT_GATTS_WRITE_CMD_REQ: 
    {
           cy_stc_ble_gatts_write_cmd_req_param_t *writeReqParameter = (cy_stc_ble_gatts_write_cmd_req_param_t  *)eventParameter;
    
     
              if(CY_BLE_DEVICE_INTERFACE_DEVICE_INBOUND_CHAR_HANDLE == writeReqParameter->handleValPair.attrHandle)
      {
                   //data = writeReqParameter->handleValPair.value.val[0];
                   //Cy_BLE_GATTS_WriteRsp(writeReqParameter->connHandle);
      }
    }
           break;
   }
    

}

void bleInterruptNotify()
{
    Cy_BLE_ProcessEvents();
}
void TransferStr( uint8* hstr,char* str)    //BLE 
{
    int k=0,i=0;
    while(str[k] != '\0'){
        hstr[i++] = str[k++];}
}

int main(void)
{
    
  
    __enable_irq(); /* Enable global interrupts. */
    /* Enable CM4.  CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */
    //Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR); 

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    //maxim_max30102_reset();
    
    int32_t temp;//temperature
    UART_Start();
    I2C_Start();
    Cy_BLE_Start(generic_EventHandler);          
    
    //printf("test 1\r\n");    test whether IIC START works
    maxim_max30102_init();  //initialize max
    
    
    
    n_ir_buffer_length=2000;
    
    while(Cy_BLE_GetState() != CY_BLE_STATE_ON)   
    {
        Cy_BLE_ProcessEvents();
    }
    Cy_BLE_RegisterAppHostCallback(bleInterruptNotify);
    
    for(int i=0;i<n_ir_buffer_length;i++)
    {
        
        /* Place your application code here. */
        maxim_max30102_read_fifo(&aun_red_buffer[i], &aun_ir_buffer[i]);  //max read fifo register
        
        
        printf("red=");
        printf("%i", aun_red_buffer[i]);
        printf("      ir=");
        printf("%i\n\r", aun_ir_buffer[i]);
        
        
    }
   
    
    printf("\n\n\nThe 2000th red= %i     ir= %i   \n\n",aun_red_buffer[1999],aun_ir_buffer[1999]);
    
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);//calculate hr and sp02
    printf("\n\n\nHR=%i,  ", n_heart_rate); 
    printf("HRvalid=%i,   ", ch_hr_valid);
    printf("SpO2=%i, ", n_sp02);
    printf("SPO2Valid=%i\n\r", ch_spo2_valid);
    /*temp=maxim_max30102_read_temperature();   max read temprature 
    printf("\nTemperature is %i\n",temp);*/
    
    /* Place your application code here. */
    static uint8 arra[4] = {0};              
    char indicat[8] = {};     
    cy_stc_ble_gatt_handle_value_pair_t serviceHandle;
    cy_stc_ble_gatt_value_t serviceData;
    TransferStr(arra,itoa(n_heart_rate, indicat, 8));
    
    serviceData.len = 5;
        
    serviceHandle.attrHandle=CY_BLE_DEVICE_INTERFACE_DEVICE_OUTBOUND_CHAR_HANDLE;
    serviceHandle.value=serviceData;
        
    Cy_BLE_GATTS_WriteAttributeValueLocal(&serviceHandle);
    printf("HR is %i",n_heart_rate);
        
     
}

/* [] END OF FILE */
