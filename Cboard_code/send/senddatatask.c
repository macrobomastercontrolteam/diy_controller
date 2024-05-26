#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "CRC8_CRC16.h"
#include "senddatatask.h"
#include "string.h"
#include "arm_math.h"
#include "INS_task.h"

/***********          ******************/

static void Data_Concatenation(uint8_t *data, uint16_t data_lenth);
void stable_check();

Controller_t tx_data; 
uint8_t data[DATA_LENGTH];
fp32 roll_deg;
fp32 pitch_deg;
fp32 yaw_deg;

fp32 acc_x;
fp32 acc_x_pre;

fp32 acc_y;
fp32 acc_z;

fp32 gyro_x;
fp32 gyro_y;
bool_t stable;

fp32 v_x;
fp32 v_x_pre;
fp32 v_y;
fp32 v_z;

fp32 d_x;
fp32 delta_x;
fp32 d_y;
fp32 d_z;

float dt;
float cutoff;
float filter;
float RC;

void StartSendDataTask(void const *argument)
{   
    // uint8_t index = 0;
    uint32_t wait_time = xTaskGetTickCount();
    uint8_t bCursor;
    for (;;)
    {   
        stable_check();
        //Process Time
        dt = 0.3f;

        bCursor = 0;
        roll_deg = INS_angle[INS_ROLL_ADDRESS_OFFSET]* 180.0f / PI;
        pitch_deg = INS_angle[INS_PITCH_ADDRESS_OFFSET]* 180.0f / PI;
        yaw_deg = INS_angle[INS_YAW_ADDRESS_OFFSET]* 180.0f / PI;
        

        acc_x_pre = acc_x;
        acc_x = INS_accel[INS_ACCEL_X_ADDRESS_OFFSET];

        cutoff = 0.5;
        RC = 1.0f / (2.0f *PI * cutoff);
        filter = 1.0f   / (1.0f + RC );

        //filtered
        acc_x = acc_x*filter + (1-filter)*acc_x_pre;

        acc_y = INS_accel[INS_ACCEL_Y_ADDRESS_OFFSET];
        acc_z = INS_accel[INS_ACCEL_Z_ADDRESS_OFFSET] - 9.81f;

        // acc_x = acc_y + sin(roll_deg) * cos(pitch_deg) * 9.81f;
        // acc_y = acc_z + cos(roll_deg) * cos(pitch_deg) * 9.81f;
        // acc_z = acc_x - sin(pitch_deg) * 9.81f ;


        v_x_pre = v_x;
        v_x = (acc_x + acc_x_pre)*dt / 2.0f;

        if (! stable) { 
            delta_x = (v_x + v_x_pre) * dt / 2.0f;}
        else {delta_x = 0;}


        d_x += delta_x;

        memcpy(&data[bCursor], &roll_deg, sizeof(roll_deg));
        bCursor += sizeof(roll_deg);
        memcpy(&data[bCursor], &pitch_deg, sizeof(pitch_deg));
        bCursor += sizeof(pitch_deg);
        memcpy(&data[bCursor], &yaw_deg, sizeof(yaw_deg));
        bCursor += sizeof(yaw_deg);

        memcpy(&data[bCursor], &d_x, sizeof(d_x));
        bCursor += sizeof(d_x);
        memcpy(&data[bCursor], &acc_y, sizeof(acc_y));
        bCursor += sizeof(acc_x);
        memcpy(&data[bCursor], &acc_z, sizeof(acc_z));
        bCursor += sizeof(acc_z);
        

        Data_Concatenation(data, DATA_LENGTH);
        HAL_UART_Transmit(&huart1, (uint8_t *)(&tx_data), sizeof(tx_data), 50);
        osDelayUntil(&wait_time, 35); //max delay time for custom controller
    }
}

/**
 * @brief           
 * @param data   锟捷段碉拷    指  
 * @param data_lenth   锟捷段筹拷  
 */
static void Data_Concatenation(uint8_t *data, uint16_t data_lenth)
{
    static uint8_t seq = 0;
    /// 帧头    
    tx_data.frame_header.sof = 0xA5;                              //     帧  始锟街节ｏ拷锟教讹拷值为 0xA5
    tx_data.frame_header.data_length = data_lenth;                //     帧    锟捷段的筹拷  
    tx_data.frame_header.seq = seq++;                             //     锟�
    append_CRC8_check_sum((uint8_t *)(&tx_data.frame_header), 5); //     帧头 CRC8 校  位
    ///       ID
    tx_data.cmd_id = CONTROLLER_CMD_ID;
    ///   锟捷讹拷
    memcpy(tx_data.data, data, data_lenth);
    /// 帧尾CRC16      校  
    append_CRC16_check_sum((uint8_t *)(&tx_data), DATA_FRAME_LENGTH);
}



void stable_check(){
    gyro_x = INS_gyro[INS_GYRO_X_ADDRESS_OFFSET];
    gyro_y = INS_gyro[INS_GYRO_Y_ADDRESS_OFFSET];

    float threashold;
    threashold = 0.2f;
    if(fabs(gyro_x) > threashold| fabs(gyro_y) > threashold){
        stable = 0;
    }else stable = 1;

}