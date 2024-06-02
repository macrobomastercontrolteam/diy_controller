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
void Get_Data();
float low_pass_filter(float acc, float acc_pre, float coeff);
float Round_Function(float number);
float Complemenraty_Fielter(float acc, float gyro, float prev_comp, float dt);
float integrate_composite(float pre, float cur);

float magnitude (float x, float y, float z, float w);
void gravity();

void Quat_deg();

Controller_t tx_data; 
uint8_t data[DATA_LENGTH];

fp32 roll_deg,pitch_deg, yaw_deg;

fp32 filter_roll, filter_pitch, filter_yaw;

fp32 acc_x,acc_x_pre;
fp32 acc_y, acc_y_pre;
fp32 acc_z, acc_z_pre;

fp32 acc_x_f, acc_y_f, acc_z_f;

fp32 v_x, v_y, v_z;
fp32 v_x_pre, v_y_pre, v_z_pre;

fp32 d_x, d_y, d_z;
fp32 total_x, total_y, total_z;

fp32 mag_x, mag_y, mag_z;
fp32 gravity_x, gravity_y, gravity_z;
fp32 gyro_x , gyro_y, gyro_z;

bool_t stable_x, stable_y, stable_z;

fp32 quat_w, quat_x, quat_y, quat_z;

fp32 comp_pitch, comp_pitch_prev;
fp32 comp_row, comp_row_prev;


float dt = 0.034f;
float cutoff;
float filter;
float RC;


void StartSendDataTask(void const *argument)
{   
    // uint8_t index = 0;
    uint32_t wait_time = xTaskGetTickCount();
    uint8_t bCursor;

    //Process Time

    for (;;)
    {   
        //initialize
        bCursor = 0;
        
        Get_Data();
        stable_check();
        
        v_x_pre = v_x;
        v_x = integrate_composite(acc_x_pre, acc_x);
        d_x = integrate_composite(v_x_pre, v_x);

        v_y_pre = v_y;
        v_y = integrate_composite(acc_y_pre, acc_y);
        d_y = integrate_composite(v_y_pre, v_y);

        v_z_pre = v_z;
        v_z += integrate_composite(acc_z_pre, acc_z);
        d_z = integrate_composite(v_z_pre, v_z) * 5.0f;

        if( ! stable_x){
            total_x += d_x;
        }

        if( ! stable_y){
            total_y += d_y;
        }

        if( ! stable_z){
            total_z += d_z;
        }

        
        //Package data
        memcpy(&data[bCursor], &roll_deg, sizeof(roll_deg));
        bCursor += sizeof(roll_deg);
        memcpy(&data[bCursor], &pitch_deg, sizeof(pitch_deg));
        bCursor += sizeof(pitch_deg);
        memcpy(&data[bCursor], &yaw_deg, sizeof(yaw_deg));
        bCursor += sizeof(yaw_deg);

        memcpy(&data[bCursor], &acc_x, sizeof(acc_x));
        bCursor += sizeof(acc_x);
        memcpy(&data[bCursor], &acc_y, sizeof(acc_y));
        bCursor += sizeof(acc_x);
        memcpy(&data[bCursor], &acc_z, sizeof(acc_z));
        bCursor += sizeof(acc_z);
        //Send data
        Data_Concatenation(data, DATA_LENGTH);
        HAL_UART_Transmit(&huart1, (uint8_t *)(&tx_data), sizeof(tx_data), 50);
        osDelayUntil(&wait_time, 34); //max delay time for custom controller
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

float low_pass_filter(float acc, float acc_pre, float coeff){
    // float coeff = 0.8f;
    acc = coeff * acc + (1.0f - coeff) * acc_pre;

    return fabs(acc) <= 9.8f ? acc : 0;
}

void Get_Data(){
    // //Angle rad => degree
    roll_deg = INS_angle[INS_ROLL_ADDRESS_OFFSET]* 180.0f / PI;
    pitch_deg = INS_angle[INS_PITCH_ADDRESS_OFFSET]* 180.0f / PI;
    yaw_deg = INS_angle[INS_YAW_ADDRESS_OFFSET]* 180.0f / PI;
    // //rad -> for testing
    // roll_deg = INS_angle[INS_ROLL_ADDRESS_OFFSET];
    // pitch_deg = INS_angle[INS_PITCH_ADDRESS_OFFSET];
    // yaw_deg = INS_angle[INS_YAW_ADDRESS_OFFSET];

    filter_roll = Round_Function(roll_deg);
    filter_pitch = Round_Function(pitch_deg);
    filter_yaw = Round_Function(yaw_deg);

    //gyro angular velocity
    gyro_x = INS_gyro[INS_GYRO_X_ADDRESS_OFFSET];
    gyro_y = INS_gyro[INS_GYRO_Y_ADDRESS_OFFSET];
    gyro_z = INS_gyro[INS_GYRO_Z_ADDRESS_OFFSET];

    //Quant
    quat_w = INS_quat[0];
    quat_x = INS_quat[1];
    quat_y = INS_quat[2];
    quat_z = INS_quat[3];

    //Magnetometer
    mag_x = INS_mag[INS_MAG_X_ADDRESS_OFFSET];
    mag_y = INS_mag[INS_MAG_Y_ADDRESS_OFFSET];
    mag_z = INS_mag[INS_MAG_Z_ADDRESS_OFFSET];

    //raw acceleration
    acc_x_pre = acc_x;
    acc_y_pre = acc_y;
    acc_z_pre = acc_z;

    acc_x = INS_accel[INS_ACCEL_X_ADDRESS_OFFSET];
    acc_x = Round_Function(acc_x);
    acc_y = INS_accel[INS_ACCEL_Y_ADDRESS_OFFSET];
    acc_y = Round_Function(acc_y);
    acc_z = INS_accel[INS_ACCEL_Z_ADDRESS_OFFSET] - 9.81f;
    acc_z = Round_Function(acc_z);

    acc_x = low_pass_filter(acc_x, acc_x_pre, 0.6f);
    acc_y = low_pass_filter(acc_y, acc_y_pre, 0.6f);
    // acc_z = low_pass_filter(acc_z, acc_z_pre, 0.6f);

    gravity();

    acc_x = low_pass_filter(acc_x, acc_x_pre, 0.8f);
    acc_y = low_pass_filter(acc_y, acc_y_pre, 0.8f);
    acc_z = low_pass_filter(acc_z, acc_z_pre, 0.7f);

}


// void Quat_deg(){
//     quat_r = asinf(2.0f.0f * (quat_w*quat_y - quat_x * quat_z ) ) * 180.0f / PI ;
//     quat_p = atan2f( (quat_w * quat_z + quat_x * quat_y),  (1.0f - 2.0f.0f *(quat_y * quat_y + quat_z * quat_z)) )* 180.0f / PI;
//     quat_yaw = atan2f( (quat_w * quat_x + quat_z * quat_y), (1.0f - 2.0f.0f *(quat_y * quat_y + quat_x * quat_x)) ) * 180.0f / PI;
// }

//    comp_row =  Complemenraty_Fielter(acc_x, gyro_x,  comp_row_prev, dt);
//     comp_pitch =  Complemenraty_Fielter(acc_y, gyro_y, comp_pitch_prev, dt);
//     comp_pitch_prev = comp_pitch;
//     comp_row_prev = comp_row;
float Complemenraty_Fielter(float acc, float gyro, float prev_comp, float dt){
    float comp_filter;
    float alpha = 0.98f; // coeff;
    comp_filter = alpha * (prev_comp + gyro * dt) +  (1.0f-alpha) * acc;

    return comp_filter;
}

void stable_check(){

    //angle control to move

    float deg_threshold =  5.0f;

    // if(fabs(pitch_deg) > threshold| fabs(roll_deg) > threshold){
    //     stable = 0;
    // }else {
    //     stable = 1;
    // }

    float gyro_threshold =  0.02f;
    float acc_threshold =  0.2f;

    if(fabs(pitch_deg) > deg_threshold && fabs(acc_x) > acc_threshold){
        stable_x = 0;
    }else {
        stable_x= 1;
    }

    if(fabs(roll_deg) > deg_threshold && fabs(acc_y) > acc_threshold ){
        stable_y = 0;
    }else {
        stable_y= 1;
    }

    if(fabs(gyro_z) > 0.01f){
        stable_z = 0;
    }else {
        stable_z= 1;
    }
}

float integrate_composite(float pre, float cur){
    float result;
    result = (pre + cur) / 2.0f * dt;
    //if stable, return current value//
    //otherwise the controller is moving
    return result * 20.0f;
}

float Round_Function(float number) {
    return roundf(number * 10.0f) / 10.0f;
}

float magnitude (float x, float y, float z, float w){
    return sqrtf(x*x + y*y + z*z + w*w);
}

void gravity(){

    Vector3 acc = {acc_x, acc_y, acc_z};
     Vector3 gravity = {0, 0, 9.8};

    float q0 = quat_w, q1 = quat_x, q2 = quat_y, q3 = quat_z;
    float R[3][3] = {
    {1.0f - 2.0f*q2*q2 - 2.0f*q3*q3, 2.0f*q1*q2 - 2.0f*q3*q0, 2.0f*q1*q3 + 2.0f*q2*q0},
    {2.0f*q1*q2 + 2.0f*q3*q0, 1.0f - 2.0f*q1*q1 - 2.0f*q3*q3, 2.0f*q2*q3 - 2.0f*q1*q0},
    {2.0f*q1*q3 - 2.0f*q2*q0, 2.0f*q2*q3 + 2.0f*q1*q0, 1.0f - 2.0f*q1*q1 - 2.0f*q2*q2}
    };

    // Rotate the acceleration vector from sensor frame to Earth frame
    Vector3 acc_earth = {
        R[0][0]*acc.x + R[0][1]*acc.y + R[0][2]*acc.z,
        R[1][0]*acc.x + R[1][1]*acc.y + R[1][2]*acc.z,
        R[2][0]*acc.x + R[2][1]*acc.y + R[2][2]*acc.z
    };

    acc_x = acc_earth.x- gravity.x;
    acc_y = acc_earth.y - gravity.y;
    // acc_z = acc_earth.z - gravity.z;
    // acc_z = acc_z - gravity.z;

}

