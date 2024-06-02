#ifndef __SEND_DATA_TASK__
#define __SEND_DATA_TASK__

#define FRAME_HEADER_LENGTH 5 // ֡ͷ���ݳ���
#define CMD_ID_LENGTH 2       // ������ID���ݳ���
#define DATA_LENGTH 30        // ���ݶγ���
#define FRAME_TAIL_LENGTH 2   // ֡β���ݳ���

#define DATA_FRAME_LENGTH (FRAME_HEADER_LENGTH + CMD_ID_LENGTH + DATA_LENGTH + FRAME_TAIL_LENGTH) // ��������֡�ĳ���

#define CONTROLLER_CMD_ID 0x0302 // �Զ��������������

typedef __packed struct
{
    __packed struct
    {
        uint8_t sof;              // ��ʼ�ֽڣ��̶�ֵΪ0xA5
        uint16_t data_length;     // ����֡��data�ĳ���
        uint8_t seq;              // �����
        uint8_t crc8;             // ֡ͷCRC8У��
    } frame_header;               // ֡ͷ
    __packed uint16_t cmd_id;     // ������
    __packed uint8_t data[30];    // �Զ��������������֡
    __packed uint16_t frame_tail; // ֡βCRC16У��
} Controller_t;                   // �Զ�����������ݰ�



typedef struct {
    float w, x, y, z;
} Quaternion;

typedef struct {
    float x, y, z;
} Vector3;


extern fp32 received_angle[3];
#endif
