

#ifndef GO1_H
#define GO1_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "Serial_device.h"
#include "pid.h"
#include "motor.h"
#include <math.h>
#include "TaskManager.h"
#include "crc_util.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#define SATURATE(_IN, _MIN, _MAX) \
    {                             \
        if (_IN < _MIN)           \
            _IN = _MIN;           \
        else if (_IN > _MAX)      \
            _IN = _MAX;           \
    }
typedef struct
{
    uint8_t id : 4;     // µç»úID: 0,1...,13,14 15±íÊ¾ÏòËùÓÐµç»ú¹ã²¥Êý¾Ý(´ËÊ±ÎÞ·µ»Ø)
    uint8_t status : 3; // ¹¤×÷Ä£Ê½: 0.Ëø¶¨ 1.FOC±Õ»· 2.±àÂëÆ÷Ð£×¼ 3.±£Áô
    uint8_t none : 1;   // ±£ÁôÎ»
} RIS_Mode_t;           // ¿ØÖÆÄ£Ê½ 1Byte

/**
 * @brief µç»ú×´Ì¬¿ØÖÆÐÅÏ¢
 *
 */
typedef struct
{
    int16_t tor_des; // ÆÚÍû¹Ø½ÚÊä³öÅ¤¾Ø unit: N.m      (q8)
    int16_t spd_des; // ÆÚÍû¹Ø½ÚÊä³öËÙ¶È unit: rad/s    (q8)
    int32_t pos_des; // ÆÚÍû¹Ø½ÚÊä³öÎ»ÖÃ unit: rad      (q15)
    int16_t k_pos;   // ÆÚÍû¹Ø½Ú¸Õ¶ÈÏµÊý unit: -1.0-1.0 (q15)
    int16_t k_spd;   // ÆÚÍû¹Ø½Ú×èÄáÏµÊý unit: -1.0-1.0 (q15)

} RIS_Comd_t; // ¿ØÖÆ²ÎÊý 12Byte

/**
 * @brief µç»ú×´Ì¬·´À¡ÐÅÏ¢
 *
 */
typedef struct
{
    int16_t torque;      // Êµ¼Ê¹Ø½ÚÊä³öÅ¤¾Ø unit: N.m     (q8)
    int16_t speed;       // Êµ¼Ê¹Ø½ÚÊä³öËÙ¶È unit: rad/s   (q8)
    int32_t pos;         // Êµ¼Ê¹Ø½ÚÊä³öÎ»ÖÃ unit: rad     (q15)
    int8_t temp;         // µç»úÎÂ¶È: -128~127¡ãC
    uint8_t MError : 3;  // µç»ú´íÎó±êÊ¶: 0.Õý³£ 1.¹ýÈÈ 2.¹ýÁ÷ 3.¹ýÑ¹ 4.±àÂëÆ÷¹ÊÕÏ 5-7.±£Áô
    uint16_t force : 12; // ×ã¶ËÆøÑ¹´«¸ÐÆ÷Êý¾Ý 12bit (0-4095)
    uint8_t none : 1;    // ±£ÁôÎ»
} RIS_Fbk_t;             // ×´Ì¬Êý¾Ý 11Byte
typedef union
{
    int32_t L;
    uint8_t u8[4];
    uint16_t u16[2];
    uint32_t u32;
    float F;
} COMData32;

typedef struct
{
    // ¶¨Òå Êý¾Ý°üÍ·
    unsigned char start[2]; // °üÍ·
    unsigned char motorID;  // µç»úID  0,1,2,3 ...   0xBB ±íÊ¾ÏòËùÓÐµç»ú¹ã²¥£¨´ËÊ±ÎÞ·µ»Ø£©
    unsigned char reserved;
} COMHead;

typedef struct
{ // ÒÔ 4¸ö×Ö½ÚÒ»×éÅÅÁÐ £¬²»È»±àÒëÆ÷»á´ÕÕû
    // ¶¨Òå Êý¾Ý
    uint8_t mode;    // µ±Ç°¹Ø½ÚÄ£Ê½
    uint8_t ReadBit; // µç»ú¿ØÖÆ²ÎÊýÐÞ¸Ä     ÊÇ·ñ³É¹¦Î»
    int8_t Temp;     // µç»úµ±Ç°Æ½¾ùÎÂ¶È
    uint8_t MError;  // µç»ú´íÎó ±êÊ¶

    COMData32 Read; // ¶ÁÈ¡µÄµ±Ç° µç»ú µÄ¿ØÖÆÊý¾Ý
    int16_t T;      // µ±Ç°Êµ¼Êµç»úÊä³öÁ¦¾Ø       7 + 8 ÃèÊö

    int16_t W; // µ±Ç°Êµ¼Êµç»úËÙ¶È£¨¸ßËÙ£©   8 + 7 ÃèÊö
    float LW;  // µ±Ç°Êµ¼Êµç»úËÙ¶È£¨µÍËÙ£©

    int16_t W2; // µ±Ç°Êµ¼Ê¹Ø½ÚËÙ¶È£¨¸ßËÙ£©   8 + 7 ÃèÊö
    float LW2;  // µ±Ç°Êµ¼Ê¹Ø½ÚËÙ¶È£¨µÍËÙ£©

    int16_t Acc;    // µç»ú×ª×Ó¼ÓËÙ¶È       15+0 ÃèÊö  ¹ßÁ¿½ÏÐ¡
    int16_t OutAcc; // Êä³öÖá¼ÓËÙ¶È         12+3 ÃèÊö  ¹ßÁ¿½Ï´ó

    int32_t Pos;  // µ±Ç°µç»úÎ»ÖÃ£¨Ö÷¿Ø0µãÐÞÕý£¬µç»ú¹Ø½Ú»¹ÊÇÒÔ±àÂëÆ÷0µãÎª×¼£©
    int32_t Pos2; // ¹Ø½Ú±àÂëÆ÷Î»ÖÃ(Êä³ö±àÂëÆ÷)

    int16_t gyro[3]; // µç»úÇý¶¯°å6Öá´«¸ÐÆ÷Êý¾Ý
    int16_t acc[3];

    // Á¦´«¸ÐÆ÷µÄÊý¾Ý
    int16_t Fgyro[3];
    int16_t Facc[3];
    int16_t Fmag[3];
    uint8_t Ftemp; // 8Î»±íÊ¾µÄÎÂ¶È  7Î»£¨-28~100¶È£©  1Î»0.5¶È·Ö±æÂÊ

    int16_t Force16; // Á¦´«¸ÐÆ÷¸ß16Î»Êý¾Ý
    int8_t Force8;   // Á¦´«¸ÐÆ÷µÍ8Î»Êý¾Ý

    uint8_t FError; //  ×ã¶Ë´«¸ÐÆ÷´íÎó±êÊ¶

    int8_t Res[1]; // Í¨Ñ¶ ±£Áô×Ö½Ú

} ServoComdV3; // ¼ÓÉÏÊý¾Ý°üµÄ°üÍ· ºÍCRC 78×Ö½Ú£¨4+70+4£©

typedef struct
{
    uint8_t head[2]; // °üÍ·         2Byte
    RIS_Mode_t mode; // µç»ú¿ØÖÆÄ£Ê½  1Byte
    RIS_Fbk_t fbk;   // µç»ú·´À¡Êý¾Ý 11Byte
    uint16_t CRC16;  // CRC          2Byte
} MotorData_t;       // ·µ»ØÊý¾Ý

typedef struct
{
    uint8_t none[8]; // ±£Áô

} LowHzMotorCmd;

typedef struct
{                      // ÒÔ 4¸ö×Ö½ÚÒ»×éÅÅÁÐ £¬²»È»±àÒëÆ÷»á´ÕÕû
                       // ¶¨Òå Êý¾Ý
    uint8_t mode;      // ¹Ø½ÚÄ£Ê½Ñ¡Ôñ
    uint8_t ModifyBit; // µç»ú¿ØÖÆ²ÎÊýÐÞ¸ÄÎ»
    uint8_t ReadBit;   // µç»ú¿ØÖÆ²ÎÊý·¢ËÍÎ»
    uint8_t reserved;

    COMData32 Modify; // µç»ú²ÎÊýÐÞ¸Ä µÄÊý¾Ý
    // Êµ¼Ê¸øFOCµÄÖ¸ÁîÁ¦¾ØÎª£º
    //  K_P*delta_Pos + K_W*delta_W + T
    int16_t T;   // ÆÚÍû¹Ø½ÚµÄÊä³öÁ¦¾Ø£¨µç»ú±¾ÉíµÄÁ¦¾Ø£©x256, 7 + 8 ÃèÊö
    int16_t W;   // ÆÚÍû¹Ø½ÚËÙ¶È £¨µç»ú±¾ÉíµÄËÙ¶È£© x128,       8 + 7ÃèÊö
    int32_t Pos; // ÆÚÍû¹Ø½ÚÎ»ÖÃ x 16384/6.2832, 14Î»±àÂëÆ÷£¨Ö÷¿Ø0µãÐÞÕý£¬µç»ú¹Ø½Ú»¹ÊÇÒÔ±àÂëÆ÷0µãÎª×¼£©

    int16_t K_P; // ¹Ø½Ú¸Õ¶ÈÏµÊý x2048  4+11 ÃèÊö
    int16_t K_W; // ¹Ø½ÚËÙ¶ÈÏµÊý x1024  5+10 ÃèÊö

    uint8_t LowHzMotorCmdIndex; // ±£Áô
    uint8_t LowHzMotorCmdByte;  // ±£Áô

    COMData32 Res[1]; // Í¨Ñ¶ ±£Áô×Ö½Ú  ÓÃÓÚÊµÏÖ±ðµÄÒ»Ð©Í¨Ñ¶ÄÚÈÝ

} MasterComdV3; // ¼ÓÉÏÊý¾Ý°üµÄ°üÍ· ºÍCRC 34×Ö½Ú

typedef struct
{
    // ¶¨Òå µç»ú¿ØÖÆÃüÁîÊý¾Ý°ü
    uint8_t head[2]; // °üÍ·         2Byte
    RIS_Mode_t mode; // µç»ú¿ØÖÆÄ£Ê½  1Byte
    RIS_Comd_t comd; // µç»úÆÚÍûÊý¾Ý 12Byte
    uint16_t CRC16;  // CRC          2Byte
} ControlData_t;     // µç»ú¿ØÖÆÃüÁîÊý¾Ý°ü

typedef struct
{
    // ¶¨Òå ·¢ËÍ¸ñÊ½»¯Êý¾Ý
    ControlData_t motor_send_data; // µç»ú¿ØÖÆÊý¾Ý½á¹¹Ìå
    int hex_len;                   // ·¢ËÍµÄ16½øÖÆÃüÁîÊý×é³¤¶È, 34
    long long send_time;           // ·¢ËÍ¸ÃÃüÁîµÄÊ±¼ä, Î¢Ãë(us)
    // ´ý·¢ËÍµÄ¸÷ÏîÊý¾Ý
    unsigned short id;   // µç»úID£¬0´ú±íÈ«²¿µç»ú
    unsigned short mode; // 0:¿ÕÏÐ, 5:¿ª»·×ª¶¯, 10:±Õ»·FOC¿ØÖÆ
    // Êµ¼Ê¸øFOCµÄÖ¸ÁîÁ¦¾ØÎª£º
    //  K_P*delta_Pos + K_W*delta_W + T
    float T;       // ÆÚÍû¹Ø½ÚµÄÊä³öÁ¦¾Ø£¨µç»ú±¾ÉíµÄÁ¦¾Ø£©£¨Nm£©
    float W;       // ÆÚÍû¹Ø½ÚËÙ¶È£¨µç»ú±¾ÉíµÄËÙ¶È£©(rad/s)
    float Pos;     // ÆÚÍû¹Ø½ÚÎ»ÖÃ£¨rad£©
    float K_P;     // ¹Ø½Ú¸Õ¶ÈÏµÊý
    float K_W;     // ¹Ø½ÚËÙ¶ÈÏµÊý
    COMData32 Res; // Í¨Ñ¶ ±£Áô×Ö½Ú  ÓÃÓÚÊµÏÖ±ðµÄÒ»Ð©Í¨Ñ¶ÄÚÈÝ
} MOTOR_send;

typedef struct
{
    // ¶¨Òå ½ÓÊÕÊý¾Ý
    MotorData_t motor_recv_data; // µç»ú½ÓÊÕÊý¾Ý½á¹¹Ìå£¬Ïê¼ûmotor_msg.h
    int hex_len;                 // ½ÓÊÕµÄ16½øÖÆÃüÁîÊý×é³¤¶È, 78
    long long resv_time;         // ½ÓÊÕ¸ÃÃüÁîµÄÊ±¼ä, Î¢Ãë(us)
    int correct;                 // ½ÓÊÕÊý¾ÝÊÇ·ñÍêÕû£¨1ÍêÕû£¬0²»ÍêÕû£©
    // ½â¶ÁµÃ³öµÄµç»úÊý¾Ý
    unsigned char motor_id; // µç»úID
    unsigned char mode;     // 0:¿ÕÏÐ, 5:¿ª»·×ª¶¯, 10:±Õ»·FOC¿ØÖÆ
    int Temp;               // ÎÂ¶È
    unsigned char MError;   // ´íÎóÂë
    float T;                // µ±Ç°Êµ¼Êµç»úÊä³öÁ¦¾Ø
    float W;                // speed
    float Pos;              // µ±Ç°µç»úÎ»ÖÃ£¨Ö÷¿Ø0µãÐÞÕý£¬µç»ú¹Ø½Ú»¹ÊÇÒÔ±àÂëÆ÷0µãÎª×¼£©
    float footForce;        // ×ã¶ËÆøÑ¹´«¸ÐÆ÷Êý¾Ý 12bit (0-4095)

} MOTOR_recv;

class go1 : public ITaskProcessor, public SerialDevice
{
public:
    MOTOR_send cmd;
    int modify_data(MOTOR_send *motor_s);

public:
    void handleReceiveData(uint8_t byte);
    void process_data();

    go1(UART_HandleTypeDef *huart);
};

#endif
#endif