#ifndef SERIAL_PROTOCOL_HPP
#define SERIAL_PROTOCOL_HPP

#include <cstdint>
#include <cstddef>
#include <cstring>

namespace serial_protocol {

#pragma pack(push, 1)

// 帧格式：SOF(0xFF) + payload(结构体字节) + EOF(0xFE)
inline constexpr uint8_t kFrameStart = 0xff;
inline constexpr uint8_t kFrameEnd = 0xfe;

//发送
struct TxData{
    float linear_x;
    float linear_y;
    float angular_z;
    uint8_t spin_or_not; //默认0不旋转 旋转1
    uint8_t posture; //移动姿态默认3 进攻姿态1 防御姿态2
};

//接收
struct RxData{
    float gimbal_yaw;
    uint8_t game_progress;                                      
    uint16_t stage_remain_time;
    uint16_t current_hp;
    uint16_t projectile_allowance_17mm;
    
    uint8_t rfid;
    uint16_t my_base_hp;
    uint16_t my_outpost_hp;
    uint16_t enemy_base_hp;
    uint16_t enemy_outpost_hp;
};

template<typename T>
struct SerialFrame{
    uint8_t start = kFrameStart; //0xff
    T payload;
    uint8_t end = kFrameEnd; //0xfe
};

#pragma pack(pop)


} //namespace serial_protocol

#endif