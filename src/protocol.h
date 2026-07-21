#pragma once

#include <stddef.h>

enum MessageType : uint8_t {
    MSG_TYPE_SET_THROTTLE = 0,
    MSG_TYPE_TOGGLE_TC = 1,
    MSG_TYPE_TOGGLE_CC = 2,
    MSG_TYPE_INC_CC = 3,
    MSG_TYPE_DEC_CC = 4,
    MSG_TYPE_SET_CONSTANTS = 5,
};

struct MessageSetThrottle {
    enum MessageType type;
    uint8_t throttle;
    uint8_t is_forwards;
} __attribute__((packed));

struct MessageToggleTc {
    enum MessageType type;
} __attribute__((packed));

struct MessageToggleCc {
    enum MessageType type;
} __attribute__((packed));

struct MessageIncCc {
    enum MessageType type;
} __attribute__((packed));

struct MessageDecCc {
    enum MessageType type;
} __attribute__((packed));

struct MessageSetConstants {
    enum MessageType type;
    float Kp;
    float Ki;
    float Kd;
    float time_constant;
    uint8_t input_filter;
} __attribute__((packed));

typedef union {
    enum MessageType type;
    struct MessageSetThrottle set_throttle;
    struct MessageToggleTc toggle_tc;
    struct MessageToggleCc toggle_cc;
    struct MessageIncCc inc_cc;
    struct MessageDecCc dec_cc;
    struct MessageSetConstants set_constants;
} __attribute__((packed)) Message_t;

#define MESSAGE_SIZE sizeof(Message_t)

#pragma pack(push, 1)

enum MessageOutType : uint8_t {
    MSG_OUT_TYPE_LOG = 0,
};

struct MessageOutLogPayload {
    // 0.0 to 1.0
    uint8_t throttle;

    // 0 RPM to 255 RPM
    uint8_t front_right_rpm;
    uint8_t front_left_rpm;
    uint8_t rear_right_rpm;
    uint8_t rear_left_rpm;
    uint8_t rear_right_target_rpm;
    uint8_t rear_left_target_rpm;

    // 0.0 to 1.0
    uint8_t rear_left_pwm;
    uint8_t rear_right_pwm;
};

union MessageOutPayload {
    struct MessageOutLogPayload log;
};

#define START_OF_FRAME_MARKER 0xAA

typedef struct {
    uint8_t sof;
    enum MessageOutType type;
    union MessageOutPayload payload;
} MessageOut_t;

#define MESSAGE_OUT_SIZE sizeof(MessageOut_t)

#pragma pack(pop)
