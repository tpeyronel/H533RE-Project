#pragma once

#include <stddef.h>

enum MessageType : uint8_t {
    MSG_TYPE_SET_THROTTLE = 0,
    MSG_TYPE_TOGGLE_TC = 1,
    MSG_TYPE_TOGGLE_CC = 2,
    MSG_TYPE_INC_CC = 3,
    MSG_TYPE_DEC_CC = 4,
};

struct MessageSetThrottle {
    enum MessageType type;
    uint8_t throttle;
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

typedef union {
    enum MessageType type;
    struct MessageSetThrottle set_throttle;
    struct MessageToggleTc toggle_tc;
    struct MessageToggleCc toggle_cc;
    struct MessageIncCc inc_cc;
    struct MessageDecCc dec_cc;
} __attribute__((packed)) Message_t;

#define MESSAGE_SIZE sizeof(Message_t)

enum MessageOutType : uint8_t {
    MSG_OUT_TYPE_LOG = 0,
};

struct MessageOutLog {
    enum MessageOutType type;
    uint8_t throttle; // 0.0 to 1.0
    uint8_t rear_left_pwm; // 0.0 to 1.0
    uint8_t rear_right_pwm; // 0.0 to 1.0
    uint8_t rear_left_slip; // 0-255 representing 0.0 to 1.0 slip ratio (clamped)
    uint8_t rear_right_slip;
} __attribute__((packed));

typedef union {
    enum MessageOutType type;
    struct MessageOutLog log;
} __attribute__((packed)) MessageOut_t;

#define MESSAGE_OUT_SIZE sizeof(MessageOut_t)
