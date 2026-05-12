#pragma once

#include <stddef.h>

enum MessageType : uint8_t {
    MSG_TYPE_SET_SPEED = 0,
    MSG_TYPE_TOGGLE_TC = 1,
} __attribute__((packed));

struct MessageSetSpeed {
    enum MessageType type;
    uint8_t speed;
} __attribute__((packed));

struct MessageToggleTc {
    enum MessageType type;
} __attribute__((packed));

typedef union {
    enum MessageType type;
    struct MessageSetSpeed set_speed;
    struct MessageToggleTc toggle_tc;
} __attribute__((packed)) Message_t;

#define MESSAGE_SIZE sizeof(Message_t)
