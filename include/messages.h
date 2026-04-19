#pragma once

#include <stdint.h>

static const uint16_t MSG_VERSION = 1;

enum MsgType : uint8_t {
  MSG_TYPE_PARAMS = 1,
  MSG_TYPE_HELLO = 2,
  MSG_TYPE_DISCOVER = 3,
  MSG_TYPE_ACK = 4,
};

struct __attribute__((packed)) ParamsPayload {
  float speed;
  float probability;
  uint8_t mode;
};

struct __attribute__((packed)) MeshPacket {
  uint16_t version;
  uint16_t size;
  uint8_t type;
  uint8_t ttl;
  uint32_t seq;
  uint32_t origin_id;
  ParamsPayload params;
};

static_assert(sizeof(ParamsPayload) == 9, "ParamsPayload size changed");
