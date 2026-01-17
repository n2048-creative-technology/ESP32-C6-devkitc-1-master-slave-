#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_now.h"
#include "config.h"

void setupRadio();
void radioSetReceiveCallback(esp_now_recv_cb_t cb);
esp_err_t radioSendBroadcast(const void *data, size_t len);
esp_err_t radioSendUnicast(const uint8_t mac[6], const void *data, size_t len);
uint32_t getNodeId();
uint32_t millis();
void printNodeId(const char *roleTag);
void radioNotePeer(const uint8_t mac[6]);
void radioNoteRx();
void logNodeList();
uint32_t radioUpdatePeerCount();
uint32_t radioGetPeerCount();
uint32_t radioGetConfirmedCount();
bool radioRememberPacket(uint32_t origin_id, uint32_t seq);

struct RadioStats {
  uint32_t tx_attempts;
  uint32_t tx_success;
  uint32_t tx_fail;
  uint32_t rx_packets;
  uint32_t last_tx_ms;
  uint32_t last_rx_ms;
};

void radioGetStats(RadioStats *out);
void radioResetStats();
