#include "../include/common.h"

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "radio";

static uint8_t s_mac_addr[6] = {0};
static uint8_t s_broadcast_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint32_t s_node_id = 0;
static bool s_radio_ready = false;
static uint32_t s_tx_attempts = 0;
static uint32_t s_tx_success = 0;
static uint32_t s_tx_fail = 0;
static uint32_t s_rx_packets = 0;
static uint32_t s_last_tx_ms = 0;
static uint32_t s_last_rx_ms = 0;

static void sendCallback(const wifi_tx_info_t *info,
                         esp_now_send_status_t status) {
  (void)info;
  if (status == ESP_NOW_SEND_SUCCESS) {
    s_tx_success++;
  } else {
    s_tx_fail++;
  }
}

static esp_err_t radioEnsurePeer(const uint8_t mac[6]) {
  if (mac == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }
  if (esp_now_is_peer_exist(mac)) {
    return ESP_OK;
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.ifidx = WIFI_IF_STA;
  peer.channel = RADIO_CHANNEL;
  peer.encrypt = false;
  return esp_now_add_peer(&peer);
}

struct PeerEntry {
  uint8_t mac[6];
  uint32_t last_seen_ms;
  uint8_t seen_samples;
  uint8_t state;
  bool in_use;
};

static PeerEntry s_peers[16] = {};
static uint32_t s_peer_count = 0;
static uint32_t s_peer_confirmed = 0;

enum PeerState : uint8_t {
  PEER_CANDIDATE = 0,
  PEER_CONFIRMED = 1,
  PEER_STALE = 2,
};

struct SeenPacket {
  uint32_t origin_id;
  uint32_t seq;
  uint32_t last_seen_ms;
  bool in_use;
};

static SeenPacket s_seen[32] = {};

static uint32_t node_id_from_mac(const uint8_t mac[6]) {
  return ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) |
         ((uint32_t)mac[4] << 8) | (uint32_t)mac[5];
}

uint32_t millis() {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

uint32_t getNodeId() {
  return s_node_id;
}

void setupRadio() {
  if (s_radio_ready) {
    return;
  }

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_netif_init());
  esp_err_t event_ret = esp_event_loop_create_default();
  if (event_ret != ESP_OK && event_ret != ESP_ERR_INVALID_STATE) {
    ESP_ERROR_CHECK(event_ret);
  }

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_channel(RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE));

  ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, s_mac_addr));
  s_node_id = node_id_from_mac(s_mac_addr);

  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_send_cb(&sendCallback));

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, s_broadcast_addr, sizeof(s_broadcast_addr));
  peer.ifidx = WIFI_IF_STA;
  peer.channel = RADIO_CHANNEL;
  peer.encrypt = false;
  esp_err_t add_peer_ret = esp_now_add_peer(&peer);
  if (add_peer_ret != ESP_OK && add_peer_ret != ESP_ERR_ESPNOW_EXIST) {
    ESP_ERROR_CHECK(add_peer_ret);
  }

  s_radio_ready = true;
  ESP_LOGI(TAG, "ESP-NOW init ok channel=%u node_id=0x%08X", RADIO_CHANNEL,
           (unsigned)s_node_id);
}

void radioSetReceiveCallback(esp_now_recv_cb_t cb) {
  ESP_ERROR_CHECK(esp_now_register_recv_cb(cb));
}

esp_err_t radioSendBroadcast(const void *data, size_t len) {
  s_tx_attempts++;
  s_last_tx_ms = millis();
  return esp_now_send(s_broadcast_addr, (const uint8_t *)data, len);
}

esp_err_t radioSendUnicast(const uint8_t mac[6], const void *data, size_t len) {
  if (mac == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }
  s_tx_attempts++;
  s_last_tx_ms = millis();
  esp_err_t ret = radioEnsurePeer(mac);
  if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_EXIST) {
    return ret;
  }
  return esp_now_send(mac, (const uint8_t *)data, len);
}

void printNodeId(const char *roleTag) {
  ESP_LOGI(TAG, "%s NODE_ID=0x%08X", roleTag, (unsigned)getNodeId());
}

void radioNotePeer(const uint8_t mac[6]) {
  if (mac == nullptr) {
    return;
  }
  esp_err_t ret = radioEnsurePeer(mac);
  if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_EXIST) {
    ESP_LOGW(TAG, "peer add failed err=%d", (int)ret);
  }

  uint32_t now = millis();
  int match_idx = -1;
  int free_idx = -1;
  for (size_t i = 0; i < (sizeof(s_peers) / sizeof(s_peers[0])); ++i) {
    if (s_peers[i].in_use) {
      if (memcmp(s_peers[i].mac, mac, 6) == 0) {
        match_idx = (int)i;
        break;
      }
    } else if (free_idx < 0) {
      free_idx = (int)i;
    }
  }

  if (match_idx < 0 && free_idx >= 0) {
    memcpy(s_peers[free_idx].mac, mac, 6);
    s_peers[free_idx].last_seen_ms = now;
    s_peers[free_idx].seen_samples = 0;
    s_peers[free_idx].state = PEER_CANDIDATE;
    s_peers[free_idx].in_use = true;
    match_idx = free_idx;
  } else if (match_idx >= 0) {
    s_peers[match_idx].last_seen_ms = now;
  }

  // Promote candidates once they are seen a few times.
  if (match_idx >= 0) {
    if (s_peers[match_idx].state == PEER_STALE) {
      s_peers[match_idx].state = PEER_CONFIRMED;
      s_peers[match_idx].seen_samples = PEER_CONFIRM_SAMPLES;
    }
    if (s_peers[match_idx].seen_samples < 0xFF) {
      s_peers[match_idx].seen_samples++;
    }
    if (s_peers[match_idx].state != PEER_CONFIRMED &&
        s_peers[match_idx].seen_samples >= PEER_CONFIRM_SAMPLES) {
      s_peers[match_idx].state = PEER_CONFIRMED;
    }
  }
}

void radioNoteRx() {
  s_rx_packets++;
  s_last_rx_ms = millis();
}

void logNodeList() {
  uint32_t now = millis();
  (void)radioUpdatePeerCount();
  ESP_LOGI(TAG, "peers=%u confirmed=%u", (unsigned)s_peer_count,
           (unsigned)s_peer_confirmed);

      for (size_t i = 0; i < (sizeof(s_peers) / sizeof(s_peers[0])); ++i) {
    if (!s_peers[i].in_use) {
      continue;
    }
    uint32_t age = now - s_peers[i].last_seen_ms;
    const char *state = "cand";
    if (s_peers[i].state == PEER_CONFIRMED) {
      state = "ok";
    } else if (s_peers[i].state == PEER_STALE) {
      state = "stale";
    }
    ESP_LOGI(TAG, "peer %02X:%02X:%02X:%02X:%02X:%02X age=%ums state=%s",
             s_peers[i].mac[0], s_peers[i].mac[1], s_peers[i].mac[2],
             s_peers[i].mac[3], s_peers[i].mac[4], s_peers[i].mac[5],
             (unsigned)age, state);
  }
}

uint32_t radioUpdatePeerCount() {
  uint32_t now = millis();
  uint32_t count = 0;
  uint32_t confirmed = 0;
  for (size_t i = 0; i < (sizeof(s_peers) / sizeof(s_peers[0])); ++i) {
    if (!s_peers[i].in_use) {
      continue;
    }
    uint32_t age = now - s_peers[i].last_seen_ms;
    if (age > PEER_REMOVE_MS) {
      s_peers[i].in_use = false;
      continue;
    }
    if (age > PEER_STALE_MS) {
      s_peers[i].state = PEER_STALE;
    }
    count++;
    if (s_peers[i].state == PEER_CONFIRMED) {
      confirmed++;
    }
  }
  s_peer_count = count;
  s_peer_confirmed = confirmed;
  return confirmed;
}

uint32_t radioGetPeerCount() {
  return s_peer_count;
}

uint32_t radioGetConfirmedCount() {
  return s_peer_confirmed;
}

bool radioRememberPacket(uint32_t origin_id, uint32_t seq) {
  uint32_t now = millis();
  int free_idx = -1;
  int oldest_idx = 0;
  uint32_t oldest_time = 0xFFFFFFFFu;

  for (size_t i = 0; i < (sizeof(s_seen) / sizeof(s_seen[0])); ++i) {
    if (s_seen[i].in_use) {
      if (s_seen[i].origin_id == origin_id && s_seen[i].seq == seq) {
        s_seen[i].last_seen_ms = now;
        return true;
      }
      if (s_seen[i].last_seen_ms < oldest_time) {
        oldest_time = s_seen[i].last_seen_ms;
        oldest_idx = (int)i;
      }
    } else if (free_idx < 0) {
      free_idx = (int)i;
    }
  }

  int idx = (free_idx >= 0) ? free_idx : oldest_idx;
  s_seen[idx].origin_id = origin_id;
  s_seen[idx].seq = seq;
  s_seen[idx].last_seen_ms = now;
  s_seen[idx].in_use = true;
  return false;
}

void radioGetStats(RadioStats *out) {
  if (out == nullptr) {
    return;
  }
  out->tx_attempts = s_tx_attempts;
  out->tx_success = s_tx_success;
  out->tx_fail = s_tx_fail;
  out->rx_packets = s_rx_packets;
  out->last_tx_ms = s_last_tx_ms;
  out->last_rx_ms = s_last_rx_ms;
}

void radioResetStats() {
  s_tx_attempts = 0;
  s_tx_success = 0;
  s_tx_fail = 0;
  s_rx_packets = 0;
  s_last_tx_ms = 0;
  s_last_rx_ms = 0;
}
