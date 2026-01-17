#include "../include/config.h"
#include "../include/master.h"
#include "../include/slave.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void app_main(void) {
#if NODE_ROLE == ROLE_MASTER
  setupMaster();
  while (true) {
    loopMaster();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
#else
  setupSlave();
  while (true) {
    loopSlave();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
#endif
}
