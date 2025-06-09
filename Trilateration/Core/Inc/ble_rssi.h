#ifndef __BLE_RSSI_H
#define __BLE_RSSI_H

#include "stdint.h"

#define NUM_DEVICES 3

typedef struct {
  float x;        // Kalman filter state (filtered RSSI)
  float P;        // Estimation error covariance
  uint8_t initialized; // Flag to check if filter is initialized
} KalmanState;

typedef struct {
  uint8_t mac[6];
  int8_t current_rssi;     // Current raw RSSI value
  float filtered_rssi;     // Current filtered RSSI value
  KalmanState kalman;      // Kalman filter state
} DeviceRSSI;

extern DeviceRSSI tracked_devices[NUM_DEVICES];

int match_mac(uint8_t *mac1, uint8_t *mac2);
float rssi_to_distance(char id, int8_t rssi);
void kalman_filter_update(KalmanState *state, float measurement);

#endif // __BLE_RSSI_H
