#include "ble_rssi.h"
#include "math.h"

DeviceRSSI tracked_devices[NUM_DEVICES] = { // little endian
  {
    .mac = {0x97, 0x5D, 0x97, 0xEB, 0x27, 0xB8},
    .kalman = {.x = 0.0f, .P = 1.0f, .initialized = 0}
  }, // RPi_1
  {
    .mac = {0x79, 0xD0, 0x8A, 0xEB, 0x27, 0xB8},
    .kalman = {.x = 0.0f, .P = 1.0f, .initialized = 0}
  }, // RPi_2
  {
    .mac = {0xEE, 0x89, 0xD9, 0xEB, 0x27, 0xB8},
    .kalman = {.x = 0.0f, .P = 1.0f, .initialized = 0}
  },
};

int match_mac(uint8_t *mac1, uint8_t *mac2) {
  for (int i = 0; i < 6; i++) {
    if (mac1[i] != mac2[i]) return 0;
  }
  return 1;
}

void kalman_filter_update(KalmanState *state, float measurement) {
    // Kalman filter parameters
    const float q = 0.47f;  // Process noise covariance
    const float r = 3.8f;    // Measurement noise covariance

    if (!state->initialized) {
        // Initialize with first measurement
        state->x = measurement;
        state->P = 1.0f;
        state->initialized = 1;
        return;
    }

    // Prediction step
    // x_pred = x (no motion model for RSSI)
    // P_pred = P + q
    state->P += q;

    // Update step
    float K = state->P / (state->P + r);  // Kalman gain
    state->x = state->x + K * (measurement - state->x);  // Update estimate
    state->P = (1.0f - K) * state->P;  // Update error covariance
}

float rssi_to_distance(char id, int8_t rssi) {
  float txPower; // Calibrated RSSI at 1 meter from the beacon
  switch(id){
  	  case 0: // Rpi 1
  		  txPower =  -60;
  		  break;
  	  case 1:
  		  txPower = -72;
  		  break;
  	  case 2:
  		  txPower = -72;
  }

  if (rssi == 0) return -1.0; // invalid RSSI
  float ratio = rssi * 1.0 / txPower;

  if (ratio < 1.0) return powf(ratio, 10);           // RSSI > txPower, distance < 1m
  else return 0.89976 * powf(ratio, 7.7095) + 0.111; // RSSI < txPower, distance >= 1m
}
