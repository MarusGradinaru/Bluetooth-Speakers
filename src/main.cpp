#include <Arduino.h>
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"

#define  pinBtStat  4
#define  pinDevEn   23
#define  pinPrev    13
#define  pinPlayPs  27
#define  pinNext    26

// ----- Custom A2DPSink class ------------------------------------------------

class BluetoothA2DPSinkEx : public BluetoothA2DPSink {
private:
  void decode_sbc_codec_config(const uint8_t sbc[4]) {
    // Sampling frequency (b7-b4 sbc[0])
    int sample_rate = 0;
    if (sbc[0] & 0x80) sample_rate = 16000;
    else if (sbc[0] & 0x40) sample_rate = 32000;
    else if (sbc[0] & 0x20) sample_rate = 44100;
    else if (sbc[0] & 0x10) sample_rate = 48000;

    // Channel mode (b3-b0 sbc[0])
    int channels = 2;
    bool joint_stereo = false;
    if (sbc[0] & 0x08) channels = 1;         // Mono
    else if (sbc[0] & 0x04) channels = 2;    // Dual channel
    else if (sbc[0] & 0x02) channels = 2;    // Stereo
    else if (sbc[0] & 0x01) {                // Joint Stereo
      channels = 2;
      joint_stereo = true;
    }

    // Blocks (b7–b4 sbc[1])
    int blocks = 0;
    if (sbc[1] & 0x80) blocks = 4;
    else if (sbc[1] & 0x40) blocks = 8;
    else if (sbc[1] & 0x20) blocks = 12;
    else if (sbc[1] & 0x10) blocks = 16;

    // Subbands (b3–b2 sbc[1])
    int subbands = 0;
    if (sbc[1] & 0x08) subbands = 4;
    else if (sbc[1] & 0x04) subbands = 8;

    // Allocation Method (b1-b0 sbc[1])  
    const char* alloc_method =
      (sbc[1] & 0x02) ? "SNR" :
      (sbc[1] & 0x01) ? "Loudness" : "Unknown";

    // Bitpool (sbc[2] sbc[3])
    uint8_t bitpool_min = sbc[2];
    uint8_t bitpool_max = sbc[3];

    // Calculate bitrate 
    int frame_length_min = 0; int frame_length_max = 0;
    if (channels == 1 || (sbc[0] & 0x0C) == 0x0C) {
      // Mono or Dual Channel
      frame_length_min = 4 + (4 * subbands * channels) / 8 + round((blocks * channels * bitpool_min) / 8.0);
      frame_length_max = 4 + (4 * subbands * channels) / 8 + round((blocks * channels * bitpool_max) / 8.0);
    } else {
      // Stereo or Joint Stereo
      int joint = joint_stereo ? subbands : 0;
      frame_length_min = 4 + (4 * subbands * channels) / 8 + round((joint + blocks * bitpool_min) / 8.0);
      frame_length_max = 4 + (4 * subbands * channels) / 8 + round((joint + blocks * bitpool_max) / 8.0);
    }
    float bitrate_min = (8.0f * frame_length_min * sample_rate) / (subbands * blocks);
    float bitrate_max = (8.0f * frame_length_max * sample_rate) / (subbands * blocks);
    int bitrate_min_kbps = (int)(bitrate_min / 1000.0 + 0.5);
    int bitrate_max_kbps = (int)(bitrate_max / 1000.0 + 0.5);

    // Show results
    Serial.println("===== Negotiated SBC Configuration =====");
    Serial.printf("Sample Rate      : %d Hz\n", sample_rate);
    Serial.printf("Channels         : %d (%s)\n", channels, joint_stereo ? "Joint Stereo" : (channels == 1 ? "Mono" : "Stereo"));
    Serial.printf("Blocks per Frame : %d\n", blocks);
    Serial.printf("Subbands         : %d\n", subbands);
    Serial.printf("Allocation       : %s\n", alloc_method);
    Serial.printf("Bitpool Range    : min = %d, max = %d\n", bitpool_min, bitpool_max);
    Serial.printf("Bitrate Estimate : min = %d kbps, max = %d kbps\n", bitrate_min_kbps, bitrate_max_kbps);
    Serial.println("========================================");
  }
public:
  BluetoothA2DPSinkEx(I2SStream &stream) : BluetoothA2DPSink(stream) {}  
protected:
  void handle_audio_cfg(uint16_t event, void *p_param) override {
    BluetoothA2DPSink::handle_audio_cfg(event, p_param);
    esp_a2d_cb_param_t *a2d = (esp_a2d_cb_param_t *)p_param;
    if (a2d->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
      uint8_t *sbc = a2d->audio_cfg.mcc.cie.sbc;
      decode_sbc_codec_config(sbc);
    }
  }
};


//--------------------------------------------------------------------

void read_data_stream(const uint8_t* data, uint32_t len) {
  static uint32_t total_bytes = 0;
  static uint32_t lastUpdate = 0;
  total_bytes += len;
  uint32_t now = millis();
  if (now - lastUpdate >= 1000) {
    Serial.printf("Received audio data: %u bytes\n", total_bytes);
    total_bytes = 0;
    lastUpdate = now;
  }
}

void rssi_callback(esp_bt_gap_cb_param_t::read_rssi_delta_param &rssiParam) {
  Serial.print("RSSI: ");
  Serial.print(rssiParam.rssi_delta);
  Serial.println(" dBm");
}

void connection_state_changed(esp_a2d_connection_state_t state, void* obj) {
  if (state >= ESP_A2D_CONNECTION_STATE_CONNECTED)
    digitalWrite(pinBtStat, HIGH); else digitalWrite(pinBtStat, LOW);
}

esp_avrc_playback_stat_t current_state = ESP_AVRC_PLAYBACK_STOPPED;

void playstatus_callback(esp_avrc_playback_stat_t playback) {
  current_state = playback;
}

I2SStream i2s;  
BluetoothA2DPSinkEx a2dp_sink(i2s);


// ----- Buttons & Interrupt Code ------------------------------------

QueueHandle_t btnQueue = xQueueCreate(5, sizeof(uint8_t));
const uint8_t btnPrev = pinPrev, btnPlayPs = pinPlayPs, btnNext = pinNext;
void IRAM_ATTR onPrevPress(void)   { xQueueSendFromISR(btnQueue, &btnPrev, NULL); }
void IRAM_ATTR onPlayPsPress(void) { xQueueSendFromISR(btnQueue, &btnPlayPs, NULL); }
void IRAM_ATTR onNextPress(void)   { xQueueSendFromISR(btnQueue, &btnNext, NULL); }

void ButtonsTask(void* pvParameters) {
  uint8_t button;
  uint32_t lastTrigger = millis(), nowTrigger;
  while (true) {
    if (xQueueReceive(btnQueue, &button, portMAX_DELAY) == pdPASS) {
      nowTrigger = millis();
      if (nowTrigger - lastTrigger > 500) { 
        lastTrigger = nowTrigger; 
        switch (button) {
          case btnPrev: {
            delay(50);
            if (digitalRead(button) == LOW) {
              Serial.println("Prev pressed");
              a2dp_sink.previous();
            }
            break;
          }
          case btnPlayPs: {
            delay(50);
            if (digitalRead(button) == LOW) {
              Serial.println("Play/Pause pressed");
              if (current_state == ESP_AVRC_PLAYBACK_PLAYING) a2dp_sink.pause();
                else a2dp_sink.play();
            }
            break;
          }
          case btnNext: {
            delay(50);
            if (digitalRead(button) == LOW) {
              Serial.println("Next pressed");
              a2dp_sink.next();
            }
            break;
  } } } } }    
}

// =========================== PROGRAM STARTING POINT ============================

void setup() {
  pinMode(pinBtStat, OUTPUT);
  digitalWrite(pinBtStat, HIGH);  
  Serial.begin(115200);
  delay(1000);
  digitalWrite(pinBtStat, LOW);

  pinMode(pinPrev, INPUT_PULLUP);
  pinMode(pinPlayPs, INPUT_PULLUP);
  pinMode(pinNext, INPUT_PULLUP);

  Serial.print("Starting ButtonsTask... ");
  if (xTaskCreatePinnedToCore(ButtonsTask, "ButtonsTask", 2048, NULL, 1, NULL, 1) == pdPASS) {
    attachInterrupt(digitalPinToInterrupt(pinPrev), onPrevPress, FALLING);
    attachInterrupt(digitalPinToInterrupt(pinPlayPs), onPlayPsPress, FALLING);
    attachInterrupt(digitalPinToInterrupt(pinNext), onNextPress, FALLING);
    Serial.println("done !");
  } else {
    Serial.println("failed !");
    while (true) delay(1000);  
  }  

  Serial.println("Initializing Bluetooth A2DP...");

  // Setup I2S with custom pins
  auto cfg = i2s.defaultConfig(TX_MODE);   // TX_MODE for DAC output
  cfg.pin_bck = 18;                        // Bit Clock (BCK) pin - connect it to BCLK on ES9023P
  cfg.pin_ws = 19;                         // Word Select (WS/LRCK) pin - connect it to LRCK on ES9023P
  cfg.pin_data = 5;                        // Data Out (DOUT) pin - connect it to DATA on ES9023P
  // Setup audio format
  cfg.sample_rate = 44100;                 // Sampling Rate (44.1kHz default for Bluetooth audio)
  cfg.bits_per_sample = 16;                // 16 bit per sample
  cfg.channels = 2;                        // Stereo (2 channels)
  cfg.i2s_format = I2S_PHILIPS_FORMAT;     // Standard I2S format for ES9023P
  // Activate I2S configuration
  i2s.begin(cfg);

//  a2dp_sink.set_rssi_active(true);
//  a2dp_sink.set_rssi_callback(rssi_callback); 
//  a2dp_sink.set_stream_reader(read_data_stream, true);
  a2dp_sink.set_on_connection_state_changed(connection_state_changed, nullptr); 
  a2dp_sink.set_avrc_rn_playstatus_callback(playstatus_callback);
  a2dp_sink.start("ESP32 Speakers");  
  Serial.println("Bluetooth ready ! Waiting for connections...");

  pinMode(pinDevEn, OUTPUT);
  digitalWrite(pinDevEn, HIGH);  
}

void loop() { delay(1000); }


// ----- TO DO ---------
// * add Pin Access Protection Authentication
// * remove all "prints" frome Release version
// * disable DAC and PA when no active BT connection
// * add a WiFi html interface