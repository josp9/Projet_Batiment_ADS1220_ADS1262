#ifndef MQTT_TB_HPP
#define MQTT_TB_HPP

#include <MQTTClient.h>
#include <string>
#include <jsoncpp/json/json.h>
#include "ads1220.hpp"

// === Variables dynamiques ADS1220 ===
extern int PGA;
extern float VREF;
extern float VFSR;


void sendToThingsboard_MQTT(float volt);

void handleRPC(Protocentral_ADS1220& adc, const std::string& payload);

#endif
