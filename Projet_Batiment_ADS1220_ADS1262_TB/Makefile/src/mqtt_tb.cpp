#include "mqtt_tb.hpp"
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>

using namespace std;

// === Variables dynamiques ADS1220 ===
int PGA = 1;
float VREF = 2.048;
float VFSR = VREF / PGA;



void sendToThingsboard_MQTT(float volt, float temp){

    const char* address = "tcp://192.168.1.46:1883";
    const char* token = "Huyzo4MPZwf22VT5uwsN";

    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;

    MQTTClient_create(&client, address, "RPI_ADS_CLIENT", MQTTCLIENT_PERSISTENCE_NONE, NULL);

    conn_opts.username = token;

    int rc = MQTTClient_connect(client, &conn_opts);

    if (rc!=MQTTCLIENT_SUCCESS)
    {
        cout << " Erreur connexion MQTT : " << rc << endl;
    }

    string payload ="{\"Tension\":" + to_string(volt) + 
                    ", \"Temperature\": " + to_string(temp) + "}";

    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    pubmsg.payload = (void*)payload.c_str();
    pubmsg.payloadlen = payload.length();
    pubmsg.qos = 1;
    pubmsg.retained = 0;

    MQTTClient_deliveryToken token_id;
    MQTTClient_publishMessage(client,"v1/devices/me/telemetry",&pubmsg,&token_id);
    
    MQTTClient_waitForCompletion(client, token_id, 1000);
    
    MQTTClient_disconnect(client, 1000);
    MQTTClient_destroy(&client);
}

void handleRPC(Protocentral_ADS1220& adc, const std::string& payload){

    Json::Value root;
    Json::Reader reader;

    bool configChanged = false;

    if(!reader.parse(payload,root)){
        return;
    }

    string method = root["method"].asString();
    int params = root["params"].asInt();

    if (method == "set_pga_gain")
    {
        switch (params)
        {
        case 1:
            adc.set_pga_gain(PGA_GAIN_1);
            break;
        case 2:
            adc.set_pga_gain(PGA_GAIN_2);
            break;
        case 4:
            adc.set_pga_gain(PGA_GAIN_4);
            break;
        case 8:
            adc.set_pga_gain(PGA_GAIN_8);
            break;
        case 16:
            adc.set_pga_gain(PGA_GAIN_16);
            break;
        case 32:
            adc.set_pga_gain(PGA_GAIN_32);
            break;
        case 64:
            adc.set_pga_gain(PGA_GAIN_64);
            break;
        case 128:
            adc.set_pga_gain(PGA_GAIN_128);
            break;
        default:
            cout << "RPC PGA invalide : " << params << endl;
            return; 
        
        }
        PGA = params; //Met à jour la variable
        VFSR =VREF / PGA; //Recalcule VFSR
        configChanged = true;
        cout <<"RPC PGA Gain réglé à " << PGA <<endl;
    }
    else if (method == "set_data_rate")
    {
        switch (params)
        {
        case 20:
            adc.set_data_rate(DR_20SPS);
            break;
        case 45:
            adc.set_data_rate(DR_45SPS);
            break;
        case 90:
            adc.set_data_rate(DR_90SPS);
            break;
        case 175:
            adc.set_data_rate(DR_175SPS);
            break;
        case 330:
            adc.set_data_rate(DR_330SPS);
            break;
        case 600:
            adc.set_data_rate(DR_600SPS);
            break;
        case 1000:
            adc.set_data_rate(DR_1000SPS);
            break;
        default:
            cout << "RPC DataRate invalide : " << params << endl;
            return;
        }
        configChanged = true;
        cout <<"RPC Data Rate réglé à " << params << "SPS" << endl;
    }

    if (configChanged)
    {
        adc.Start_Conv();
    }
           
}

