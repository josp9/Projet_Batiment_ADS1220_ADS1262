#include "ads1220.hpp"
#include "ads1262.hpp"
#include "mqtt_tb.hpp"

#include <iostream>
#include <unistd.h>

using namespace std;

#define CS_PIN 4
#define DRDY_PIN 5

//=== ADS1220 ===
//#define PGA 1
//#define VREF 2.048
//#define VFSR VREF/PGA
#define FULL_SCALE_ADS1220 (((long int)1<<23)-1)

float convertToVoltAds1220(int32_t i32data){

  float raw_convert =(i32data * VFSR ) / FULL_SCALE_ADS1220;

  return raw_convert;
}

float readADS1220Temperature(Protocentral_ADS1220& adc){
    //Active le mode capteur de temperature interne
    adc.TemperatureSensorMode_enable();
    usleep(50000);

    //Lecture brute
    int32_t raw_temp = adc.Read_SingleShot_WaitForData();

    float temperature = (raw_temp /1000.0 *0.03125);

    //Desactive le mode capteur de temperature
    adc.TemperatureSensorMode_disable();
    
    return temperature;
}

int main() {

    cout << "=== Sélection du convertisseur ADC ===" << endl;
    cout << "1. ADS1220" << endl;
    cout << "2. ADS1262" <<endl;
    cout << "Choisissez un module (1 ou 2) : ";

    int choix;
    cin >> choix;

    if (choix==1)
    {
        Protocentral_ADS1220 pc_ads1220;

        cout << "Initialisation du module ADS1220..." << endl;

        if (!pc_ads1220.init("/dev/spidev0.0", CS_PIN, DRDY_PIN)) {
            cerr << "Erreur : impossible d'initialiser le module ADS1220." << endl;
            return -1;
        }

        cout << "Module initialisé avec succès !" << endl;


        // Configuration de base
        //pc_ads1220.select_mux_channels(MUX_SE_CH0);   // Canal AIN0 par rapport à GND
        pc_ads1220.set_pga_gain(PGA_GAIN_1);          // Gain = 1
        pc_ads1220.PGA_ON();
        pc_ads1220.set_data_rate(DR_20SPS);          // 20 échantillons/s
        pc_ads1220.set_conv_mode_single_shot();       // Mode échantillonnage unique
        //pc_ads1220.internal_reference();              // Référence interne 2.048 V

        const char* address ="tcp://192.168.1.46:1883";
        const char* token ="Huyzo4MPZwf22VT5uwsN";

        MQTTClient client;
        MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
        MQTTClient_create(&client, address, "RPI_ADS_CLIENT", MQTTCLIENT_PERSISTENCE_NONE, NULL);
        conn_opts.username=token;
        MQTTClient_connect(client, &conn_opts);
        MQTTClient_subscribe(client, "v1/devices/me/rpc/request/+", 1);

        MQTTClient_setCallbacks(
                client,
                &pc_ads1220,
                NULL,
                [](void* context, char* topicName, int topicLen,
                MQTTClient_message* message) -> int
                {
                    (void)topicLen;
                    Protocentral_ADS1220* adc = static_cast<Protocentral_ADS1220*>(context);
                    string payload((char*)message->payload, message->payloadlen);
                    handleRPC(*adc, payload);
                    MQTTClient_freeMessage(&message);
                    MQTTClient_free(topicName);
                    return 1;
                },
                NULL
            );


        cout << "Configuration terminée." << endl;

        while (true) {
            //Lecture Tension
            //int32_t raw_data = adc.Read_SingleShot_WaitForData();
            int32_t raw_data = pc_ads1220.Read_SingleShot_SingleEnded_WaitForData(MUX_AIN0_AIN1);
            float ADS1220_voltage = convertToVoltAds1220(raw_data);

            //Lecture Temperature
            float temp = readADS1220Temperature(pc_ads1220);

            cout << "Donnée brute : " << raw_data
                 << " | PGA : "<<PGA
                 << " | Tension : " << ADS1220_voltage << " V" 
                 << " | Temperature : " << temp << "°C" << endl;

            //Envoi vers ThingsBoard     
            sendToThingsboard_MQTT(ADS1220_voltage,temp);
           

        sleep(2);
        }
        
    }else if (choix==2)
    {
        // --- ADS1262 ---
        ADS1262 p_adc1262("/dev/spidev0.0", 2000000);
        cout << "\nInitialisation du module ADS1262 ..." <<endl;
        
        if (!p_adc1262.init())
        {
            cerr << "Erreur : impossible d'initialiser le module ADS1262" <<endl;
            return -1;
        }
        
        cout << "Configuration ADS1262 terminée" <<endl;

        uint8_t buffer[4];
        p_adc1262.start_Read_Data_Continuous();

        while (true)
        {
     
            p_adc1262.readData(buffer, 4);

            // Reconstruction du mot 32 bits 
            int32_t rawADS62 = (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];

            if (rawADS62 & 0x800000)
            {
                rawADS62 |= 0xFF000000;
            }

            float volt = (rawADS62 * 1) / 1;

            cout << "Donnée brute : " << rawADS62 << " | Tensoin : " << volt << "V" <<endl;

            sleep(2);
            

        }
        


    }else{

        cerr << "Choix invalide. Veuillez relancer le programme." << endl;
        return -1;
    }
       

    return 0;
}
