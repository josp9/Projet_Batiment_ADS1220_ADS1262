#ifndef ADS1220
#define ADS1220


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <wiringPi.h>


//ADS1220 SPI commands
#define SPI_MASTER_DUMMY    0xFF
#define RESET               0x06   
#define START               0x08    
#define WREG  0x40
#define RREG  0x20

//Config registers
#define CONFIG_REG0_ADDRESS 0x00
#define CONFIG_REG1_ADDRESS 0x01
#define CONFIG_REG2_ADDRESS 0x02
#define CONFIG_REG3_ADDRESS 0x03

#define REG_CONFIG3_IDAC1routing_MASK    0xE0
#define REG_CONFIG3_IDAC2routing_MASK    0x1C
#define REG_CONFIG2_VREF_MASK            0xC0
#define REG_CONFIG2_FIR_MASK             0x30
#define REG_CONFIG2_IDACcurrent_MASK     0x07
#define REG_CONFIG1_MODE_MASK            0x18
#define REG_CONFIG1_DR_MASK       0xE0
#define REG_CONFIG0_PGA_GAIN_MASK 0x0E
#define REG_CONFIG0_MUX_MASK      0xF0

#define IDAC1_disable     0x00  
#define IDAC1_AIN0        0x20  
#define IDAC1_AIN1        0x40  
#define IDAC1_AIN2        0x60  
#define IDAC1_AIN3        0x80  
#define IDAC1_REFP0       0xA0  
#define IDAC1_REFN0       0xC0  
#define IDAC1_reserved    0xE0  

#define IDAC2_disable     0x00  
#define IDAC2_AIN0        0x04  
#define IDAC2_AIN1        0x08  
#define IDAC2_AIN2        0x0C  
#define IDAC2_AIN3        0x10  
#define IDAC2_REFP0       0x14  
#define IDAC2_REFN0       0x18  
#define IDAC2_reserved    0x1C  

#define IDAC_OFF     0x00  
#define IDAC_10      0x01  
#define IDAC_50      0x02  
#define IDAC_100     0x03  
#define IDAC_250     0x04  
#define IDAC_500     0x05  
#define IDAC_1000    0x06  
#define IDAC_1500    0x07  

#define FIR_OFF      0x00  
#define FIR_5060     0x10  
#define FIR_50Hz     0x20  
#define FIR_60Hz     0x30  

#define VREF_2048       0x00  
#define VREF_REFP0      0x40  
#define VREF_AIN0       0x80  
#define VREF_ANALOG     0xC0  

#define MODE_NORMAL     0x00  
#define MODE_DUTYCYCLE  0x08  
#define MODE_TURBO      0x10  
#define MODE_RESERVED   0x18  

#define DR_20SPS    0x00
#define DR_45SPS    0x20
#define DR_90SPS    0x40
#define DR_175SPS   0x60
#define DR_330SPS   0x80
#define DR_600SPS   0xA0
#define DR_1000SPS  0xC0

#define PGA_GAIN_1   0x00
#define PGA_GAIN_2   0x02
#define PGA_GAIN_4   0x04
#define PGA_GAIN_8   0x06
#define PGA_GAIN_16  0x08
#define PGA_GAIN_32  0x0A
#define PGA_GAIN_64  0x0C
#define PGA_GAIN_128 0x0E

#define MUX_AIN0_AIN1   0x00
#define MUX_AIN0_AIN2   0x10
#define MUX_AIN0_AIN3   0x20
#define MUX_AIN1_AIN2   0x30
#define MUX_AIN1_AIN3   0x40
#define MUX_AIN2_AIN3   0x50
#define MUX_AIN1_AIN0   0x60
#define MUX_AIN3_AIN2   0x70
#define MUX_AIN0_AVSS   0x80
#define MUX_AIN1_AVSS   0x90
#define MUX_AIN2_AVSS   0xA0
#define MUX_AIN3_AVSS   0xB0

#define MUX_SE_CH0      0x80
#define MUX_SE_CH1      0x90
#define MUX_SE_CH2      0xA0
#define MUX_SE_CH3      0xB0

class Protocentral_ADS1220 {
private:
    int spi_fd; // Descripteur device SPI

    uint8_t m_config_reg0; //Config reg0 (écriture)
    uint8_t m_config_reg1; //Config reg1
    uint8_t m_config_reg2; //Config reg2
    uint8_t m_config_reg3; //Config reg3

    uint8_t Config_Reg0; // Valeur Lue reg0 (Lecture)
    uint8_t Config_Reg1;  
    uint8_t Config_Reg2;
    uint8_t Config_Reg3;

    uint8_t DataReg[3]; // Buffer 3 octets 
    int m_drdy_pin; //Numero GPIO pour DRDY
    int m_cs_pin; //Numero GPIO pour CS (chip select)

    
    bool spi_transfer_ioctl(int fd, uint8_t *tx, uint8_t *rx, size_t len, uint32_t speed_hz = 2000000);

public:
    Protocentral_ADS1220();//constructeur
    ~Protocentral_ADS1220();//destrcuteur

    bool init(const char* spi_device, int cs_pin, int drdy_pin);//Initialise SPI + GPIO
    void ads1220_Reset();// Envoie commande Reset
    void Start_Conv(); //Envoi commande START/SYNC
    void SPI_Command(uint8_t data_in); //Envoie un octet SPI
    void writeRegister(uint8_t address, uint8_t value); // Ecrit registre
    uint8_t readRegister(uint8_t address); // Lit registre
    uint8_t* Read_Data(); //Lecture de données
    int32_t DataToInt(); //convertit DataReg[3] en int32 
    int32_t Read_WaitForData();//Attend DRDY puis lit et convertit
    int32_t Read_SingleShot_WaitForData();//Conversion single-shot puis lit
    int32_t Read_SingleShot_SingleEnded_WaitForData(uint8_t channel_no);// Select MUX et read
    int32_t Read_Data_Samples();//lecture brute et conversion
    bool WaitForData(unsigned int timeout_ms);//boucle polling DRDY avec timeout
    void PrintRegisterValues(); //Affiche les registres lus

    void select_mux_channels(int channels_conf);//Change champ MUX dans reg0 et ecrit
    void set_pga_gain(int pgagain);
    void PGA_ON();//Active PGA
    void PGA_OFF();//Desective PGA
    void set_data_rate(int datarate); //configure data rate
    void set_OperationMode(int OPmode);//Configure operation mode
    void set_conv_mode_single_shot();//configure mode conversion single-shot
    void set_conv_mode_continuous();//configure mode conversion continue
    void TemperatureSensorMode_enable();//Active mode capteur temperature
    void TemperatureSensorMode_disable();//Desactive mode capteur temperature
    void CurrentSources_ON(); //Active sources de courant IDAC
    void CurrentSources_OFF();// Desactive IDAC
    void set_VREF(int vref);// configure source référence
    void set_FIR_Filter(int filter);//configure filtre FIR
    void LowSideSwitch_OPEN();// Ouvre low-side switch
    void LowSideSwitch_CLOSED();//ferme low-side switch
    void set_IDAC_Current(int IDACcurrent);//configure courant IDAC
    void set_IDAC1_Route(int IDAC1routing);//configure routage IDAC1
    void set_IDAC2_Route(int IDAC2routing);//configure routage IDAC2
    void DRDYmode_default();//Configure DRDY mode default
    void DRDYmode_DOUT();
    void internal_reference();//Selection reference interne
    void external_reference();//Selection reference externe
    
    void cs_drive(bool level); //Gère l'état GPIO CS

    uint8_t* get_config_reg(); //Lit et retourne les registres
};

#endif