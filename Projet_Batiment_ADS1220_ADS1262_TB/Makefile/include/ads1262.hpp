#ifndef ADS1262_HPP
#define ADS1262_HPP

#include <cstdint>
#include <string>

#define CONFIG_SPI_MASTER_DUMMY   0xFF

// Commandes SPI
#define RREG 0x20
#define WREG 0x40		
#define START 0x08		
#define STOP  0x0A		
#define RDATAC 0x10		
#define SDATAC 0x11		
#define RDATA  0x12		

// GPIO pins
const int ADS1262_DRDY_PIN = 2;
const int ADS1262_CS_PIN   = 3;
const int ADS1262_START_PIN = 4;
const int ADS1262_PWDN_PIN  = 5;

// Registres
#define POWER      0x01
#define INTERFACE  0x02
#define MODE0      0x03
#define MODE1      0x04
#define MODE2      0x05
#define INPMUX     0x06
#define OFCAL0     0x07
#define OFCAL1     0x08
#define OFCAL2     0x09
#define FSCAL0     0x0A
#define FSCAL1     0x0B
#define FSCAL2     0x0C
#define IDACMUX    0x0D
#define IDACMAG    0x0E
#define REFMUX     0x0F
#define TDACP      0x10
#define TDACN      0x11
#define GPIOCON    0x12
#define GPIODIR    0x13
#define GPIODAT    0x14
#define ADC2CFG    0x15
#define ADC2MUX    0x16
#define ADC2OFC0   0x17
#define ADC2OFC1   0x18
#define ADC2FSC0   0x19
#define ADC2FSC1   0x1A

class ADS1262 {
	private:
	    int spi_fd; //File descriptor du périphérique SPI
	    std::string spi_device; //Nom du device
	    uint8_t mode; //Mode SPI
	    uint8_t bits; // Taille mot SPI
	    uint32_t speed; // Fréquence

	public:
	    //Constructeur
	    ADS1262(std::string device = "/dev/spidev0.0", uint32_t speed = 2000000);
		//Destructeur
	    ~ADS1262();

	    bool init(); //Initalisation générale
	    void reset();//Reset matériel
	    void writeRegister(uint8_t address, uint8_t value);//Ecriture registre
	    uint8_t readRegister(uint8_t address);// Lecture registre
	    void sendCommand(uint8_t cmd); // Envoi d'une commande simple

	    void enableStart();//Active START pin
	    void disableStart();// Desactive Start pin
	    void hardStop(); //Stop matériel
	    void start_Data_Conversion_Command();// Commande START
	    void softStop(); //Commande STOP
	    void start_Read_Data_Continuous(); //Active Lecture continue
	    void stop_Read_Data_Continuous(); // Stop lecture continue
	    void readData(uint8_t* buffer, size_t len); //Lecture de données
};

#endif 
