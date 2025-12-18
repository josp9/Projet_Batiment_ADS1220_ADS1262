#include "ads1262.hpp"

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <string>
#include <wiringPi.h>

using namespace std;

//------- constructeur ------
ADS1262::ADS1262(string device, uint32_t spd){
    this->spi_device = device; //Non du device SPI
    this->speed = spd; // Frequence
    this->mode = SPI_MODE_1;// Mode SPI
    this->bits = 8; // 8 bits par mot SPI
    this->spi_fd = -1; // SPI non ouvert pour l'instant
}

//------- Destructeur ----
ADS1262::~ADS1262(){
    if (this->spi_fd >= 0)  //Si SPI ouvert
    {
        close(this->spi_fd ); //On ferme
    }
    
}

bool ADS1262::init(){

    wiringPiSetupGpio(); // Init GPIO

    pinMode(ADS1262_CS_PIN, OUTPUT);
    pinMode(ADS1262_START_PIN, OUTPUT);
    pinMode(ADS1262_PWDN_PIN, OUTPUT);
    pinMode(ADS1262_DRDY_PIN, INPUT);

    digitalWrite(ADS1262_CS_PIN, HIGH);
    digitalWrite(ADS1262_START_PIN, LOW);
    digitalWrite(ADS1262_PWDN_PIN, HIGH);

    //Ouverture du device SPI
    this->spi_fd = open(this->spi_device.c_str(), O_RDWR);
    if (this->spi_fd < 0) {
        perror("Erreur ouverture SPI");
        return false;
    }
    // Configuration du SPI
    ioctl(this->spi_fd, SPI_IOC_WR_MODE, &this->mode); //Mode SPI
    ioctl(this->spi_fd, SPI_IOC_WR_BITS_PER_WORD, &this->bits);
    ioctl(this->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &this->speed);

    cout << "SPI initialisé sur " << this->spi_device
              << " (" << this->speed << " Hz)" << endl;

    reset(); //Reset matériel
    usleep(100000); // Pause 100 ms

    hardStop(); // Stop matériel
    usleep(300000);

    //--- Ecriture initiale des registres ---
    writeRegister(POWER, 0x11);
    usleep(10000);
    writeRegister(INTERFACE, 0x05);
    usleep(10000);
    writeRegister(MODE0, 0x00);
    usleep(10000);
    writeRegister(MODE1, 0x80);
    usleep(10000);
    writeRegister(MODE2, 0x06);
    usleep(10000);
    writeRegister(INPMUX, 0x01);
    usleep(10000);
    writeRegister(OFCAL0, 0x00);
    usleep(10000);
    writeRegister(OFCAL1, 0x00);
    usleep(10000);
    writeRegister(OFCAL2, 0x00);
    usleep(10000);
    writeRegister(FSCAL0, 0x00);
    usleep(10000);
    writeRegister(FSCAL1, 0x00);
    usleep(10000);
    writeRegister(FSCAL2, 0x40);
    usleep(10000);
    writeRegister(IDACMUX, 0xBB);
    usleep(10000);
    writeRegister(IDACMAG, 0x00);
    usleep(10000);
    writeRegister(REFMUX, 0x00);
    usleep(10000);

    enableStart(); //Active conversions en continu
    return true;
}

//------Reset matériel -----
void ADS1262::reset(){

    cout << "Réinitialisation de module..." << endl;
    digitalWrite(ADS1262_PWDN_PIN, HIGH); //Mode normal
    usleep(100000);

    digitalWrite(ADS1262_PWDN_PIN, LOW); //Reset
    usleep(100000);

    digitalWrite(ADS1262_PWDN_PIN, HIGH); // On rallume
    usleep(100000);
}

//Start = HIGH
void ADS1262::enableStart(){
    digitalWrite(ADS1262_START_PIN, HIGH);

}

//Start = LOW
void ADS1262::disableStart(){
     digitalWrite(ADS1262_START_PIN, LOW);
    
}

// Sopt matériel
void ADS1262::hardStop(){
    disableStart(); //STOP
    usleep(100000);
}

//Envoie commande START
void ADS1262::start_Data_Conversion_Command(){
    sendCommand(START);
}

//Envoie commande STOP
void ADS1262::softStop(){
    sendCommand(STOP);
}

//Active mode lecture continue
void ADS1262::start_Read_Data_Continuous(){
    sendCommand(RDATAC);
}

// Stop lecture continue
void ADS1262::stop_Read_Data_Continuous(){
    sendCommand(SDATAC);
}

//----- Envoi commande SPI -----
void ADS1262::sendCommand(uint8_t cmd){
    uint8_t tx[1] = { cmd }; // Commande à envoyer
    digitalWrite(ADS1262_CS_PIN, LOW); // active SPI
    write(this->spi_fd, tx, 1); //Envoi 1 octect
    usleep(1000);
    digitalWrite(ADS1262_CS_PIN, HIGH);
}

void ADS1262::writeRegister(uint8_t address, uint8_t value) {
    //Commande WREG + adresse 
    //Nombre de registre(0=1 registre)
    //Valeur à écrire
    uint8_t tx[3] = { (uint8_t)(WREG | address), 0x00, value };
    digitalWrite(ADS1262_CS_PIN, LOW);
    write(this->spi_fd, tx, 3);
    usleep(2000);
    digitalWrite(ADS1262_CS_PIN, HIGH);
}

//------ Ecriture d'un registre --------
uint8_t ADS1262::readRegister(uint8_t address)
{
    uint8_t tx[3] = { (uint8_t)(RREG | address), 0x00, CONFIG_SPI_MASTER_DUMMY };
    uint8_t rx[3] = {0};// Buffer réception

    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx; //buffer envoi
    tr.rx_buf = (unsigned long)rx; //buffer reception
    tr.len = 3; //nombre d'octets
    tr.speed_hz = this->speed;
    tr.bits_per_word = this->bits;

    digitalWrite(ADS1262_CS_PIN, LOW);
    ioctl(this->spi_fd, SPI_IOC_MESSAGE(1), &tr); // Transaction SPI
    digitalWrite(ADS1262_CS_PIN, HIGH);
    usleep(50);

    return rx[2]; // Le 3ème octet contient la valeur lue
}

//------- Lecture de données ----------
void ADS1262::readData(uint8_t* buffer, size_t len) {
    //attente du signal DRDY = LOW
    int timeout = 1000000;
    while (digitalRead(ADS1262_DRDY_PIN) == HIGH && timeout > 0) {
        usleep(100);
        timeout -= 100;
    }

    if (timeout <= 0) {
        cerr << "Délai d'attente pour DRDY" << endl;
        return;
    }

    uint8_t tx[len];
    memset(tx, CONFIG_SPI_MASTER_DUMMY, len); //On envoie des 0xFF

    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)buffer;
    tr.len = len;
    tr.speed_hz = this->speed;
    tr.bits_per_word = this->bits;

    digitalWrite(ADS1262_CS_PIN, LOW);
    ioctl(this->spi_fd, SPI_IOC_MESSAGE(1), &tr);
    digitalWrite(ADS1262_CS_PIN, HIGH);
}
