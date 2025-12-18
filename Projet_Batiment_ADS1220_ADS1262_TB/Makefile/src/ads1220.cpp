#include "ads1220.hpp"

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <string>

using namespace std;

#ifndef SPI_NO_CS
    #define SPI_NO_CS 0x40
#endif

Protocentral_ADS1220::Protocentral_ADS1220() {
    this->spi_fd = -1;
    this->m_cs_pin = 2;
    this->m_drdy_pin = 25;

    this->m_config_reg0 = 0x00;
    this->m_config_reg1 = 0x04;
    this->m_config_reg2 = 0x10;
    this->m_config_reg3 = 0x00;

    this->Config_Reg0 = 0;
    this->Config_Reg1 = 0;
    this->Config_Reg2 = 0;
    this->Config_Reg3 = 0;
    this->DataReg[0] = 0;
    this->DataReg[1] = 0;
    this->DataReg[2] = 0;
}

Protocentral_ADS1220::~Protocentral_ADS1220() {
    if (this->spi_fd >= 0) {
        close(this->spi_fd);
    }
}

bool Protocentral_ADS1220::spi_transfer_ioctl(int fd, uint8_t *tx, uint8_t *rx, size_t len, uint32_t speed_hz) {
    if (fd < 0) {
        return false;
    }

    struct spi_ioc_transfer tr; //Structure de message pour iotcl
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = len;
    tr.speed_hz = speed_hz;
    tr.delay_usecs = 0;
    tr.bits_per_word = 8;
    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    return (ret >= 0);
}

bool Protocentral_ADS1220::init(const char* spi_device, int cs_pin, int drdy_pin) {
    if (cs_pin >= 0) { //Si on a fourni un cs_pin valide
        this->m_cs_pin = cs_pin; //Met à jour le CS
    }

    if (drdy_pin >= 0) {  //Si on a fourni un drdy_pin valide
        this->m_drdy_pin = drdy_pin; //Met à jour DRDY pin
    }

    if (wiringPiSetupGpio() == -1) { //Initialise GPIO
        cerr << "wiringPiSetupGpio() a échoué - nécessite l'installation de wiringPi et des privilèges root\n";
        return false; // Echec initialisation GPIO
    }

    // Open SPI device
    this->spi_fd = open(spi_device, O_RDWR); //Ouvre le device SPI
    if (this->spi_fd < 0) { //Si echec ouverture
        perror("open(spi_device)"); //Affiche l'erreur
        return false;
    }

    // Configure SPI: mode = MODE1, bits = 8, speed = 2MHz
    uint8_t mode = SPI_MODE_1;
    uint8_t bits = 8;
    uint32_t speed = 2000000;

    mode |= SPI_NO_CS; //Indique au driver de ne pas gérer CS

    if (ioctl(this->spi_fd, SPI_IOC_WR_MODE, &mode) < 0) { //ecrire le mode
        mode &= ~SPI_NO_CS; // Retire le flag SPI_NO_CS si échec
        if (ioctl(this->spi_fd, SPI_IOC_WR_MODE, &mode) < 0) { //Ecrire sans SPI_NO_CS
            perror("SPI_IOC_WR_MODE");
            cerr << "Avertissement : impossible de configurer correctement le mode SPI\n";
        } else {
            cerr << "Avertissement : SPI_NO_CS non pris en charge ; le noyau peut basculer automatiquement CE0\n";
        }
    }

    //Configure bits/word
    if (ioctl(this->spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        perror("SPI_IOC_WR_BITS_PER_WORD");
        close(this->spi_fd);//Ferme SPI
        this->spi_fd = -1;
        return false;//echec
    }

    //Configure vitesse SPI
    if (ioctl(this->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("SPI_IOC_WR_MAX_SPEED_HZ");
        close(this->spi_fd);
        this->spi_fd = -1;
        return false;
    }

    pinMode(this->m_cs_pin, OUTPUT); // Configure la pin CS en Sortie
    digitalWrite(this->m_cs_pin, HIGH); // Met CS à HIGH

    pinMode(this->m_drdy_pin, INPUT);
    pullUpDnControl(this->m_drdy_pin, PUD_DOWN);

    ads1220_Reset(); //Envoi la commande Reset au convertisseur
    usleep(50000); // Attend 50ms

    this->m_config_reg0 = 0x00;
    this->m_config_reg1 = 0x04;
    this->m_config_reg2 = 0x10;
    this->m_config_reg3 = 0x00;

    writeRegister(CONFIG_REG0_ADDRESS, this->m_config_reg0);
    writeRegister(CONFIG_REG1_ADDRESS, this->m_config_reg1);
    writeRegister(CONFIG_REG2_ADDRESS, this->m_config_reg2);
    writeRegister(CONFIG_REG3_ADDRESS, this->m_config_reg3);

    return true;
}

//Controle logicile du CS
void Protocentral_ADS1220::cs_drive(bool level) {
    digitalWrite(m_cs_pin, level ? HIGH : LOW);
    delayMicroseconds(1);
}

void Protocentral_ADS1220::SPI_Command(uint8_t data_in) {//Envoi octet
    uint8_t tx = data_in; //octet à transmettre
    uint8_t rx = 0; //Buffer reception
    cs_drive(false); //Active CS (Low)
    spi_transfer_ioctl(this->spi_fd, &tx, &rx, 1);//Envoi 1 octet
    cs_drive(true);
    delayMicroseconds(1);
}

void Protocentral_ADS1220::writeRegister(uint8_t address, uint8_t value) { //Ecri registre
    uint8_t tx[2]; //buffer transmission
    uint8_t rx[2]; // buffer reception
    tx[0] = (uint8_t)(WREG | (address << 2)); // Commande WREG + adresse
    tx[1] = value;
    cs_drive(false);
    bool ok = spi_transfer_ioctl(this->spi_fd, tx, rx, 2);
    cs_drive(true);
    (void)ok;
    delayMicroseconds(2);
}

uint8_t Protocentral_ADS1220::readRegister(uint8_t address) {
    uint8_t tx[2];
    uint8_t rx[2];
    tx[0] = (uint8_t)(RREG | (address << 2));
    tx[1] = SPI_MASTER_DUMMY;
    cs_drive(false);
    bool ok = spi_transfer_ioctl(this->spi_fd, tx, rx, 2);
    cs_drive(true);
    (void)ok;
    delayMicroseconds(2);
    return rx[1];
}

//Envoi commande reset
void Protocentral_ADS1220::ads1220_Reset() {
    SPI_Command(RESET);
    usleep(10000);
}

void Protocentral_ADS1220::Start_Conv() {
    SPI_Command(START);
    delayMicroseconds(2);
}

void Protocentral_ADS1220::PrintRegisterValues() {
    this->Config_Reg0 = readRegister(CONFIG_REG0_ADDRESS);
    this->Config_Reg1 = readRegister(CONFIG_REG1_ADDRESS);
    this->Config_Reg2 = readRegister(CONFIG_REG2_ADDRESS);
    this->Config_Reg3 = readRegister(CONFIG_REG3_ADDRESS);

    cout << "Config_Reg : " << endl;
    cout << "Reg0 = 0x" << hex << (int)this->Config_Reg0 << dec << endl;
    cout << "Reg1 = 0x" << hex << (int)this->Config_Reg1 << dec << endl;
    cout << "Reg2 = 0x" << hex << (int)this->Config_Reg2 << dec << endl;
    cout << "Reg3 = 0x" << hex << (int)this->Config_Reg3 << dec << endl;
    cout << endl;
}

void Protocentral_ADS1220::select_mux_channels(int channels_conf) { //Modifie champ MUX dans reg0
    this->m_config_reg0 &= ~REG_CONFIG0_MUX_MASK;//Clear bit MUX du reg0
    this->m_config_reg0 |= (uint8_t)channels_conf;//Place nouvelle configuration MUX
    writeRegister(CONFIG_REG0_ADDRESS, this->m_config_reg0); //Ecrit reg0
}

void Protocentral_ADS1220::set_pga_gain(int pgagain) {// Modifie gain PGA
    this->m_config_reg0 &= ~REG_CONFIG0_PGA_GAIN_MASK;//clear bits PGA
    this->m_config_reg0 |= (uint8_t)pgagain;//Insére le nouveau gain
    writeRegister(CONFIG_REG0_ADDRESS, this->m_config_reg0);// Ecrit reg0
}

void Protocentral_ADS1220::PGA_ON(void) {
    this->m_config_reg0 &= ~(1 << 0);// Mets à 0 le bit 0
    writeRegister(CONFIG_REG0_ADDRESS, this->m_config_reg0);
}

void Protocentral_ADS1220::PGA_OFF(void) {
    this->m_config_reg0 |= (1 << 0);//Met à 1 le bit 0
    writeRegister(CONFIG_REG0_ADDRESS, this->m_config_reg0);
}

void Protocentral_ADS1220::set_data_rate(int datarate) {
    this->m_config_reg1 &= ~REG_CONFIG1_DR_MASK;//clear bit DR
    this->m_config_reg1 |= (uint8_t)datarate; //Insére valeur souhaite
    writeRegister(CONFIG_REG1_ADDRESS, this->m_config_reg1); //Ecrit reg1
}

void Protocentral_ADS1220::set_OperationMode(int OPmode) {
    this->m_config_reg1 &= ~REG_CONFIG1_MODE_MASK;// Clear bits mode
    this->m_config_reg1 |= (uint8_t)OPmode;
    writeRegister(CONFIG_REG1_ADDRESS, this->m_config_reg1);
}

void Protocentral_ADS1220::set_conv_mode_single_shot(void) {
    this->m_config_reg1 &= ~(1 << 2);//clear bit 2 pour single-shot
    writeRegister(CONFIG_REG1_ADDRESS, this->m_config_reg1);
}

void Protocentral_ADS1220::set_conv_mode_continuous(void) {
    this->m_config_reg1 |= (1 << 2);//set bit 2 pour continous
    writeRegister(CONFIG_REG1_ADDRESS, this->m_config_reg1);
}

void Protocentral_ADS1220::TemperatureSensorMode_disable(void) {
    this->m_config_reg1 &= ~(1 << 1);
    writeRegister(CONFIG_REG1_ADDRESS, this->m_config_reg1);
}

void Protocentral_ADS1220::TemperatureSensorMode_enable(void) {
    this->m_config_reg1 |= (1 << 1);
    writeRegister(CONFIG_REG1_ADDRESS, this->m_config_reg1);
}

void Protocentral_ADS1220::CurrentSources_OFF(void) {
    this->m_config_reg1 &= ~(1 << 0);
    writeRegister(CONFIG_REG1_ADDRESS, this->m_config_reg1);
}

void Protocentral_ADS1220::CurrentSources_ON(void) {
    this->m_config_reg1 |= (1 << 0);
    writeRegister(CONFIG_REG1_ADDRESS, this->m_config_reg1);
}

void Protocentral_ADS1220::set_VREF(int vref) {
    this->m_config_reg2 &= ~REG_CONFIG2_VREF_MASK;
    this->m_config_reg2 |= (uint8_t)vref;
    writeRegister(CONFIG_REG2_ADDRESS, this->m_config_reg2);
}

void Protocentral_ADS1220::set_FIR_Filter(int filter) {
    this->m_config_reg2 &= ~REG_CONFIG2_FIR_MASK;
    this->m_config_reg2 |= (uint8_t)filter;
    writeRegister(CONFIG_REG2_ADDRESS, this->m_config_reg2);
}

void Protocentral_ADS1220::LowSideSwitch_OPEN(void) {
    this->m_config_reg2 &= ~(1 << 3);
    writeRegister(CONFIG_REG2_ADDRESS, this->m_config_reg2);
}

void Protocentral_ADS1220::LowSideSwitch_CLOSED(void) {
    this->m_config_reg2 |= (1 << 3);
    writeRegister(CONFIG_REG2_ADDRESS, this->m_config_reg2);
}

void Protocentral_ADS1220::set_IDAC_Current(int IDACcurrent) {
    this->m_config_reg2 &= ~REG_CONFIG2_IDACcurrent_MASK;
    this->m_config_reg2 |= (uint8_t)IDACcurrent;
    writeRegister(CONFIG_REG2_ADDRESS, this->m_config_reg2);
}

void Protocentral_ADS1220::set_IDAC1_Route(int IDAC1routing) {
    this->m_config_reg3 &= ~REG_CONFIG3_IDAC1routing_MASK;
    this->m_config_reg3 |= (uint8_t)IDAC1routing;
    writeRegister(CONFIG_REG3_ADDRESS, this->m_config_reg3);
}

void Protocentral_ADS1220::set_IDAC2_Route(int IDAC2routing) {
    this->m_config_reg3 &= ~REG_CONFIG3_IDAC2routing_MASK;
    this->m_config_reg3 |= (uint8_t)IDAC2routing;
    writeRegister(CONFIG_REG3_ADDRESS, this->m_config_reg3);
}

void Protocentral_ADS1220::DRDYmode_default(void) {
    this->m_config_reg3 &= ~(1 << 3);
    writeRegister(CONFIG_REG3_ADDRESS, this->m_config_reg3);
}

void Protocentral_ADS1220::DRDYmode_DOUT(void) {
    this->m_config_reg3 |= (1 << 3);
    writeRegister(CONFIG_REG3_ADDRESS, this->m_config_reg3);
}

uint8_t* Protocentral_ADS1220::get_config_reg() {
    static uint8_t config_Buff[4];
    this->m_config_reg0 = readRegister(CONFIG_REG0_ADDRESS);
    this->m_config_reg1 = readRegister(CONFIG_REG1_ADDRESS);
    this->m_config_reg2 = readRegister(CONFIG_REG2_ADDRESS);
    this->m_config_reg3 = readRegister(CONFIG_REG3_ADDRESS);

    config_Buff[0] = this->m_config_reg0;
    config_Buff[1] = this->m_config_reg1;
    config_Buff[2] = this->m_config_reg2;
    config_Buff[3] = this->m_config_reg3;
    return config_Buff;
}

bool Protocentral_ADS1220::WaitForData(unsigned int timeout_ms) {
    unsigned int waited = 0;
    while (true) {
        int val = digitalRead(this->m_drdy_pin);
        if (val == LOW) return true;
        if (timeout_ms == 0) return false;
        usleep(1000); // 1 ms
        waited++;
        if (waited >= timeout_ms) return false;
    }
    return false;
}

uint8_t* Protocentral_ADS1220::Read_Data(void) {
    uint8_t tx[3] = {0x00, 0x00, 0x00};
    uint8_t rx[3] = {0, 0, 0};

    cs_drive(false);
    bool ok = spi_transfer_ioctl(this->spi_fd, tx, rx, 3);
    cs_drive(true);
    (void)ok;

    this->DataReg[0] = rx[0];
    this->DataReg[1] = rx[1];
    this->DataReg[2] = rx[2];
    return this->DataReg;
}

int32_t Protocentral_ADS1220::DataToInt() {
    int32_t result = 0;
    result = (int32_t)this->DataReg[0];
    result = (result << 8) | this->DataReg[1];
    result = (result << 8) | this->DataReg[2];

    if (this->DataReg[0] & (1 << 7)) {
        result |= 0xFF000000;
    }
    return result;
}

int32_t Protocentral_ADS1220::Read_WaitForData() {
    if (!WaitForData(60)) {
        return 0;
    }
    Read_Data();
    return DataToInt();
}

int32_t Protocentral_ADS1220::Read_Data_Samples() {
    uint8_t tx[3] = { SPI_MASTER_DUMMY, SPI_MASTER_DUMMY, SPI_MASTER_DUMMY };
    uint8_t rx[3] = {0, 0, 0};
    cs_drive(false);
    spi_transfer_ioctl(this->spi_fd, tx, rx, 3);
    cs_drive(true);

    long int bit24 = rx[0];
    bit24 = (bit24 << 8) | rx[1];
    bit24 = (bit24 << 8) | rx[2];
    bit24 = (bit24 << 8);
    int32_t mResult32 = (bit24 >> 8);
    return mResult32;
}

int32_t Protocentral_ADS1220::Read_SingleShot_WaitForData(void) {
    Start_Conv();
    return Read_WaitForData();
}

int32_t Protocentral_ADS1220::Read_SingleShot_SingleEnded_WaitForData(uint8_t channel_no) {
    select_mux_channels(channel_no);
    return Read_SingleShot_WaitForData();
}

#define VREF_MASK ((1 << 6) | (1 << 7))
#define VREF_INT (0 << 6)
#define VREF_EXT (1 << 6)

void Protocentral_ADS1220::internal_reference() {
    this->m_config_reg2 &= ~VREF_MASK;
    this->m_config_reg2 |= VREF_INT;
    writeRegister(CONFIG_REG2_ADDRESS, this->m_config_reg2);
}

void Protocentral_ADS1220::external_reference() {
    this->m_config_reg2 &= ~VREF_MASK;
    this->m_config_reg2 |= VREF_EXT;
    writeRegister(CONFIG_REG2_ADDRESS, this->m_config_reg2);
}
