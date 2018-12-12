#ifndef SX127X_H
#define SX127X_H

#include <stdint.h>

#define SX127X_OSC_FREQ_HZ                               32000000

// Register names (LoRa Mode, from table 85)
#define SX127X_REG_00_FIFO                                0x00
#define SX127X_REG_01_OP_MODE                             0x01
#define SX127X_REG_06_FRF_MSB                             0x06
#define SX127X_REG_07_FRF_MID                             0x07
#define SX127X_REG_08_FRF_LSB                             0x08
#define SX127X_REG_09_PA_CONFIG                           0x09
#define SX127X_REG_0A_PA_RAMP                             0x0a
#define SX127X_REG_0B_OCP                                 0x0b
#define SX127X_REG_0C_LNA                                 0x0c
#define SX127X_REG_0D_FIFO_ADDR_PTR                       0x0d
#define SX127X_REG_0E_FIFO_TX_BASE_ADDR                   0x0e
#define SX127X_REG_0F_FIFO_RX_BASE_ADDR                   0x0f
#define SX127X_REG_10_FIFO_RX_CURRENT_ADDR                0x10
#define SX127X_REG_11_IRQ_FLAGS_MASK                      0x11
#define SX127X_REG_12_IRQ_FLAGS                           0x12
#define SX127X_REG_13_RX_NB_BYTES                         0x13
#define SX127X_REG_14_RX_HEADER_CNT_VALUE_MSB             0x14
#define SX127X_REG_15_RX_HEADER_CNT_VALUE_LSB             0x15
#define SX127X_REG_16_RX_PACKET_CNT_VALUE_MSB             0x16
#define SX127X_REG_17_RX_PACKET_CNT_VALUE_LSB             0x17
#define SX127X_REG_18_MODEM_STAT                          0x18
#define SX127X_REG_19_PKT_SNR_VALUE                       0x19
#define SX127X_REG_1A_PKT_RSSI_VALUE                      0x1a
#define SX127X_REG_1B_RSSI_VALUE                          0x1b
#define SX127X_REG_1C_HOP_CHANNEL                         0x1c
#define SX127X_REG_1D_MODEM_CONFIG1                       0x1d
#define SX127X_REG_1E_MODEM_CONFIG2                       0x1e
#define SX127X_REG_1F_SYMB_TIMEOUT_LSB                    0x1f
#define SX127X_REG_20_PREAMBLE_MSB                        0x20
#define SX127X_REG_21_PREAMBLE_LSB                        0x21
#define SX127X_REG_22_PAYLOAD_LENGTH                      0x22
#define SX127X_REG_23_MAX_PAYLOAD_LENGTH                  0x23
#define SX127X_REG_24_HOP_PERIOD                          0x24
#define SX127X_REG_25_FIFO_RX_BYTE_ADDR                   0x25
#define SX127X_REG_26_MODEM_CONFIG3                       0x26
#define SX127X_REG_2C_RSSI_WIDEBAND                       0x2C

#define SX127X_REG_40_DIO_MAPPING1                        0x40
#define SX127X_REG_41_DIO_MAPPING2                        0x41
#define SX127X_REG_42_VERSION                             0x42

#define SX127X_REG_4B_TCXO                                0x4b
#define SX127X_REG_4D_PA_DAC                              0x4d
#define SX127X_REG_5B_FORMER_TEMP                         0x5b
#define SX127X_REG_61_AGC_REF                             0x61
#define SX127X_REG_62_AGC_THRESH1                         0x62
#define SX127X_REG_63_AGC_THRESH2                         0x63
#define SX127X_REG_64_AGC_THRESH3                         0x64

// SX127X_REG_01_OP_MODE                             0x01
#define SX127X_LONG_RANGE_MODE                       0x80
#define SX127X_ACCESS_SHARED_REG                     0x40
#define SX127X_LOW_FREQUENCY_MODE_ON                 0x08
#define SX127X_MODE                                  0x07
#define SX127X_MODE_SLEEP                            0x00
#define SX127X_MODE_STDBY                            0x01
#define SX127X_MODE_FSTX                             0x02
#define SX127X_MODE_TX                               0x03
#define SX127X_MODE_FSRX                             0x04
#define SX127X_MODE_RXCONTINUOUS                     0x05
#define SX127X_MODE_RXSINGLE                         0x06
#define SX127X_MODE_CAD                              0x07

// SX127X_REG_09_PA_CONFIG                           0x09
#define SX127X_PA_SELECT                             0x80
#define SX127X_MAX_POWER                             0x70
#define SX127X_OUTPUT_POWER                          0x0f

// SX127X_REG_0A_PA_RAMP                             0x0a
#define SX127X_LOW_PN_TX_PLL_OFF                     0x10
#define SX127X_PA_RAMP                               0x0f
#define SX127X_PA_RAMP_3_4MS                         0x00
#define SX127X_PA_RAMP_2MS                           0x01
#define SX127X_PA_RAMP_1MS                           0x02
#define SX127X_PA_RAMP_500US                         0x03
#define SX127X_PA_RAMP_250US                         0x0
#define SX127X_PA_RAMP_125US                         0x05
#define SX127X_PA_RAMP_100US                         0x06
#define SX127X_PA_RAMP_62US                          0x07
#define SX127X_PA_RAMP_50US                          0x08
#define SX127X_PA_RAMP_40US                          0x09
#define SX127X_PA_RAMP_31US                          0x0a
#define SX127X_PA_RAMP_25US                          0x0b
#define SX127X_PA_RAMP_20US                          0x0c
#define SX127X_PA_RAMP_15US                          0x0d
#define SX127X_PA_RAMP_12US                          0x0e
#define SX127X_PA_RAMP_10US                          0x0f

// SX127X_REG_0B_OCP                                 0x0b
#define SX127X_OCP_ON                                0x20
#define SX127X_OCP_TRIM                              0x1f

// SX127X_REG_0C_LNA                                 0x0c
#define SX127X_LNA_GAIN                              0xe0
#define SX127X_LNA_BOOST                             0x03
#define SX127X_LNA_BOOST_DEFAULT                     0x00
#define SX127X_LNA_BOOST_150PC                       0x11

// SX127X_REG_11_IRQ_FLAGS_MASK                      0x11
#define SX127X_RX_TIMEOUT_MASK                       0x80
#define SX127X_RX_DONE_MASK                          0x40
#define SX127X_PAYLOAD_CRC_ERROR_MASK                0x20
#define SX127X_VALID_HEADER_MASK                     0x10
#define SX127X_TX_DONE_MASK                          0x08
#define SX127X_CAD_DONE_MASK                         0x04
#define SX127X_FHSS_CHANGE_CHANNEL_MASK              0x02
#define SX127X_CAD_DETECTED_MASK                     0x01

// SX127X_REG_12_IRQ_FLAGS                           0x12
#define SX127X_RX_TIMEOUT                            0x80
#define SX127X_RX_DONE                               0x40
#define SX127X_PAYLOAD_CRC_ERROR                     0x20
#define SX127X_VALID_HEADER                          0x10
#define SX127X_TX_DONE                               0x08
#define SX127X_CAD_DONE                              0x04
#define SX127X_FHSS_CHANGE_CHANNEL                   0x02
#define SX127X_CAD_DETECTED                          0x01

// SX127X_REG_18_MODEM_STAT                          0x18
#define SX127X_RX_CODING_RATE                        0xe0
#define SX127X_MODEM_STATUS                          0x1f
#define SX127X_MODEM_STATUS_CLEAR                    0x10
#define SX127X_MODEM_STATUS_HEADER_INFO_VALID        0x08
#define SX127X_MODEM_STATUS_RX_ONGOING               0x04
#define SX127X_MODEM_STATUS_SIGNAL_SYNCHRONIZED      0x02
#define SX127X_MODEM_STATUS_SIGNAL_DETECTED          0x01

// SX127X_REG_1C_HOP_CHANNEL                         0x1c
#define SX127X_PLL_TIMEOUT                           0x80
#define SX127X_RX_PAYLOAD_CRC_IS_ON                  0x40
#define SX127X_FHSS_PRESENT_CHANNEL                  0x3f

// SX127X_REG_1D_MODEM_CONFIG1                       0x1d
#define SX127X_BW                                    0xf0
#define SX127X_BW_7_8KHZ                             0x00
#define SX127X_BW_10_4KHZ                            0x10
#define SX127X_BW_15_6KHZ                            0x20
#define SX127X_BW_20_8KHZ                            0x30
#define SX127X_BW_31_25KHZ                           0x40
#define SX127X_BW_41_7KHZ                            0x50
#define SX127X_BW_62_5KHZ                            0x60
#define SX127X_BW_125KHZ                             0x70
#define SX127X_BW_250KHZ                             0x80
#define SX127X_BW_500KHZ                             0x90
#define SX127X_CODING_RATE                           0x0E
#define SX127X_CODING_RATE_4_5                       0x02
#define SX127X_CODING_RATE_4_6                       0x04
#define SX127X_CODING_RATE_4_7                       0x06
#define SX127X_CODING_RATE_4_8                       0x08
#define SX127X_IMPLICIT_HEADER_MODE_ON               0x01

// SX127X_REG_1E_MODEM_CONFIG2                       0x1e
#define SX127X_SPREADING_FACTOR                      0xf0
#define SX127X_SPREADING_FACTOR_64CPS                0x60
#define SX127X_SPREADING_FACTOR_128CPS               0x70
#define SX127X_SPREADING_FACTOR_256CPS               0x80
#define SX127X_SPREADING_FACTOR_512CPS               0x90
#define SX127X_SPREADING_FACTOR_1024CPS              0xa0
#define SX127X_SPREADING_FACTOR_2048CPS              0xb0
#define SX127X_SPREADING_FACTOR_4096CPS              0xc0
#define SX127X_TX_CONTINUOUS_MODE                    0x08
#define SX127X_RX_PAYLOAD_CRC_ON                     0x04
#define SX127X_SYM_TIMEOUT_MSB                       0x03

// SX127X_REG_26_MODEM_CONFIG3                       0x26
#define SX127X_LOW_DATA_RATE_OPTIMIZE                0x08
#define SX127X_AGC_AUTO_ON                           0x04


// SX127X_REG_4D_PA_DAC                              0x4d
#define SX127X_PA_DAC_DISABLE                        0x04
#define SX127X_PA_DAC_ENABLE                         0x07

enum Sx127x_BW {
    Sx127x_BW_7_8_KHZ = 0x00,
    Sx127x_BW_10_4_KHZ = 0x01,
    Sx127x_BW_15_6_KHZ = 0x02,
    Sx127x_BW_20_8_KHZ = 0x03,
    Sx127x_BW_31_25_KHZ = 0x04,
    Sx127x_BW_41_7_KHZ = 0x05,
    Sx127x_BW_62_5_KHZ = 0x06,
    Sx127x_BW_125_KHZ = 0x07,
    Sx127x_BW_250_KHZ = 0x08,
    Sx127x_BW_500_KHZ = 0x09,
};

enum Sx127x_CR {
    Sx127x_CR_4_5 = 0x01,
    Sx127x_CR_4_6 = 0x02,
    Sx127x_CR_4_7 = 0x03,
    Sx127x_CR_4_8 = 0x04,
};

enum Sx127x_SF {
    Sx127x_SF_64 = 0x06,
    Sx127x_SF_128 = 0x07,
    Sx127x_SF_256 = 0x08,
    Sx127x_SF_512 = 0x09,
    Sx127x_SF_1024 = 0x0A,
    Sx127x_SF_2048 = 0x0B,
    Sx127x_SF_4096 = 0x0C,
};

template <class SpiCtrlTempl, class LogTempl>
class Sx127x
{
public:
    enum Sx127x_Result {Sx127x_OK, Sx127x_BUSY, Sx127x_WRONGMODE};

    Sx127x(SpiCtrlTempl& spi, LogTempl& log);
    void init(uint32_t freq, Sx127x_BW bw, Sx127x_SF sf, Sx127x_CR cr, uint8_t outPower);
    Sx127x_Result send(uint8_t* msg, uint32_t len);
    Sx127x_Result startRx(void);
    Sx127x_Result stopRx(void);
    uint8_t getStatus(void);
    uint8_t getMode(void);
    uint8_t getFlags(void);
    void clearFlags(void);
    void clearFlags(uint8_t flags);
    void getData(uint8_t* data, uint8_t* len);
    uint8_t getRssi(void);
    uint8_t getCurrentRssi(void);
    int8_t getSnr(void);
    void printRegisters(void);
    uint8_t getRand30(void);
    uint8_t getRand(void);

private:
    bool isBusy(void);

    uint8_t lowFreqMode;

    uint32_t decodeFrf(uint32_t freq);
    uint8_t decodeBw(uint32_t bw);
    uint8_t decodeSf(uint32_t sf);
    uint8_t decodeCr(uint32_t cr);

    SpiCtrlTempl& spi;
    LogTempl& log;
};

template <class SpiCtrlTempl, class LogTempl>
Sx127x<SpiCtrlTempl, LogTempl>::Sx127x(SpiCtrlTempl& spi, LogTempl& log) : spi(spi), log(log)
{
}

template <class SpiCtrlTempl, class LogTempl>
void Sx127x<SpiCtrlTempl, LogTempl>::init(uint32_t freq, Sx127x_BW bw, Sx127x_SF sf, Sx127x_CR cr, uint8_t outPower)
{
    this->spi.init();

    this->lowFreqMode = SX127X_LOW_FREQUENCY_MODE_ON;
    this->spi.writeReg(SX127X_REG_01_OP_MODE, SX127X_MODE_SLEEP | this->lowFreqMode);
    this->spi.writeReg(SX127X_REG_0B_OCP, SX127X_OCP_ON | 0x0B); //Imax = 100mA

    uint32_t frf = this->decodeFrf(freq);
    this->spi.writeReg(SX127X_REG_06_FRF_MSB, (frf >> 16) & 0xFF);
    this->spi.writeReg(SX127X_REG_07_FRF_MID, (frf >> 8) & 0xFF);
    this->spi.writeReg(SX127X_REG_08_FRF_LSB, frf & 0xFF);

    this->spi.writeReg(SX127X_REG_01_OP_MODE, SX127X_MODE_SLEEP | this->lowFreqMode);
    this->spi.writeReg(SX127X_REG_01_OP_MODE, SX127X_MODE_SLEEP | this->lowFreqMode | SX127X_LONG_RANGE_MODE);

    uint8_t config1 = this->decodeBw(bw) | this->decodeCr(cr);
    this->spi.writeReg(SX127X_REG_1D_MODEM_CONFIG1, config1);

    uint8_t config2 = this->decodeSf(sf) | SX127X_RX_PAYLOAD_CRC_ON | SX127X_SYM_TIMEOUT_MSB;
    this->spi.writeReg(SX127X_REG_1E_MODEM_CONFIG2, config2);

    float ts = 1000 * (float)sf / (float)bw; //Symbol time [ms]
    if (ts > 16) {
        uint8_t config3 = SX127X_LOW_DATA_RATE_OPTIMIZE;
        this->spi.writeReg(SX127X_REG_26_MODEM_CONFIG3, config3);
    }

    uint8_t paConfig = SX127X_PA_SELECT | SX127X_MAX_POWER | ((outPower - 2) & SX127X_OUTPUT_POWER);

    this->spi.writeReg(SX127X_REG_21_PREAMBLE_LSB, 16);

    this->spi.writeReg(SX127X_REG_09_PA_CONFIG, paConfig);
}

template <class SpiCtrlTempl, class LogTempl>
typename Sx127x<SpiCtrlTempl, LogTempl>::Sx127x_Result Sx127x<SpiCtrlTempl, LogTempl>::send(uint8_t* msg, uint32_t len)
{
    if (this->isBusy())
        return Sx127x_BUSY;

    this->spi.writeReg(SX127X_REG_01_OP_MODE, SX127X_MODE_STDBY | this->lowFreqMode);
    this->spi.writeReg(SX127X_REG_22_PAYLOAD_LENGTH, len);
    this->spi.writeReg(SX127X_REG_0E_FIFO_TX_BASE_ADDR, 0x00);
    this->spi.writeReg(SX127X_REG_0D_FIFO_ADDR_PTR, 0x00);
    this->spi.writeReg(SX127X_REG_00_FIFO, msg, len);
    this->spi.writeReg(SX127X_REG_01_OP_MODE, SX127X_MODE_TX | this->lowFreqMode | SX127X_LONG_RANGE_MODE);

    return Sx127x_OK;
}

template <class SpiCtrlTempl, class LogTempl>
typename Sx127x<SpiCtrlTempl, LogTempl>::Sx127x_Result Sx127x<SpiCtrlTempl, LogTempl>::startRx(void)
{
    if (this->isBusy())
        return Sx127x_BUSY;

    this->spi.writeReg(SX127X_REG_01_OP_MODE, SX127X_MODE_STDBY | this->lowFreqMode);
    this->spi.writeReg(SX127X_REG_0F_FIFO_RX_BASE_ADDR, 0x00);
    this->spi.writeReg(SX127X_REG_01_OP_MODE, SX127X_MODE_RXCONTINUOUS | this->lowFreqMode | SX127X_LONG_RANGE_MODE);

    return Sx127x_OK;
}

template <class SpiCtrlTempl, class LogTempl>
typename Sx127x<SpiCtrlTempl, LogTempl>::Sx127x_Result Sx127x<SpiCtrlTempl, LogTempl>::stopRx(void)
{
    this->spi.writeReg(SX127X_REG_01_OP_MODE, SX127X_MODE_STDBY | this->lowFreqMode | SX127X_LONG_RANGE_MODE);;
    return Sx127x_OK;
}

template <class SpiCtrlTempl, class LogTempl>
uint8_t Sx127x<SpiCtrlTempl, LogTempl>::getStatus(void)
{
    return this->spi.readReg(SX127X_REG_18_MODEM_STAT);
}

template <class SpiCtrlTempl, class LogTempl>
uint8_t Sx127x<SpiCtrlTempl, LogTempl>::getMode(void)
{
    return this->spi.readReg(SX127X_REG_01_OP_MODE);
}

template <class SpiCtrlTempl, class LogTempl>
uint8_t Sx127x<SpiCtrlTempl, LogTempl>::getFlags(void)
{
    return this->spi.readReg(SX127X_REG_12_IRQ_FLAGS);
}

template <class SpiCtrlTempl, class LogTempl>
void Sx127x<SpiCtrlTempl, LogTempl>::clearFlags(void)
{
    this->spi.writeReg(SX127X_REG_12_IRQ_FLAGS, 0xFF);
}

template <class SpiCtrlTempl, class LogTempl>
void Sx127x<SpiCtrlTempl, LogTempl>::clearFlags(uint8_t flags)
{
    this->spi.writeReg(SX127X_REG_12_IRQ_FLAGS, flags);
}

template <class SpiCtrlTempl, class LogTempl>
void Sx127x<SpiCtrlTempl, LogTempl>::getData(uint8_t* data, uint8_t* len)
{
    *len = this->spi.readReg(SX127X_REG_25_FIFO_RX_BYTE_ADDR);
    this->spi.writeReg(SX127X_REG_0D_FIFO_ADDR_PTR, 0x00);
    for (uint8_t i = 0; i < *len; i++)
        data[i] = this->spi.readReg(SX127X_REG_00_FIFO);
}

template <class SpiCtrlTempl, class LogTempl>
uint8_t Sx127x<SpiCtrlTempl, LogTempl>::getRssi(void)
{
    return this->spi.readReg(SX127X_REG_1A_PKT_RSSI_VALUE);
}

template <class SpiCtrlTempl, class LogTempl>
uint8_t Sx127x<SpiCtrlTempl, LogTempl>::getCurrentRssi(void)
{
    return this->spi.readReg(SX127X_REG_1B_RSSI_VALUE);
}

template <class SpiCtrlTempl, class LogTempl>
int8_t Sx127x<SpiCtrlTempl, LogTempl>::getSnr(void)
{
    return (int8_t)this->spi.readReg(SX127X_REG_19_PKT_SNR_VALUE);
}

template <class SpiCtrlTempl, class LogTempl>
void Sx127x<SpiCtrlTempl, LogTempl>::printRegisters(void)
{
    this->log.printWithNewline("");
    this->log.printWithNewline("----------------------------------------------------");
    this->log.printWithNewline("   | 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F");
    this->log.printWithNewline("----------------------------------------------------");
    for(uint32_t i=0; i<0x80; i+=0x10) {
        this->log.printAsHex((uint8_t)i);
        this->log.print(" |");
        for(uint32_t j=0; j<0x10; j+=0x01) {
            this->log.print(" ");
            this->log.printAsHex(this->spi.readReg((uint8_t)(i | j)));
        }
        this->log.printWithNewline("");
    }
    this->log.printWithNewline("----------------------------------------------------");
    this->log.printWithNewline("");
}

template <class SpiCtrlTempl, class LogTempl>
uint8_t Sx127x<SpiCtrlTempl, LogTempl>::getRand30(void)
{
    return this->spi.readReg(SX127X_REG_2C_RSSI_WIDEBAND) % 30;
}

template <class SpiCtrlTempl, class LogTempl>
uint8_t Sx127x<SpiCtrlTempl, LogTempl>::getRand(void)
{
    return this->spi.readReg(SX127X_REG_2C_RSSI_WIDEBAND);
}

template <class SpiCtrlTempl, class LogTempl>
bool Sx127x<SpiCtrlTempl, LogTempl>::isBusy(void)
{
    uint8_t mode = this->spi.readReg(SX127X_REG_01_OP_MODE);
    if ((mode & SX127X_MODE) == SX127X_MODE_TX)
        return true;
    else
        return false;

}

template <class SpiCtrlTempl, class LogTempl>
uint32_t Sx127x<SpiCtrlTempl, LogTempl>::decodeFrf(uint32_t freq)
{
    return (uint32_t)((uint64_t)freq * (uint64_t)524288 / (uint64_t)SX127X_OSC_FREQ_HZ);
}

template <class SpiCtrlTempl, class LogTempl>
uint8_t Sx127x<SpiCtrlTempl, LogTempl>::decodeBw(uint32_t bw)
{
    switch(bw) {
    case Sx127x_BW_7_8_KHZ:
        return SX127X_BW_7_8KHZ;
    case Sx127x_BW_10_4_KHZ:
        return SX127X_BW_10_4KHZ;
    case Sx127x_BW_15_6_KHZ:
        return SX127X_BW_15_6KHZ;
    case Sx127x_BW_20_8_KHZ:
        return SX127X_BW_20_8KHZ;
    case Sx127x_BW_31_25_KHZ:
        return SX127X_BW_31_25KHZ;
    case Sx127x_BW_41_7_KHZ:
        return SX127X_BW_41_7KHZ;
    case Sx127x_BW_62_5_KHZ:
        return SX127X_BW_62_5KHZ;
    case Sx127x_BW_125_KHZ:
        return SX127X_BW_125KHZ;
    case Sx127x_BW_250_KHZ:
        return SX127X_BW_250KHZ;
    case Sx127x_BW_500_KHZ:
        return SX127X_BW_500KHZ;
    default:
        return SX127X_BW_41_7KHZ;
    }
}

template <class SpiCtrlTempl, class LogTempl>
uint8_t Sx127x<SpiCtrlTempl, LogTempl>::decodeSf(uint32_t sf)
{
    switch(sf) {
    case Sx127x_SF_64:
        return SX127X_SPREADING_FACTOR_64CPS;
    case Sx127x_SF_128:
        return SX127X_SPREADING_FACTOR_128CPS;
    case Sx127x_SF_256:
        return SX127X_SPREADING_FACTOR_256CPS;
    case Sx127x_SF_512:
        return SX127X_SPREADING_FACTOR_512CPS;
    case Sx127x_SF_1024:
        return SX127X_SPREADING_FACTOR_1024CPS;
    case Sx127x_SF_2048:
        return SX127X_SPREADING_FACTOR_2048CPS;
    case Sx127x_SF_4096:
        return SX127X_SPREADING_FACTOR_4096CPS;
    default:
        return SX127X_SPREADING_FACTOR_1024CPS;
    }
}

template <class SpiCtrlTempl, class LogTempl>
uint8_t Sx127x<SpiCtrlTempl, LogTempl>::decodeCr(uint32_t cr)
{
    switch(cr) {
    case Sx127x_CR_4_5:
        return SX127X_CODING_RATE_4_5;
    case Sx127x_CR_4_6:
        return SX127X_CODING_RATE_4_6;
    case Sx127x_CR_4_7:
        return SX127X_CODING_RATE_4_7;
    case Sx127x_CR_4_8:
        return SX127X_CODING_RATE_4_8;
    default:
        return SX127X_CODING_RATE_4_5;
    }
}

#endif
