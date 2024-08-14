#include "Dw3000Device.h"

#include "esphome/core/log.h"

#include "SPI.h"

#include "dw3000.h"

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements */
extern dwt_txconfig_t txconfig_options; // from dw3000_config_options.cpp
extern SPISettings _fastSPI; // from dw3000_port.cpp

namespace esphome {
namespace uwb {

#define SPI_SPEED_HZ 16000000
#define PIN_RST 27  // ----> DWM3120 RST
#define PIN_IRQ 34  // ----> DWM3120 IRQ
#define PIN_SS   4  // ----> DWM3120 SPICSN

// TX power control, see DW3000 User Manual Table 7: TX power recommendation values
#define TX_PWR_CH5_DEFAULTS (0xFDFDFDFD) // Channel 5 power defaults
#define TX_PWR_CH9_DEFAULTS (0xFEFEFEFE) // Channel 9 power defaults
// See DW3000 User Manual 8.2.2.21 Sub-register 0x01:0C – Transmit power control
// each byte: 64 fine control steps (bits 7-2) and 4 coarse control steps (bits 1-0)
// The fine control steps provide approximately 25 dB of control, while the coarse steps provide an additional 7 dB of control
// Channel 9: the maximum coarse gain setting is 2, and the setting 3 should not be used.
// Channel 5: the recommended coarse gain setting is 2. There is only a marginal increase in TX power using coarse gain setting of 3,
//            but there is a relatively large increase in current
// in general, the power settings for all parts of the packet should be programmed to the same value
#define DATA_PWR (0xFF) // bits 7-0, power level applied during Data portion of frame
#define PHR_PWR  (0xFC) // bits 15-8, PHY header (PHR), if the DATA_PWR is set to 0xFF, it is recommended to set PHR_PWR to 0xFC.
#define SHR_PWR  (0xFF) // bits 23-16, Synchronisation Header (SHR)
#define STS_PWR  (0xFF) // bits 31-24, Scrambled Timestamp Sequence (STS)

/* Default antenna delay values for 64 MHz PRF */
#define TX_ANT_DLY  (16385) // Decawave default was confirmed to be good (by ranging with 8.0m distance)
#define RX_ANT_DLY  TX_ANT_DLY // for simplicity: RX = TX antenna delay

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SFD type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};


Dw3000Device::Dw3000Device() {}

void Dw3000Device::setup() {
    ESP_LOGD(TAG, "setup");

    /* Configure SPI rate, DW3000 supports up to 36 MHz */
    _fastSPI = SPISettings(SPI_SPEED_HZ, MSBFIRST, SPI_MODE0);
    port_set_dw_ic_spi_fastrate(PIN_IRQ, PIN_RST, PIN_SS);

    delay(2U); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ {
        delay(2U);
     };

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        ESP_LOGE(TAG, "dwt_initialise FAIL");
    } else {
        ESP_LOGD(TAG, "dwt_initialise OK");
    }

    /* Configure DW IC */
    /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    if (dwt_configure(&config) == DWT_ERROR) {
        ESP_LOGE(TAG, "dwt_configure FAIL");
    }
     else {
        ESP_LOGD(TAG, "dwt_configure OK");
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    txconfig_options.power = STS_PWR << 24 | SHR_PWR << 16 | PHR_PWR << 8 | DATA_PWR;
    dwt_configuretxrf(&txconfig_options);

    /* Apply default antenna delay value. */
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setrxantennadelay(RX_ANT_DLY);

    ESP_LOGD(TAG, "setup done");
}

void Dw3000Device::loop() {

}

}  // namespace uwb
}  // namespace esphome
