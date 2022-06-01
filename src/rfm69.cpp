/******************************************************************
 * RFM69 radio management
 ******************************************************************/

#include "rover.h"
#include <RH_RF69.h>

extern StatusDisplay *statusDisplay;

// global for keeping signal strength of last received message
int16_t lastSignalStrength = 0;

#ifdef USE_ENCRYPTION
#include "encryption_key.h"
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(11, 19);

void setupRfm69() {
    // initialize RFM69
    debug("initializing RFM69");
    if (!rf69.init())
        debug("init failed");
    if (!rf69.setFrequency(915))
        debug("setFrequency failed");
    if (!rf69.setModemConfig(rf69.FSK_Rb9_6Fd19_2)) // FSK, Whitening, Rb = 9.6kbs,  Fd = 19.2kHz
        debug("setModemConfig failed");
#ifdef USE_ENCRYPTION
    rf69.setSyncWords(syncWords, 2);  // must define in encryption_key.h, 2-byte array
    rf69.setEncryptionKey(encryptionKey); // must define in encryption_key.h, 16-byte array
#endif
    rf69.setTxPower(17, true);
    // RadioHead seems to never set RssiThresh, and the power-on value is not the value
    // recommended in the docs. Use the low-level spi method to set it here.
    rf69.spiWrite(RH_RF69_REG_29_RSSITHRESH, 233); // -116.5 dB, this is the recommended value
#ifdef USB_DEBUG
    uint16_t version = rf69.deviceType();
    debug(String("RFM69 initialized: version 0x" + String(version, HEX)));
    //rf69.printRegisters();
#endif
}

void updateSignalStrength() {
    lastSignalStrength = rf69.lastRssi();
    statusDisplay->setRssi(lastSignalStrength);
}
