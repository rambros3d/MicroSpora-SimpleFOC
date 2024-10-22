#define PHA_H PA10
#define PHB_H PA9
#define PHC_H PA8

#define PHA_L PB15
#define PHB_L PB14
#define PHC_L PB13

#define ISENSA PA0  // Phase A current sense
#define ISENSB PA1  // Phase B current sense
#define ISENSC PA2  // Phase C current sense

#define LED_BUILTIN PC6

#define SPI_MOSI PB5  // SPI MOSI
#define SPI_MISO PB4  // SPI MISO
#define SPI_SCK PB3   // SPI Clock
#define SPI_SS PC4    // SPI Chip Select (SS)
#define DRV_CS SPI_SS

#define ENC_SDO PA6  // configured as MISO
#define ENC_NC PA7   // Not connected (dummy MOSI)
#define ENC_CLK PA5
#define ENC_CS PA4

#define CAN_RX PB8       // CAN Receive Pin
#define CAN_TX PB9       // CAN Transmit Pin
#define CAN_ENABLE PC13  // CAN Transmit Pin

#define VSENS PB11   // Voltage sense pin
#define VSCALE 11.0  // Scaling factor
