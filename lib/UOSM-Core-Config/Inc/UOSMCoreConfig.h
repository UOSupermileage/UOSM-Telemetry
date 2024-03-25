#define MAX_SERIAL_PRINT_LENGTH 128

#ifdef __cplusplus

extern "C" {
#endif

#include <stdint.h>
#include <stdarg.h>
#ifdef MBED
#endif

void GPIO_DigitalWrite(uint8_t pin, uint8_t val);
void SPI_Transfer(void * data, uint32_t size);
void ExternalSerialPrint(const char * message, ...);
void ExternalSerialPrintln(const char * message, ...);
void SPI_CS_HIGH();
void SPI_CS_LOW();

#ifdef __cplusplus
}
#endif

#define MCP2515_CS_PIN 7
#define MCP2515_CS_HIGH()       SPI_CS_HIGH()
#define MCP2515_CS_LOW()        SPI_CS_LOW()

//#define EXT_SPI_CAN             hspi2
//#define SPI_CAN                 &hspi2
//#define SPI_TIMEOUT             10

#define MCP2515_SPI_TRANSMIT(BUFFER, SIZE)  SPI_Transfer(BUFFER, SIZE)
#define MCP2515_SPI_RECEIVE(BUFFER, SIZE)   SPI_Transfer(BUFFER, SIZE)
#define MCP2515_SPI_READY true
//
//#define MCP2515_SPI_TRANSMIT(BUFFER, SIZE)    HAL_SPI_Transmit(SPI_CAN, BUFFER, SIZE, SPI_TIMEOUT);
//#define MCP2515_SPI_RECEIVE(BUFFER, SIZE)     HAL_SPI_Receive(SPI_CAN, BUFFER, SIZE, SPI_TIMEOUT);
//#define MCP2515_SPI_READY                     HAL_SPI_GetState(SPI_CAN) == HAL_SPI_STATE_READY