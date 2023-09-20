#define MAX_SERIAL_PRINT_LENGTH 128

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdarg.h>

void GPIO_DigitalWrite(uint8_t pin, uint8_t val);
void SPI_Transfer(void * data, uint32_t size);
void ExternalSerialPrint(const char * message, ...);
void ExternalSerialPrintln(const char * message, ...);

#ifdef __cplusplus
}
#endif

#define MCP2515_CS_PIN 15
#define MCP2515_CS_HIGH()       GPIO_DigitalWrite(MCP2515_CS_PIN, 1)
#define MCP2515_CS_LOW()        GPIO_DigitalWrite(MCP2515_CS_PIN, 0)

#define MCP2515_SPI_TRANSMIT(BUFFER, SIZE)  SPI_Transfer(BUFFER, SIZE)
#define MCP2515_SPI_RECEIVE(BUFFER, SIZE)   SPI_Transfer(BUFFER, SIZE)
#define MCP2515_SPI_READY true