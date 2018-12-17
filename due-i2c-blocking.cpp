// I2C routines without interrupt/DMA (blocking!) for ATSAM3X8E
// $Id$

#include <due-i2c-blocking.h>
#include <omower-defs.h>

uint16_t i2cWriteOne(uint8_t bus, uint8_t device, uint8_t address, uint8_t data) {
  uint8_t buf[1];

  buf[0] = data;
  return i2cWrite(bus, device, address, buf, 1);
} // uint16_t i2cWriteOne(uint8_t bus, uint8_t device, uint8_t address, uint8_t data)

uint16_t i2cWriteOneNa(uint8_t bus, uint8_t device, uint8_t data) {
  uint8_t buf[1];

  buf[0] = data;
  return i2cWriteNa(bus, device, buf, 1);
} // uint16_t i2cWriteOneNa(uint8_t bus, uint8_t device, uint8_t data)

// Write a data to i2c
uint16_t i2cWriteEx(uint8_t bus, uint8_t device, uint8_t address, boolean needAddress,
                    uint8_t *data, uint16_t count) {
  unsigned int status;
  unsigned int timeoutCounter;

  if (bus) {
    // reset status register
    status = WIRE1_INTERFACE->TWI_SR;
    // set device and internal address (1 byte)
    WIRE1_INTERFACE->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;
    WIRE1_INTERFACE->TWI_MMR = 0;
    WIRE1_INTERFACE->TWI_MMR = TWI_MMR_DADR(device) | ((1 << TWI_MMR_IADRSZ_Pos) & TWI_MMR_IADRSZ_Msk);
    if (needAddress) {
      WIRE1_INTERFACE->TWI_IADR = 0;
      WIRE1_INTERFACE->TWI_IADR = address;
    }

    // Send data bytes
    for (uint16_t i = 0; i < count; i++) {
      WIRE1_INTERFACE->TWI_THR = data[i];
      if (i == (count - 1))
        WIRE1_INTERFACE->TWI_CR = TWI_CR_STOP;
      timeoutCounter = 0;
      while (1) {
        status = WIRE1_INTERFACE->TWI_SR;
        if (status & TWI_SR_TXRDY)
          break;
        if (++timeoutCounter > TWI_TIMEOUT_COUNTER) {
          WIRE1_INTERFACE->TWI_CR = TWI_CR_STOP;
          return i;
        }
      }
    }
    timeoutCounter = 0;
    // Wait for transfer complete
    while (1) {
      status = WIRE1_INTERFACE->TWI_SR;
      if (status & TWI_SR_TXCOMP)
        break;
      if (++timeoutCounter > TWI_TIMEOUT_COUNTER)
        return count - 1;
    }
  } else {
    // reset status register
    status = WIRE_INTERFACE->TWI_SR;
    // set device and internal address (1 byte)
    WIRE_INTERFACE->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;
    WIRE_INTERFACE->TWI_MMR = 0;
    WIRE_INTERFACE->TWI_MMR = TWI_MMR_DADR(device) | ((1 << TWI_MMR_IADRSZ_Pos) & TWI_MMR_IADRSZ_Msk);
    if (needAddress) {
      WIRE_INTERFACE->TWI_IADR = 0;
      WIRE_INTERFACE->TWI_IADR = address;
    }

    // Send data bytes
    for (uint16_t i = 0; i < count; i++) {
      WIRE_INTERFACE->TWI_THR = data[i];
      if (i == (count - 1))
        WIRE_INTERFACE->TWI_CR = TWI_CR_STOP;
      timeoutCounter = 0;
      while (1) {
        status = WIRE_INTERFACE->TWI_SR;
        if (status & TWI_SR_TXRDY)
          break;
        if (++timeoutCounter > TWI_TIMEOUT_COUNTER) {
          WIRE_INTERFACE->TWI_CR = TWI_CR_STOP;
          return i;
        }
      }
    }
    timeoutCounter = 0;
    // Wait for transfer complete
    while (1) {
      status = WIRE_INTERFACE->TWI_SR;
      if (status & TWI_SR_TXCOMP)
        break;
      if (++timeoutCounter > TWI_TIMEOUT_COUNTER)
        return count - 1;
    }
  }
  return count;
} // void i2cWriteEx(uint8_t bus, uint8_t device, uint8_t address, boolean needAddress,
  //                 uint8_t *data, uint16_t count);

// Write a data to i2c without internal address
uint16_t i2cWriteNa(uint8_t bus, uint8_t device, uint8_t *data, uint16_t count) {
  i2cWriteEx(bus, device, 0, false, data, count);
} // uint16_t i2cWrite(uint8_t bus, uint8_t device, uint8_t *data, uint16_t count)

// Write data to i2c with internal address
uint16_t i2cWrite(uint8_t bus, uint8_t device, uint8_t address, uint8_t *data, uint16_t count) {
  i2cWriteEx(bus, device, address, true, data, count);
} // uint16_t i2cWrite(uint8_t bus, uint8_t device, uint8_t address, uint8_t *data, uint16_t count)

// Read data from i2c
uint16_t i2cReadEx(uint8_t bus, uint8_t device, uint8_t address, boolean needAddress,
                   uint8_t *data, uint8_t count) {
  unsigned int status;
  unsigned int timeoutCounter;

  if (bus) {
    // reset status register
    status = WIRE1_INTERFACE->TWI_SR;
    // set device and internal address (1 byte)
    WIRE1_INTERFACE->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;
    WIRE1_INTERFACE->TWI_MMR = 0;
    WIRE1_INTERFACE->TWI_MMR = TWI_MMR_DADR(device) | ((1 << TWI_MMR_IADRSZ_Pos) & TWI_MMR_IADRSZ_Msk)
                              | TWI_MMR_MREAD;
    if (needAddress) {
      WIRE1_INTERFACE->TWI_IADR = 0;
      WIRE1_INTERFACE->TWI_IADR = address;
    }

    // Check if we're receiving just one byte
    if (count == 1)
      WIRE1_INTERFACE->TWI_CR = TWI_CR_START | TWI_CR_STOP;
    else
      WIRE1_INTERFACE->TWI_CR = TWI_CR_START;
    // Read data from bus
    for (uint16_t i = 0; i < count; i++) {
      if ((count != 1) && (i == (count - 1)))
        WIRE1_INTERFACE->TWI_CR = TWI_CR_STOP;
      timeoutCounter = 0;
      while (1) {
        status = WIRE1_INTERFACE->TWI_SR;
        if (status & TWI_SR_RXRDY)
          break;

        if (status & (TWI_SR_NACK | TWI_SR_ARBLST | TWI_SR_OVRE)) {
          WIRE1_INTERFACE->TWI_CR = TWI_CR_STOP;
          return i;
        }

        if (++timeoutCounter > TWI_TIMEOUT_COUNTER) {
          WIRE1_INTERFACE->TWI_CR = TWI_CR_STOP;
          return i;
        }
      }
      data[i] = WIRE1_INTERFACE->TWI_RHR;
    }
    timeoutCounter = 0;
    // Wait for transfer complete
    while (1) {
      status = WIRE1_INTERFACE->TWI_SR;
      if (status & TWI_SR_TXCOMP)
        break;
      if (++timeoutCounter > TWI_TIMEOUT_COUNTER)
        return count;
    }
  } else {
    // reset status register
    status = WIRE_INTERFACE->TWI_SR;
    // set device and internal address (1 byte)
    WIRE_INTERFACE->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;
    WIRE_INTERFACE->TWI_MMR = 0;
    WIRE_INTERFACE->TWI_MMR = TWI_MMR_DADR(device) | ((1 << TWI_MMR_IADRSZ_Pos) & TWI_MMR_IADRSZ_Msk)
                              | TWI_MMR_MREAD;
    if (needAddress) {
      WIRE_INTERFACE->TWI_IADR = 0;
      WIRE_INTERFACE->TWI_IADR = address;
    }

    // Check if we're receiving just one byte
    if (count == 1)
      WIRE_INTERFACE->TWI_CR = TWI_CR_START | TWI_CR_STOP;
    else
      WIRE_INTERFACE->TWI_CR = TWI_CR_START;
    // Read data from bus
    for (uint16_t i = 0; i < count; i++) {
      if ((count != 1) && (i == (count - 1)))
        WIRE_INTERFACE->TWI_CR = TWI_CR_STOP;
      timeoutCounter = 0;
      while (1) {
        status = WIRE_INTERFACE->TWI_SR;
        if (status & TWI_SR_RXRDY)
          break;

        if (status & (TWI_SR_NACK | TWI_SR_ARBLST | TWI_SR_OVRE)) {
          WIRE_INTERFACE->TWI_CR = TWI_CR_STOP;
          return i;
        }
        
        if (++timeoutCounter > TWI_TIMEOUT_COUNTER) {
          WIRE_INTERFACE->TWI_CR = TWI_CR_STOP;
          return i;
        }
      }
      data[i] = WIRE_INTERFACE->TWI_RHR;
    }
    timeoutCounter = 0;
    // Wait for transfer complete
    while (1) {
      status = WIRE_INTERFACE->TWI_SR;
      if (status & TWI_SR_TXCOMP)
        break;
      if (++timeoutCounter > TWI_TIMEOUT_COUNTER)
        return count;
    }
  }
  return count;
} // uint16_t i2cReadEx(uint8_t bus, uint8_t device, uint8_t address, boolean needAddress,
  //                    uint8_t *data, uint8_t count)

// Read from i2c with internal address
uint16_t i2cRead(uint8_t bus, uint8_t device, uint8_t address, uint8_t *data, uint8_t count) {
  return i2cReadEx(bus, device, address, true, data, count);
} // uint16_t i2cRead(uint8_t bus, uint8_t device, uint8_t address, uint8_t *data, uint8_t count)

// Read from i2c without internal address
uint16_t i2cReadNa(uint8_t bus, uint8_t device, uint8_t *data, uint8_t count) {
  return i2cReadEx(bus, device, 0, false, data, count);
} // uint16_t i2cReadNa(uint8_t bus, uint8_t device, uint8_t *data, uint8_t count)

// Initialize I2C bus (speed = 100000 or 400000)
void i2cInit(uint8_t bus, uint32_t speed) {
  if (bus) {
    // spam SCL to reset some strange devices
    pinMode(PIN_SCL1, OUTPUT);
    pinMode(PIN_SDA1, OUTPUT);
    digitalWrite(PIN_SDA1, LOW);
    for (int i = 0; i < 256; i++) {
      digitalWrite(PIN_SCL1, HIGH);
      delayMicroseconds(1);
      digitalWrite(PIN_SCL1, LOW);
      delayMicroseconds(1);
    }
    digitalWrite(PIN_SDA1, HIGH);
    for (int i = 0; i < 256; i++) {
      digitalWrite(PIN_SCL1, HIGH);
      delayMicroseconds(1);
      digitalWrite(PIN_SCL1, LOW);
      delayMicroseconds(1);
    }
    pinMode(PIN_SCL1, INPUT_PULLUP);
    pinMode(PIN_SDA1, INPUT_PULLUP);

    pmc_enable_periph_clk(WIRE1_INTERFACE_ID);
    PIO_Configure(g_APinDescription[PIN_WIRE1_SDA].pPort,
                  g_APinDescription[PIN_WIRE1_SDA].ulPinType,
                  g_APinDescription[PIN_WIRE1_SDA].ulPin,
                  g_APinDescription[PIN_WIRE1_SDA].ulPinConfiguration);
    PIO_Configure(g_APinDescription[PIN_WIRE1_SCL].pPort,
                  g_APinDescription[PIN_WIRE1_SCL].ulPinType,
                  g_APinDescription[PIN_WIRE1_SCL].ulPin,
                  g_APinDescription[PIN_WIRE1_SCL].ulPinConfiguration);

    NVIC_DisableIRQ(TWI0_IRQn);
    NVIC_ClearPendingIRQ(TWI0_IRQn);

    // Disable PDC channel
    WIRE1_INTERFACE->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

    TWI_ConfigureMaster(WIRE1_INTERFACE, speed, VARIANT_MCK);
  } else {
    // spam SCL to reset some strange devices
    pinMode(PIN_SDA, OUTPUT);
    pinMode(PIN_SCL, OUTPUT);
    digitalWrite(PIN_SDA, LOW);
    for (int i = 0; i < 256; i++) {
      digitalWrite(PIN_SCL, HIGH);
      delayMicroseconds(1);
      digitalWrite(PIN_SCL, LOW);
      delayMicroseconds(1);
    }
    digitalWrite(PIN_SDA, HIGH);
    for (int i = 0; i < 256; i++) {
      digitalWrite(PIN_SCL, HIGH);
      delayMicroseconds(1);
      digitalWrite(PIN_SCL, LOW);
      delayMicroseconds(1);
    }
    pinMode(PIN_SDA, INPUT_PULLUP);
    pinMode(PIN_SCL, INPUT_PULLUP);

    pmc_enable_periph_clk(WIRE_INTERFACE_ID);
    PIO_Configure(g_APinDescription[PIN_WIRE_SDA].pPort,
                  g_APinDescription[PIN_WIRE_SDA].ulPinType,
                  g_APinDescription[PIN_WIRE_SDA].ulPin,
                  g_APinDescription[PIN_WIRE_SDA].ulPinConfiguration);
    PIO_Configure(g_APinDescription[PIN_WIRE_SCL].pPort,
                  g_APinDescription[PIN_WIRE_SCL].ulPinType,
                  g_APinDescription[PIN_WIRE_SCL].ulPin,
                  g_APinDescription[PIN_WIRE_SCL].ulPinConfiguration);

    NVIC_DisableIRQ(TWI1_IRQn);
    NVIC_ClearPendingIRQ(TWI1_IRQn);

    // Disable PDC channel
    WIRE_INTERFACE->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

    TWI_ConfigureMaster(WIRE_INTERFACE, speed, VARIANT_MCK);
  }
} // void i2cInit(uint8_t bus, uint32_t speed)
