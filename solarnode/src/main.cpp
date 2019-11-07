/** \file main.cpp */

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>


#include <SPI.h>
#include <RF24.h>

#include <solaredge.h>
#include <sunray.h>

/**
 * \brief The main nRF24L01 object.
 *
 * The radio is connected to the Ardunio on the following pins:
 *
 *  9: Chip Enable (CE)
 * 10: SPI Slave Select (SS)
 * 11: SPI MOSI
 * 12: SPI MISO
 * 13: SPI Clock (SCK)
 */
RF24 radio(9, 10);

uint8_t nrf_local_addr[] = "node1";  ///< Our nRF address
uint8_t nrf_remote_addr[] = "node2"; ///< The remote node's nRF address.

static const int serial_tx_pin = 3; ///< The Arduino pin to use for Serial TX
static const int serial_rx_pin = 4; ///< The Arduino pin to use for Serial RX

static const uint32_t timeout_us = 200000; ///< The default time to wait for a message.

/**
 * \brief The software serial port, used for debugging output.
 *
 * We cannot use the hardware serial port because it is being used for the
 * RS-485 connection to the SolarEdge inverter.
 *
 * We use pins 3 (TX) and 4 (RX) on the Arduino for the software serial port.
 */
SoftwareSerial SerialMonitor(serial_rx_pin, serial_tx_pin);

/**
 * \brief The Modbus baud rate.
 *
 * We use 9600bps here because that is the inverter's default.
 */
static const long modbus_baud_rate = 9600;

/**
 * \brief The inverter's configured Modbus ID.
 *
 * This is a configurable setting on the inverter; the default is 1, but I
 * changed it to 2 for testing purposes and have not changed it back.
 */
int inverter_id = 2;

SolarEdgeInverter inverter;     ///< The main inverter object.

/**
 * \brief Prints the last error encountered.
 */
void
print_last_error()
{
    SerialMonitor.print(F("[E] "));
    SerialMonitor.println(inverter.LastError());
}

int
serial_putc(char c, FILE *)
{
    SerialMonitor.write(c);
    return c;
}

/**
 * \brief The Arduino main setup function.
 *
 * Perform any necessary initialization here.
 */
void
setup()
{
    SerialMonitor.begin(9600);
    while (!SerialMonitor)
        delay(100);

    fdevopen(&serial_putc, 0);

    // Set up Modbus.
    if (!ModbusRTUClient.begin(modbus_baud_rate)) {
        SerialMonitor.println(F("[E] Modbus client failure"));
        while(1);
    }

    // Set up the inverter.
    inverter.begin(&ModbusRTUClient, inverter_id);

    delay(1000);

    SerialMonitor.println(F("setting up radio"));
    radio.begin();

    if (!radio.isChipConnected()) {
        while(true) {
            SerialMonitor.println(F("no radio found!"));
            delay(1000);
        }
    }

    // Set up the nRF radio.
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_2MBPS);
    radio.openWritingPipe(nrf_remote_addr);
    radio.openReadingPipe(1, nrf_local_addr);
    radio.setRetries(15, 15);
    radio.startListening();

    radio.printDetails();
    SerialMonitor.println(F("=== setup complete ==="));
}

/**
 * \brief Sends a message via the nRF24L01 radio to the collector node.
 *
 * This function will automatically calculate the message CRC and add it to the
 * message.
 *
 * param[in]  The message to send.
 *
 * \returns True if sending the message succeeded; false otherwise.
 */
bool
send_message(msg_t *msg)
{
    if (add_checksum(msg) == 0) {
        // Something went wrong and we should handle it.
        return false;
    }

    radio.stopListening();

    if (!radio.write(msg, sizeof(msg_t))) {
        SerialMonitor.println(F("failed to send msg"));
        return false;
    }

    radio.startListening();
    return true;
}

/**
 * \brief Attempts to receive a message from the collector node via the nRF24L01
 *        radio.
 *
 * param[out] A `msg_t` in which to place the received message.
 *
 * \returns True if a message was received within the timeout; false otherwise.
 */
bool
receive_message(msg_t *recv)
{
    if (recv == NULL)
        return false;

    uint32_t start = micros();

    while (!radio.available()) {
        if ((micros() - start) > timeout_us) {
            SerialMonitor.println(F("timeout waiting for msg"));
            return false;
        }
    }

    radio.read(recv, sizeof(msg_t));

    return true;
}

/**
 * \brief The main Arduino loop.
 *
 * This function is called repeatedly in a while-loop until the end of time (or
 * power, whichever comes first).
 */
void
loop()
{
    msg_t msg;
    sunray_timesync(&msg);

    SerialMonitor.println(F("getting the time..."));

    if (send_message(&msg)) {
        if (receive_message(&msg)) {
            SerialMonitor.print(F("The time is now "));
            SerialMonitor.println(msg.payload.ts);
        }
    }

    delay(2500);
    SerialMonitor.println();
}

void
get_values()
{
    inverter_status_t status = I_UNUSED;
    float value = 0;

    if (!inverter.OperatingState(&status))
        goto err;

    if (!inverter.DCVoltage(&value))
        goto err;

    if (!inverter.DCCurrent(&value))
        goto err;

    if (!inverter.DCPower(&value))
        goto err;

    if (!inverter.HeatSinkTemperature(&value))
        goto err;

    if (!inverter.ACPower(&value))
        goto err;

    if (!inverter.TotalACCurrent(&value))
        goto err;

    if (!inverter.ACFrequency(&value))
        goto err;

    if (!inverter.LifetimeEnergyProduction(&value))
        goto err;

    return;

err:
    print_last_error();
}
