/*
 * EMS-ESP - https://github.com/emsesp/EMS-ESP
 * Copyright 2020  Paul Derbyshire
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#if defined(ESP8266)

#ifndef EMSESP_EMSUART_H
#define EMSESP_EMSUART_H

#include <Arduino.h>

#include <user_interface.h>

#define EMSUART_UART 0      // UART 0
#define EMSUART_CONFIG 0x1C // 8N1 (8 bits, no parity, 1 stop bit)
#define IRTUART_CONFIG_PASSIVE (0x1C | (1 << UCRXI) ) // 8N1 (8 bits, no stop bits, 1 parity), invert Rxd
#define IRTUART_CONFIG_ACTIVE  (0x1C | (1 << UCRXI) | (1 << UCTXI)) // 8N1 (8 bits, no stop bits, 1 parity), invert Rxd, Invert TX
#define EMSUART_BAUD 9600   // uart baud rate for the EMS circuit
#define IRTUART_BAUD 4800   // uart baud rate for the EMS circuit

#define EMS_MAXBUFFERS 3     // buffers for circular filling to avoid collisions
#define EMS_MAXBUFFERSIZE 33 // max size of the buffer. EMS packets are max 32 bytes, plus extra for BRK
#define IRT_MAXTXBUFFERSIZE (EMS_MAXBUFFERSIZE / 2)		// max size of tx buffer

#define EMSUART_recvTaskPrio 2     // 0, 1 or 2. 0 being the lowest
#define EMSUART_recvTaskQueueLen 3 // number of queued Rx triggers

#define EMS_TXMODE_DEFAULT 1
#define EMS_TXMODE_EMSPLUS 2
#define EMS_TXMODE_HT3 3
#define EMS_TXMODE_HW 4
#define EMS_TXMODE_IRT_PASSIVE 5
#define EMS_TXMODE_IRT_ACTIVE_POLL 6
#define EMS_TXMODE_IRT_ACTIVE 7

// LEGACY
#define EMSUART_TX_BIT_TIME 104 // bit time @9600 baud

// EMS 1.0
#define EMSUART_TX_BUSY_WAIT (EMSUART_TX_BIT_TIME / 8) // 13
// #define EMSUART_TX_TIMEOUT (32 * 8) // 256 for tx_mode 1 - see https://github.com/emsesp/EMS-ESP/issues/398#issuecomment-645886277
#define EMSUART_TX_TIMEOUT (220 * 8) // 1760 as in v1.9 (180 ms)
#define EMSUART_TX_BRK_EMS (EMSUART_TX_BIT_TIME * 10)
#define IRTUART_TX_MSG_TIMEOUT_MS 4000	// if a message is still waiting after 4s abort

// HT3/Junkers - Time to send one Byte (8 Bits, 1 Start Bit, 1 Stop Bit) plus 7 bit delay. The -8 is for lag compensation.
// since we use a faster processor the lag is negligible
#define EMSUART_TX_WAIT_HT3 (EMSUART_TX_BIT_TIME * 17) // 1768
#define EMSUART_TX_BRK_HT3 (EMSUART_TX_BIT_TIME * 11)

// EMS+ - Time to send one Byte (8 Bits, 1 Start Bit, 1 Stop Bit) and delay of another Bytetime.
#define EMSUART_TX_WAIT_PLUS (EMSUART_TX_BIT_TIME * 20) // 2080
#define EMSUART_TX_BRK_PLUS (EMSUART_TX_BIT_TIME * 11)


namespace emsesp {

#define EMS_TX_STATUS_ERR 0
#define EMS_TX_STATUS_OK 1

class EMSuart {
  public:
    EMSuart()  = default;
    ~EMSuart() = default;

    static void ICACHE_FLASH_ATTR     start(const uint8_t tx_mode, const uint8_t rx_gpio, const uint8_t tx_gpio);
    static void ICACHE_FLASH_ATTR     stop();
    static void ICACHE_FLASH_ATTR     send_poll(uint8_t data);
    static uint16_t ICACHE_FLASH_ATTR transmit(uint8_t * buf, uint8_t len);

  private:
    static void IRAM_ATTR   emsuart_rx_intr_handler(void * para);
    static void IRAM_ATTR   irtuart_rx_intr_handler(void * para);
    static void ICACHE_FLASH_ATTR emsuart_recvTask(os_event_t * events);
    static void ICACHE_FLASH_ATTR restart();
    static uint16_t ICACHE_FLASH_ATTR irtuart_check_tx(uint8_t reset_if_done);
    static uint16_t ICACHE_FLASH_ATTR irtuart_send_tx_buffer(uint8_t address, uint8_t *telegram, uint8_t len);

};

} // namespace emsesp

#endif
#endif