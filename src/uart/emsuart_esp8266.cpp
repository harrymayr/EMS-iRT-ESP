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

#include "uart/emsuart_esp8266.h"

#include "emsesp.h"

namespace emsesp {

typedef struct {
    uint8_t length;
    uint8_t buffer[EMS_MAXBUFFERSIZE];
} EMSRxBuf_t;

typedef struct {
	uint8_t	valid;
	uint8_t	state;
	uint8_t	address;
	unsigned long	start_time;

	uint8_t	tx_bytes;
	uint8_t	rx_bytes;

	uint8_t	pos;
	uint8_t	len;
	uint8_t	buffer[IRT_MAXTXBUFFERSIZE];

} _IRTTxBuf;

    /* iRT UART transfer status */
    typedef enum {
        IRT_RX_STATUS_IDLE,
        IRT_RX_STATUS_BUSY // Rx package is being received
    } _IRT_RX_STATUS;

os_event_t   recvTaskQueue[EMSUART_recvTaskQueueLen]; // our Rx queue
EMSRxBuf_t   aEMSRxBuf[EMS_MAXBUFFERS];
EMSRxBuf_t * pEMSRxBuf     = &aEMSRxBuf[0];
EMSRxBuf_t * pCurrent      = pEMSRxBuf;
_IRTTxBuf  * pIRTTxBuf;
uint8_t      emsRxBufIdx_  = 0;
uint8_t      tx_mode_      = 0xFF;
bool         drop_next_rx_ = true;
uint8_t      irtRxStatus = IRT_RX_STATUS_BUSY;
uint8_t		_disable_rxtx = 0;

static uint8_t irtuart_tx_received_byte(_IRTTxBuf *pTx, uint8_t rx, uint8_t *buffer, uint8_t in_length)
{
	/* for each incoming byte, test if we need to send anything */

	uint8_t out_length = in_length;

	if (pTx == NULL) return out_length;
	if (pTx->valid != 1) return out_length;

	if (pTx->state > 90) return out_length;

	// check if all data has been send
	if (pTx->pos >= pTx->len) {
		pTx->valid = 2; /* done */
		pTx->state = 98;
		return out_length;
	}

	if (pTx->state == 1) {
		/* check for the 'break' 0 byte or the address */
		if ((rx == 0) || (rx == pTx->address)) {
			pTx->state = 2; // break byte
			if (rx == pTx->address) pTx->state = 3;
		} else {
			// not our time slot, wait for break
			pTx->state = 0;
		}
	} else if (pTx->state == 2) {
		if (rx == pTx->address) {
			pTx->state = 3;
		} else {
			// not our slot
			pTx->state = 0;
		}
	} else if (pTx->state == 4) {
		/* we just send our address, we should receive the echo */
		if (rx == pTx->address) {
			pTx->state = 5;
		} else {
			// something went wrong, abort
			pTx->valid = 3; // error
			pTx->state = 99;
		}
	} else if (pTx->state == 5) {
		/* check for inverse address from boiler */
		if (rx == (0xFF - pTx->address)) {
			pTx->state = 10; // start tx
		} else {
			// something went wrong, abort
			pTx->valid = 3; // error
			pTx->state = 99;
		}
	}

	if (pTx->state == 14) { // receiving bytes
		pTx->buffer[pTx->pos++] = rx;
		if (pTx->rx_bytes > 0) pTx->rx_bytes--;
		if (pTx->rx_bytes) {
			// check for rx bytes
			pTx->state = 14;
		} else {
			// msg done, check for next
			pTx->state = 10;
		}
	}

	if (pTx->state == 13) { // echo from boiler
		if (rx == pTx->buffer[pTx->pos]) {
			pTx->pos++;
			if (pTx->tx_bytes > 0) pTx->tx_bytes--;
			if (pTx->tx_bytes > 0) {
				pTx->state = 11;
			} else if (pTx->rx_bytes > 0) {
				// check for rx bytes
				pTx->state = 14;
			} else {
				// msg done, check for next
				pTx->state = 10;
			}
		} else {
			// something went wrong, abort
			pTx->valid = 3; // error
			pTx->state = 99;
		}
	}

	if (pTx->state == 10) {
		/* start first msg byte */
		if (pTx->pos < pTx->len) {
			if (pTx->buffer[pTx->pos] & 0x80) {
				pTx->tx_bytes = 4;
				pTx->rx_bytes = 2; /* response plus inverted response */
			} else {
				pTx->tx_bytes = 4;
				pTx->rx_bytes = 0;
			}
			pTx->state = 11;
		} else {
			pTx->valid = 2; /* done */
			pTx->state = 20; /* done */
		}
	}

	if (pTx->state == 12) { // own echo
		if (rx == pTx->buffer[pTx->pos]) {
			pTx->state = 13;
		} else {
			// something went wrong, abort
			pTx->valid = 3; // error
			pTx->state = 99;
		}
	}
	if (pTx->state == 11) {
		/* transmit byte */
		USF(EMSUART_UART) = pTx->buffer[pTx->pos];
		pTx->state = 12;
	}

	return out_length;
}

void IRAM_ATTR EMSuart::irtuart_rx_intr_handler(void * para) {
	static uint8_t length;
	static uint8_t uart_buffer[EMS_MAXBUFFERSIZE + 2];

	unsigned long now_millis = millis();

	// is a new buffer? if so init the thing for a new telegram
 	if (irtRxStatus == IRT_RX_STATUS_IDLE) {
		irtRxStatus = IRT_RX_STATUS_BUSY; // status set to busy
		length                     = 0;
	}
 
	/* If we have valid transmit buffer, detect break before processing data */
	if ((pIRTTxBuf) && (pIRTTxBuf->valid == 1)) {
		if ( (pIRTTxBuf->state < 20) && ((now_millis - pIRTTxBuf->start_time) >= IRTUART_TX_MSG_TIMEOUT_MS) ) {
			pIRTTxBuf->state = 97; // abort on timeout
			pIRTTxBuf->valid = 3; // exit
		}
		if (pIRTTxBuf->state == 0) {
			if ( USIS(EMSUART_UART) & (1 << UIBD) ) {
				// we have a break, go to next state
				pIRTTxBuf->state = 1;
			}
		}
	}

    // fill IRQ buffer, by emptying Rx FIFO
    uint8_t rx_cnt = 0;
    if (USIS(EMSUART_UART) & ((1 << UIFF) | (1 << UITO) | (1 << UIBD))) {
        while (((USS(EMSUART_UART) >> USRXC) & 0xFF) && (rx_cnt < 100)) {
				rx_cnt ++;
            uint8_t rx = USF(EMSUART_UART);
            if (length < EMS_MAXBUFFERSIZE) uart_buffer[length++] = rx;

				length = irtuart_tx_received_byte(pIRTTxBuf, rx, uart_buffer, length);
        }

        // clear Rx FIFO full and Rx FIFO timeout interrupts
        USIC(EMSUART_UART) = (1 << UIFF) | (1 << UITO);
	}
	if ((pIRTTxBuf) && (pIRTTxBuf->valid == 1)) {
		if (pIRTTxBuf->state == 3) {

			/* send own address, to let the boiler know we want to transmit */
			USF(EMSUART_UART) = pIRTTxBuf->address;
			pIRTTxBuf->state = 4;
		}
	}


    // BREAK detection = End of IRT data block or overflow
    if ((USIS(EMSUART_UART) & ((1 << UIBD))) || (length >= IRT_MAX_TELEGRAM_LENGTH)) {
        ETS_UART_INTR_DISABLE();          // disable all interrupts and clear them
        USIC(EMSUART_UART) = (1 << UIBD); // INT clear the BREAK detect interrupt

        pEMSRxBuf->length = (length > EMS_MAXBUFFERSIZE) ? EMS_MAXBUFFERSIZE : length;
        os_memcpy((void *)pEMSRxBuf->buffer, (void *)&uart_buffer, pEMSRxBuf->length); // copy data into transfer buffer, including the BRK 0x00 at the end
        length                     = 0;
        irtRxStatus = IRT_RX_STATUS_IDLE; // set the status flag stating BRK has been received and we can start a new package
        ETS_UART_INTR_ENABLE();                          // re-enable UART interrupts

            pCurrent  = pEMSRxBuf;                                   // current buffer to receive task
            pEMSRxBuf = &aEMSRxBuf[++emsRxBufIdx_ % EMS_MAXBUFFERS]; // next free EMS Receive buffer
        system_os_post(EMSUART_recvTaskPrio, 0, 0); // call irtuart_recvTask() at next opportunity
    }

}

//
// Main interrupt handler
// Important: must not use ICACHE_FLASH_ATTR
//
void IRAM_ATTR EMSuart::emsuart_rx_intr_handler(void * para) {

    if (USIR(EMSUART_UART) & (1 << UIBD)) {  // BREAK detection = End of EMS data block
        USC0(EMSUART_UART) &= ~(1 << UCBRK); // reset tx-brk
        USIC(EMSUART_UART) = (1 << UIBD);    // INT clear the BREAK detect interrupt
        pEMSRxBuf->length = 0;
        while ((USS(EMSUART_UART) >> USRXC) & 0x0FF) { // read fifo into buffer
            uint8_t rx = USF(EMSUART_UART);
            if (pEMSRxBuf->length < EMS_MAXBUFFERSIZE) {
                if (pEMSRxBuf->length || rx) { // skip a leading zero
                    pEMSRxBuf->buffer[pEMSRxBuf->length++] = rx;
                }
            } else {
                drop_next_rx_ = true;
            }
        }
        if (pEMSRxBuf->buffer[pEMSRxBuf->length - 1]) { // check if last byte is break
            pEMSRxBuf->length++;
        }
        // Ignore telegrams with no data value, then transmit EMS buffer, excluding the BRK
        if (!drop_next_rx_ && (pEMSRxBuf->length > 4 || pEMSRxBuf->length == 2)) {
            pCurrent  = pEMSRxBuf;                                   // current buffer to receive task
            pEMSRxBuf = &aEMSRxBuf[++emsRxBufIdx_ % EMS_MAXBUFFERS]; // next free EMS Receive buffer
            system_os_post(EMSUART_recvTaskPrio, 0, 0);              // call emsuart_recvTask() at next opportunity
        }
        drop_next_rx_ = false;
    }
}

/*
 * system task triggered on BRK interrupt
 * incoming received messages are always asynchronous
 * The full buffer is sent to EMSESP::incoming_telegram()
 */
void ICACHE_FLASH_ATTR EMSuart::emsuart_recvTask(os_event_t * events) {

    EMSESP::incoming_telegram((uint8_t *)pCurrent->buffer, pCurrent->length - 1);
}

/*
 * init UART0 driver
 */
void ICACHE_FLASH_ATTR EMSuart::start(const uint8_t tx_mode, const uint8_t rx_gpio, const uint8_t tx_gpio) {
    if (tx_mode_ != 0xFF) { // it's a restart no need to configure uart
        tx_mode_ = tx_mode;
        restart();
        return;
    }
    tx_mode_ = tx_mode;

    ETS_UART_INTR_ATTACH(nullptr, nullptr);

    // pin settings
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0RXD_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD);

    if (tx_mode_ <= EMS_TXMODE_HW) {
      // set 9600, 8 bits, no parity check, 1 stop bit
      USD(EMSUART_UART)  = (UART_CLK_FREQ / EMSUART_BAUD);
      USC0(EMSUART_UART) = EMSUART_CONFIG; // 8N1
      USC0(EMSUART_UART) |= ((1 << UCRXRST) | (1 << UCTXRST));  // set clear fifo bits
      USC0(EMSUART_UART) &= ~((1 << UCRXRST) | (1 << UCTXRST)); // clear bits
      // set interrupts for triggers
      USIC(EMSUART_UART) = 0xFFFF; // clear all interupts
      USIE(EMSUART_UART) = 0;      // disable all interrupts
    }
    else {
      // set 4800, 8 bits, no parity check, 1 stop bit
      USD(EMSUART_UART)  = (UART_CLK_FREQ / IRTUART_BAUD);
      if ((tx_mode_ == EMS_TXMODE_IRT_ACTIVE_POLL) || (tx_mode_ == EMS_TXMODE_IRT_ACTIVE) ) {
  		USC0(EMSUART_UART) = IRTUART_CONFIG_ACTIVE; // 8N1
      } else {
	  	USC0(EMSUART_UART) = IRTUART_CONFIG_PASSIVE; // 8N1
      }
      USC0(EMSUART_UART) |= ((1 << UCRXRST) | (1 << UCTXRST));  // set clear fifo bits
      USC0(EMSUART_UART) &= ~((1 << UCRXRST) | (1 << UCTXRST)); // clear bits
      // set interrupts for triggers
      USIC(EMSUART_UART) = 0xFFFF; // clear all interupts
      USIE(EMSUART_UART) = 0;      // disable all interrupts
	  USC1(EMSUART_UART) = 0;                                                // reset config first
      USC1(EMSUART_UART) = (0x01 << UCFFT) | (0x01 << UCTOT) | (0 << UCTOE); // enable interupts
			// Create a transmit buffer
		pIRTTxBuf = (_IRTTxBuf *)malloc(sizeof(_IRTTxBuf));
		pIRTTxBuf->valid = 0;
		pIRTTxBuf->state = 0;

    }


    // conf1 params
    // UCTOE = RX TimeOut enable (default is 1)
    // UCTOT = RX TimeOut Threshold (7 bit) = want this when no more data after 1 characters (default is 2)
    // UCFFT = RX FIFO Full Threshold (7 bit) = want this to be 31 for 32 bytes of buffer (default was 127)
    // see https://www.espressif.com/sites/default/files/documentation/esp8266-technical_reference_en.pdf
    //
    // change: don't care, we do not use these interrupts
    // USC1(EMSUART_UART) = (0x7F << UCFFT) | (0x01 << UCTOT) | (1 << UCTOE); // enable interupts


    // set up interrupt callbacks for Rx
    system_os_task(emsuart_recvTask, EMSUART_recvTaskPrio, recvTaskQueue, EMSUART_recvTaskQueueLen);

    // disable esp debug which will go to Tx and mess up the line - see https://github.com/espruino/Espruino/issues/655
    system_set_os_print(0);

    if (tx_gpio == 1 && rx_gpio == 3) {
        system_uart_de_swap();
    } else if (tx_gpio == 15 && rx_gpio == 13) {
        system_uart_swap(); // swap Rx and Tx pins to use GPIO13 (D7) and GPIO15 (D8) respectively
    }

    if (tx_mode_ <= EMS_TXMODE_HW) 
        ETS_UART_INTR_ATTACH(emsuart_rx_intr_handler, nullptr);
    else
        ETS_UART_INTR_ATTACH(irtuart_rx_intr_handler, nullptr);

    drop_next_rx_ = true;

    restart();
}

/*
 * stop UART0 driver
 * This is called prior to an OTA upload and also before a save to the filesystem to prevent conflicts
 */
void ICACHE_FLASH_ATTR EMSuart::stop() {
    USIE(EMSUART_UART) = 0;              // disable receive interrupts
    USC0(EMSUART_UART) &= ~(1 << UCBRK); // clear Tx-BRK bit
    _disable_rxtx = 1;
}

/*
 * re-start UART0 driver
 */
void ICACHE_FLASH_ATTR EMSuart::restart() {
    if (USIR(EMSUART_UART) & ((1 << UIBD))) {
        USIC(EMSUART_UART) = (1 << UIBD); // INT clear the <brk> detect interrupt
        drop_next_rx_      = true;
    }
    if (tx_mode_ <= EMS_TXMODE_HW) 
	    USIE(EMSUART_UART) = (1 << UIBD); // enable brk interrupt
	else
		USIE(EMSUART_UART) = (1 << UIBD) | (1 << UIFF) | (0 << UITO);
    _disable_rxtx = 0;
}

/*
 * Sends a 1-byte poll, ending with a <BRK>
 */
void ICACHE_FLASH_ATTR EMSuart::send_poll(uint8_t data) {
    transmit(&data, 1);
}

/*
 * Send data to Tx line, ending with a <BRK>
 * buf contains the CRC and len is #bytes including the CRC
 * returns code, 0=success, 1=brk error, 2=watchdog timeout
 */
uint16_t ICACHE_FLASH_ATTR EMSuart::transmit(uint8_t * buf, uint8_t len) {
    if (len == 0 || len >= EMS_MAXBUFFERSIZE) {
        return EMS_TX_STATUS_ERR; // nothing or to much to send
    }
    if (tx_mode_ == 0) {
        return EMS_TX_STATUS_OK;
    }

    // See https://github.com/emsesp/EMS-ESP/issues/380
    if (tx_mode_ == EMS_TXMODE_HW) { // tx_mode 4
        for (uint8_t i = 0; i < len; i++) {
            USF(EMSUART_UART) = buf[i];
        }
        USC0(EMSUART_UART) |= (1 << UCBRK); // send <BRK> at the end, clear by interrupt
        return EMS_TX_STATUS_OK;
    }

    // EMS+ https://github.com/emsesp/EMS-ESP/issues/23#
    if (tx_mode_ == EMS_TXMODE_EMSPLUS) { // tx_mode 2, With extra tx delay for EMS+
        for (uint8_t i = 0; i < len; i++) {
            USF(EMSUART_UART) = buf[i];
            delayMicroseconds(EMSUART_TX_WAIT_PLUS); // 2070
        }
        USC0(EMSUART_UART) |= (1 << UCTXI); // set break
        delayMicroseconds(EMSUART_TX_BRK_PLUS);
        USC0(EMSUART_UART) &= ~(1 << UCTXI);
        return EMS_TX_STATUS_OK;
    }

    // Junkers logic by @philrich, tx_mode 3
    if (tx_mode_ == EMS_TXMODE_HT3) {
        for (uint8_t i = 0; i < len; i++) {
            USF(EMSUART_UART) = buf[i];
            // just to be safe wait for tx fifo empty (still needed?)
            while (((USS(EMSUART_UART) >> USTXC) & 0xff)) {
            }
            // wait until bits are sent on wire
            delayMicroseconds(EMSUART_TX_WAIT_HT3);
        }
        USC0(EMSUART_UART) |= (1 << UCTXI); // set break bit
        delayMicroseconds(EMSUART_TX_BRK_HT3);
        USC0(EMSUART_UART) &= ~(1 << UCTXI);
        return EMS_TX_STATUS_OK;
    }

	if (tx_mode_ == EMS_TXMODE_IRT_ACTIVE_POLL) {
		uint16_t status __attribute__ ((aligned (4)));
		status = irtuart_send_tx_buffer(1, buf+4, len-4);
        delayMicroseconds(EMSUART_TX_BIT_TIME * 5); // burn CPU cycles...
		if ((status & 0xFF00) == 0x0200) 
			return status;
		else
			return EMS_TX_STATUS_OK;
	}
    /*
     * Logic for tx_mode of 1
     * based on code from https://github.com/emsesp/EMS-ESP/issues/103 by @susisstrolch
     * 
     * Logic (modified by @MichaelDvP):
     * wait after each byte for the master echo
     * after last byte echo send a fixed break and leave.
     * The master echo will trigger the interrupt.
     */

    // send the bytes along the serial line
    for (uint8_t i = 0; i < len; i++) {
        volatile uint8_t _usrxc     = (USS(EMSUART_UART) >> USRXC) & 0xFF;
        uint16_t         timeoutcnt  __attribute__ ((aligned (4))) = EMSUART_TX_TIMEOUT;
        USF(EMSUART_UART)           = buf[i]; // send each Tx byte
        // wait for echo
        while ((((USS(EMSUART_UART) >> USRXC) & 0xFF) == _usrxc) && (--timeoutcnt > 0)) {
            delayMicroseconds(EMSUART_TX_BUSY_WAIT); // burn CPU cycles...
        }
    }
    USC0(EMSUART_UART) |= (1 << UCTXI); // snd break
    delayMicroseconds(EMSUART_TX_BRK_EMS);
    USC0(EMSUART_UART) &= ~(1 << UCTXI);
    return EMS_TX_STATUS_OK; // send the Tx ok status back
}
/*
 * check if we are not transmitting, busy transmitting
 * or have a complete buffer
 */

uint16_t ICACHE_FLASH_ATTR EMSuart::irtuart_check_tx(uint8_t reset_if_done)
{
	uint8_t tx_valid;
	uint8_t tx_state;
	uint16_t tx_ret __attribute__ ((aligned (4)));

	unsigned long now_millis __attribute__ ((aligned (4))) = millis();

	if (_disable_rxtx) return 0x0100;

	ETS_UART_INTR_DISABLE(); // disable rx interrupt
	tx_valid = 0;
	tx_state = 0;
	if (pIRTTxBuf) {
		// check for timeout of message
		if (pIRTTxBuf->valid == 1) {
			if ((now_millis - pIRTTxBuf->start_time) >= IRTUART_TX_MSG_TIMEOUT_MS) {
				pIRTTxBuf->state = 97; // abort on timeout
				pIRTTxBuf->valid = 3; // exit
			}
		}
		// copy current state with irq's off
		tx_valid = pIRTTxBuf->valid;
		tx_state = pIRTTxBuf->state;
		if (pIRTTxBuf->valid > 1) {
			pIRTTxBuf->valid = 0;
			pIRTTxBuf->state = 0;
		}
	}
	ETS_UART_INTR_ENABLE(); // receive anything from FIFO...

	// combine valid and state as single entry
	tx_ret = tx_valid;
	tx_ret = tx_ret << 8;
	tx_ret |= tx_state;

	return tx_ret;
}
uint16_t ICACHE_FLASH_ATTR EMSuart::irtuart_send_tx_buffer(uint8_t address, uint8_t *telegram, uint8_t len)
{
	uint16_t status __attribute__ ((aligned (4)));

	if ((pIRTTxBuf == NULL) || (len >= IRT_MAXTXBUFFERSIZE)) return 0xFF00;

	if (_disable_rxtx) return 0xFF01;

	status = irtuart_check_tx(1);
	// if we are still processing or there is a finished buffer
	// return status, only if empty add buffer
	if ((len < 1) || (status >= 0x0100)) return status;

	// the irq will only pick-up a buffer if valid == 1
	// we just checked it is  not, so no need to disable irq

	pIRTTxBuf->state = 0;
	pIRTTxBuf->start_time = millis();
	pIRTTxBuf->valid = 0;
	pIRTTxBuf->address = address;

	memcpy(pIRTTxBuf->buffer, telegram, len);

	pIRTTxBuf->tx_bytes = 0;
	pIRTTxBuf->rx_bytes = 0;

	pIRTTxBuf->pos = 0;
	pIRTTxBuf->len = len;

	pIRTTxBuf->state = 0;
	// The last instruction puts the buffer to valid
	// after this it may be picked up by the irq
	pIRTTxBuf->valid = 1;

	return 0x0100;
}

} // namespace emsesp

#endif
