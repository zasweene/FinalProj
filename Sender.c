#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <string.h>

//zephyr includes
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "SEGGER_RTT.h"

#define LOG_LEVEL 3
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rx_send_resp);

#define APP_NAME "RX SENDRESP v1.0"

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard 8 symbol SFD,
                      * 1 to use non-standard 8 symbol,
                      * 2 for non-standard 16 symbol SFD and
                      * 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,  /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0      /* PDOA mode off */
};

/* As "TX then wait for a response" example sends a blink message encoded as
 * per the ISO/IEC 24730-62:2013 standard which includes a bit signalling
 * that a response is listened for, this example will respond with a valid
 * frame (that will be ignored anyway) following the same standard.
 * The response is a 21-byte frame composed of the following fields:
 * - byte 0/1: frame control (0x8C41 to indicate a data frame using 16-bit
 * source addressing and 64-bit destination addressing).
 * - byte 2: sequence number, incremented for each new frame.
 * - byte 3/4: application ID (0x609A for data frames in this standard).
 * - byte 5 -> 12: 64-bit destination address.
 * - byte 13/14: 16-bit source address, hard coded in this example to
 * keep it simple.
 * - byte 15: function code (0x10 to indicate this is an activity
 * control message).
 * - byte 16: activity code (0x00 to indicate activity is finished).
 * - byte 17/18: new tag blink rate.
 * - byte 19/20: frame check-sum, automatically set by DW IC.  */

/* Payload is 66 bytes */
#define MAX_PAYLOAD_LEN 66

/* tx msg is the payload length plus the header length of 21 bytes. */
static uint8_t tx_msg[MAX_PAYLOAD_LEN + 21];

/* Chunk of data that it will send*/
uint8_t user_input[MAX_PAYLOAD_LEN];

/* function to grab next sequence of bytes from memory. */
void get_chunk(int index, uint8_t* chunk) {
    int start_index = (index - 1) * MAX_PAYLOAD_LEN;

    /* This function grabs bytes from the image stored in the deca_spi.c file. */
    int bound = sizeArray();

    if(index == 0){
        for (int i = 0; i < MAX_PAYLOAD_LEN; i++) {
            chunk[i] = 0xFF;
            
        }
    }
    if(index > 0){
        // Check if the chunk index is within the bounds of the hex array   
        if (start_index < bound) {
            for (int i = 0; i < MAX_PAYLOAD_LEN; i++) {
                if (start_index + i < bound) {
                    chunk[i] = hexArray(start_index + i);
                } else {
                    chunk[i] = 0xAA; // Fill remaining bytes with AA if out of bounds
                }
            }
        } else {
            for (int i = 0; i < MAX_PAYLOAD_LEN; i++) {
                if (start_index + i < bound) {
                    chunk[i] = 0xAA;
                } 
            }
        }
    }
    
}

/* Indexes to access to sequence number and destination address of the data frame in the tx_msg array. */
#define DATA_FRAME_SN_IDX 2
#define DATA_FRAME_DEST_IDX 5

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 1000

/* Buffer to store received frame. */
static uint8_t rx_buffer[32];

/* Index to access to source address of the blink frame in the rx_buffer array. */
#define BLINK_FRAME_SRC_IDX 2

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and
 * power of the spectrum at the current temperature.
 * These values can be calibrated prior to taking reference measurements. */
extern dwt_txconfig_t txconfig_options;


int app_main(void)
{
    /* Hold copy of status register state here for reference so that
     * it can be examined at a debug breakpoint. */
    uint32_t status_reg = 0;

    /* Hold copy of frame length of frame received (if good) so
     * that it can be examined at a debug breakpoint. */
    uint16_t frame_len = 0;

    /* Display application name on LCD. */
    LOG_INF(APP_NAME);

    /* Configure SPI rate, DW3000 supports up to 38 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    /* Target specific drive of RSTn line into DW IC low for a period. */
    reset_DWIC();

    /* Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC,
     * or could wait for SPIRDY event) */
    Sleep(2);

    /* Need to make sure DW IC is in IDLE_RC before proceeding */
    while (!dwt_checkidlerc()) { };

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        LOG_ERR("INIT FAILED");
        while (1) {};
    }

    /* Enabling LEDs here for debug so that for each TX the D1 LED will flash
     * on DW3000 red eval-shield boards. */
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK) ;

    /* Configure DW IC. */
    /* If the dwt_configure returns DWT_ERROR either the PLL or RX calibration
     * has failed the host should reset the device */
    if (dwt_configure(&config)) {
        LOG_INF("CONFIG FAILED");
        while (1) {};
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* Loop forever sending and receiving frames periodically. */
    while (1) {

        /* Activate reception immediately. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Poll until a frame is properly received or an error occurs.
         * STATUS register is 5 bytes long but, as the events we are looking
         * at are in the lower bytes of the register, we can use this simplest
         * API function to access it. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)))
        {};

        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
        {
            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
            if (frame_len <= 1023) { // Changed from FRAME_LEN_MAX
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            /* Clear good RX frame event in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
#if 1
            uint8_t len1[15];
            sprintf(len1, "msg len %d", frame_len);
            LOG_HEXDUMP_INF((uint8_t*)&rx_buffer, frame_len, (uint8_t*) &len1);
#endif
            /* Validate the frame is the one expected as sent by "TX then
             * wait for a response" example. */
            if ((frame_len == 14) &&
                (rx_buffer[0] == 0xC5) &&
                (rx_buffer[10] == 0x43) &&
                (rx_buffer[11] == 0x2)) {

                uint8_t header[] = {
                    0x41, 0x8C, 0x00,         // Frame control + Seq num (update later)
                    0x9A, 0x60,               // App ID
                    0x00, 0x00, 0x00, 0x00,   // Dest address
                    0x00, 0x00, 0x00, 0x00,
                    'D', 'W',                 // Source address
                    0x10,                     // Function code
                    0x00,                     // Activity code
                    0x00, 0x00                // Blink rate
                };
                memset(tx_msg, 0, sizeof(tx_msg));
                memcpy(tx_msg, header, sizeof(header));
                
                tx_msg[DATA_FRAME_SN_IDX] = rx_buffer[1];

                /* Grab next byte chunk from memory and append the header to form a full frame. */
                get_chunk(tx_msg[DATA_FRAME_SN_IDX] - 5, user_input);
                int input_len = sizeof(user_input);
                memcpy(&tx_msg[sizeof(header)], user_input, input_len);
                
                /* Copy source address of blink in response destination address. */
                for (int i = 0; i < 8; i++) {
                    tx_msg[DATA_FRAME_DEST_IDX + i] = rx_buffer[BLINK_FRAME_SRC_IDX + i];
                }

                /* Write response frame data to DW IC and prepare transmission. */
                dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
                dwt_writetxfctrl(sizeof(tx_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

                /* Send the response. */
                dwt_starttx(DWT_START_TX_IMMEDIATE);

                /* Poll DW IC untilTX frame sent event set. */
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
                {};

                /* Clear TX frame sent event. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
#if 1
                uint8_t len2[15];
                sprintf(len2, "resp len %d", sizeof(tx_msg));
                LOG_HEXDUMP_INF((uint8_t*)&tx_msg, sizeof(tx_msg), (uint8_t*) &len2);
#endif

            }
        }
        else {
            /* Clear RX error events in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
    }
}