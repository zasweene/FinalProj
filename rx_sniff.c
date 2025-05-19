#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define LOG_LEVEL 3
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tx_wait_resp_int);

#define APP_NAME "TX W4R IRQ v1.0"

static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard 8 symbol SFD,
                      *   1 to use non-standard 8 symbol,
                      *   2 for non-standard 16 symbol SFD and
                      *   3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,  /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0      /* PDOA mode off */
};

/* The frame sent in this example is a blink encoded as per the
 * SO/IEC 24730-62:2013 standard. It is a 14-byte frame composed of
 * the following fields:
 *     - byte 0: frame control (0xC5 to indicate a multipurpose frame
 *                 using 64-bit addressing).
 *     - byte 1: This is the frame number that it is requesting. This increments upon each valid frame receipt. 
 *     - byte 2 -> 9: device ID.
 *     - byte 10: encoding header (0x43 to indicate no extended ID,
 *                 temperature, or battery status is carried in the message).
 *     - byte 11: EXT header (0x02 to indicate tag is listening for a
 *                 response immediately after this message).
 *     - byte 12/13: frame check-sum, automatically set by DW IC. */
static uint8_t tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0x43, 0x02, 0, 0};

/* Index to access the sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

/* Delay from end of transmission to activation of reception, expressed in
 * UWB microseconds (1 uus is 512/499.2 microseconds).*/
#define TX_TO_RX_DELAY_UUS 60

/* Receive response timeout, expressed in UWB microseconds.*/
#define RX_RESP_TO_UUS 100000

#define DFLT_TX_DELAY_MS 0

#define RX_TO_TX_DELAY_MS 0

#define RX_ERR_TX_DELAY_MS 0

/* Current inter-frame delay period.
 * This global static variable is also used as the mechanism to signal
 * events to the background main loop from the interrupt handler callbacks,
 * which set it to positive delay values. */
static int32_t tx_delay_ms = -1;

/* Buffer to store received frame. 92 byte frame.*/
static uint8_t rx_buffer[92];
static int32_t needed[1];

/* Declaration of static functions. */
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_to_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);
static void tx_conf_cb(const dwt_cb_data_t *cb_data);

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and
 * power of the spectrum at the current temperature.
 * These values can be calibrated prior to taking reference measurements. */
extern dwt_txconfig_t txconfig_options;

int app_main(void)
{
    needed[0] = 0;
    /* Display application name. */
    LOG_INF(APP_NAME);

    /* Configure SPI rate, DW3000 supports up to 38 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    reset_DWIC(); 
    Sleep(2);

    /* Need to make sure DW IC is in IDLE_RC before proceeding */
    while (!dwt_checkidlerc()) {};

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        LOG_ERR("INIT FAILED");
        while (1) {};
    }

    /* Configure DW IC. */
    /* If the dwt_configure returns DWT_ERROR either the PLL or RX calibration
     * has failed the host should reset the device */
    if (dwt_configure(&config)) {
        LOG_ERR("CONFIG FAILED");
        while (1) {};
    }

    /* Enabling LEDs here for debug so that for each TX the D1 LED will flash
     * on DW3000 red eval-shield boards. */
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK) ;

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* Register the call-backs (SPI CRC error callback is not used). */
    dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb, NULL, NULL);

    /* Enable wanted interrupts (TX confirmation, RX good frames,
     * RX timeouts and RX errors). */
    dwt_setinterrupt(SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK |
                     SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK |
                     SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK |
                     SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK |
                     SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK |
                     SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK |
                     SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK |
                     SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK,
                     0,
                     DWT_ENABLE_INT);

    /* Clearing the SPI ready interrupt */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RCINIT_BIT_MASK | SYS_STATUS_SPIRDY_BIT_MASK);

    /* Install DW IC IRQ handler. */
    port_set_dwic_isr(dwt_isr);

    /* Set delay to turn reception on after transmission of the frame.*/
    dwt_setrxaftertxdelay(TX_TO_RX_DELAY_UUS);

    /* Set response frame timeout. */
    dwt_setrxtimeout(RX_RESP_TO_UUS);

    /* Loop forever sending and receiving frames periodically. */
    while (1) {

        /* Write frame data to DW IC and prepare transmission. */
        dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

        /* Start transmission, indicating that a response is expected so that
         * reception is enabled immediately after the frame is sent. */
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        /* Wait for any RX event. */
        while (tx_delay_ms == -1) { /* spin */ };

        /* Execute the defined delay before next transmission. */
        if (tx_delay_ms > 0) {
            Sleep(tx_delay_ms);
        }

        /* Reset the TX delay and event signaling mechanism ready to
         * await the next event. */
        tx_delay_ms = -1;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_ok_cb()
 *
 * @brief Callback to process RX good frame events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    
    /* Clear local RX buffer to avoid having leftovers from previous receptions.
     * This is not necessary but is included here to aid reading the RX
     * buffer. */
    for (int i = 0; i < FRAME_LEN_MAX; i++ ) {
        rx_buffer[i] = 0;
    }

    /* A frame has been received, copy it to our local buffer. */
    if (cb_data->datalength <= FRAME_LEN_MAX) {       
        
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
        
        /* If the correct frame is received, request the next frame. */
        if(rx_buffer[2] == needed[0]) {
            needed[0] = needed[0] + 1;
            tx_msg[BLINK_FRAME_SN_IDX] = needed[0];
        }
#if 1
        LOG_INF("OK: len: %d resp:", cb_data->datalength);
        for (int i = 0; i < cb_data->datalength; i++) {
            LOG_RAW("%02X", ((uint8_t *)&rx_buffer)[i]);
        }
        LOG_RAW("\n");
#endif
    }

    tx_delay_ms = DFLT_TX_DELAY_MS;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_to_cb()
 *
 * @brief Callback to process RX timeout events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_to_cb(const dwt_cb_data_t *cb_data)
{
    (void) cb_data;

    LOG_INF("%s: timeout", __func__);

    tx_delay_ms = RX_TO_TX_DELAY_MS;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_err_cb()
 *
 * @brief Callback to process RX error events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
    (void) cb_data;

    LOG_INF("%s: error", __func__);

    tx_delay_ms = RX_ERR_TX_DELAY_MS;

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tx_conf_cb()
 *
 * @brief Callback to process TX confirmation events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
    
    (void) cb_data;

}
