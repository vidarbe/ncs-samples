/**
 * Hardware-triggered SPI sensor sampling with flash storage and BLE advertising.
 *
 * This application demonstrates high frequency and low power SPI sensor sampling
 *   1. SPIM — Performs DMA-based SPI transactions to read sensor data.
 *   2. GRTC — Generates periodic compare events at the
 *      configured sample rate, triggering each SPI transfer without CPU involvement.
 *   3. TIMER (counter mode) — Counts completed SPI transfers. When a full batch
 *      (SENSOR_SAMPLES_PER_REPORT) has been collected, it fires an interrupt.
 *   4. DPPI — Wires the above
 *      peripherals together in hardware so the SPI transfers can be started without
 *      without cpu involvement.
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/sys/__assert.h>

#include <nrfx_spim.h>
#include <nrfx_timer.h>
#include <hal/nrf_gpio.h>
#include <zephyr/drivers/timer/nrf_grtc_timer.h>
#include <nrfx_grtc.h>
#include <helpers/nrfx_gppi.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

/* To set up a repeating trigger interval with GRTC we must use channel 0, but this channel
 * is currently reserved to the zephyr system timer. Starting from SDK v3.3.0
 * the app should be be able to request this channel for itself. Until then, please cherry-pick
 * the commit from the PR linked below.
 * 
 * https://docs.nordicsemi.com/bundle/ps_nrf54LM20A/page/grtc.html#d616e367
 */
BUILD_ASSERT(DT_NODE_HAS_PROP(DT_NODELABEL(grtc), extended_channels),
	     "GRTC extended channels support is required (see "
	     "https://github.com/nrfconnect/sdk-zephyr/pull/3877).");
/*
 * Peripheral instance index must match the devicetree node labels used in app.overlay
 * (spi21, timer22).
 */
#define SPIM_INSTANCE_IDX 21
#define SPIM_NODE         DT_NODELABEL(_CONCAT(spi, SPIM_INSTANCE_IDX))

/* SPI pin mapping sourced from the "zephyr,user" devicetree node */
#define USER_NODE DT_PATH(zephyr_user)

#define SPIM_CLK_PIN  DT_GPIO_PIN(USER_NODE, spim_sck_gpios)
#define SPIM_MISO_PIN DT_GPIO_PIN(USER_NODE, spim_miso_gpios)
#define SPIM_MOSI_PIN DT_GPIO_PIN(USER_NODE, spim_mosi_gpios)
#define SPIM_SS_PIN   DT_GPIO_PIN(USER_NODE, spim_ss_gpios)

#define SPIM_CLK_PORT  DT_PROP(DT_GPIO_CTLR(USER_NODE, spim_sck_gpios), port)
#define SPIM_MISO_PORT DT_PROP(DT_GPIO_CTLR(USER_NODE, spim_miso_gpios), port)
#define SPIM_MOSI_PORT DT_PROP(DT_GPIO_CTLR(USER_NODE, spim_mosi_gpios), port)
#define SPIM_SS_PORT   DT_PROP(DT_GPIO_CTLR(USER_NODE, spim_ss_gpios), port)

#define TIMER_HW_COUNTER_INSTANCE_IDX 22
#define HW_COUNTER_NODE               DT_NODELABEL(_CONCAT(timer, TIMER_HW_COUNTER_INSTANCE_IDX))
/*
 * Samples are collected at SENSOR_SAMPLE_RATE_HZ and batched into groups of
 * SENSOR_SAMPLES_PER_REPORT before being handed to the main thread.
 */
#define SENSOR_SAMPLE_SIZE            sizeof(uint32_t)
#define SENSOR_REPORT_RATE_HZ         1
#define SENSOR_SAMPLE_RATE_HZ         1000
#define SENSOR_SAMPLE_PERIOD_US       (1000000 / SENSOR_SAMPLE_RATE_HZ)

BUILD_ASSERT((SENSOR_SAMPLE_RATE_HZ % SENSOR_REPORT_RATE_HZ) == 0,
	     "SENSOR_SAMPLE_RATE_HZ must be divisible by SENSOR_REPORT_RATE_HZ");

#define SENSOR_SAMPLES_PER_REPORT (SENSOR_SAMPLE_RATE_HZ / SENSOR_REPORT_RATE_HZ)

#define ADVERTISING_INTERVAL 1600
#define DEVICE_NAME          CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN      (sizeof(DEVICE_NAME) - 1)

const struct device *flash_dev = DEVICE_DT_GET_ONE(jedec_spi_nor);

/* Dummy TX payload — in a real application this would be the sensor read command. */
static uint8_t spim_tx[] = {0xAA, 0x55};
/*
 * Double-buffered RX area. While the DMA fills one buffer the main thread can
 * consume the other. Each buffer holds one full batch
 * of samples (SENSOR_SAMPLES_PER_REPORT × SENSOR_SAMPLE_SIZE bytes).
 */
static uint8_t spim_rx[2][SENSOR_SAMPLES_PER_REPORT * SENSOR_SAMPLE_SIZE];
static uint8_t *current_rx_buffer;

/*
 * rx_length is set to a single SENSOR_SAMPLE_SIZE
 * because SPIM EasyDMA array-list mode (NRFX_SPIM_FLAG_RX_POSTINC) automatically
 * increments the RX pointer by rx_length after each transaction.
 */
nrfx_spim_xfer_desc_t xfer_desc =
	NRFX_SPIM_XFER_TRX(spim_tx, sizeof(spim_tx), spim_rx[0], SENSOR_SAMPLE_SIZE);

struct sensor_sample_batch {
	uint8_t *data;
};

K_MSGQ_DEFINE(sensor_msgq, sizeof(struct sensor_sample_batch), 16, 4);

static nrfx_timer_t counter_timer_inst =
	NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(TIMER_HW_COUNTER_INSTANCE_IDX));
static nrfx_spim_t spim_inst = NRFX_SPIM_INSTANCE(NRF_SPIM_INST_GET(SPIM_INSTANCE_IDX));
/* GRTC compare channel used to generate the periodic sample trigger. */
static int32_t grtc_channel;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/*
 * During autonomous (DPPI-driven) operation the NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER
 * flag suppresses this callback
 */
static void spi_cb(nrfx_spim_event_t const *p_event, void *p_context)
{
	ARG_UNUSED(p_context);

	LOG_HEXDUMP_INF(p_event->xfer_desc.p_rx_buffer, p_event->xfer_desc.rx_length, "SPI RX:");
}

static void spi_init(void)
{
	int err;
	const uint32_t spim_clk = NRF_GPIO_PIN_MAP(SPIM_CLK_PORT, SPIM_CLK_PIN);
	const uint32_t spim_mosi = NRF_GPIO_PIN_MAP(SPIM_MOSI_PORT, SPIM_MOSI_PIN);
	const uint32_t spim_miso = NRF_GPIO_PIN_MAP(SPIM_MISO_PORT, SPIM_MISO_PIN);
	const uint32_t spim_ss = NRF_GPIO_PIN_MAP(SPIM_SS_PORT, SPIM_SS_PIN);

	/* Connect SPIM instance IRQ to irq handler */
	IRQ_CONNECT(DT_IRQN(SPIM_NODE), DT_IRQ(SPIM_NODE, priority), nrfx_spim_irq_handler,
		    &spim_inst, 0);

	nrfx_spim_config_t config =
		NRFX_SPIM_DEFAULT_CONFIG(spim_clk, spim_mosi, spim_miso, spim_ss);

	/*
	 * Change default configuration to prevent workaround for errata 8 from being enabled
	 * in the driver. Otherwise there will be an interrupt triggered for each transaction.
	 *
	 * Errata description:
	 * https://docs.nordicsemi.com/bundle/errata_nRF54LM20A_EngB/page/ERR/nRF54LM20A/EngineeringB/latest/anomaly_20A_8.html#anomaly_20A_8
	 *
	 * Workaround implementation:
	 * https://github.com/zephyrproject-rtos/hal_nordic/blob/70c1b32dd68753b0604beab377392ed5ebc19176/nrfx/drivers/src/nrfx_spim.c#L455
	 */
	config.frequency = NRFX_MHZ_TO_HZ(8);

	config.use_hw_ss = true;
	config.miso_pull = NRF_GPIO_PIN_PULLUP;

	err = nrfx_spim_init(&spim_inst, &config, spi_cb, NULL);
	if (err) {
		LOG_ERR("nrfx_spim_init() failed. (err: %d)", err);
		return;
	}
}
/*
 * Switches the DMA target to the idle buffer so the previously filled buffer
 * can be safely consumed by the main thread.
 */
static void spi_new_buffer_prepare(void)
{
	int err;

	/* Swap receive buffers */
	current_rx_buffer = (xfer_desc.p_rx_buffer == spim_rx[0]) ? spim_rx[1] : spim_rx[0];

	xfer_desc.p_rx_buffer = current_rx_buffer;

	err = nrfx_spim_xfer(&spim_inst, &xfer_desc,
			     NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER | NRFX_SPIM_FLAG_RX_POSTINC |
				     NRFX_SPIM_FLAG_REPEATED_XFER | NRFX_SPIM_FLAG_HOLD_XFER);
	if (err) {
		LOG_ERR("nrfx_spim_xfer() failed. (err: %d)", err);
	}
}
/*
 * Counter compare interrupt called when the number of samples has reached
 * SENSOR_SAMPLES_PER_REPORT.
 */
void batch_samples_complete(nrf_timer_event_t event_type, void *p_context)
{
	ARG_UNUSED(event_type);
	ARG_UNUSED(p_context);

	int err;
	struct sensor_sample_batch samples;

	samples.data = current_rx_buffer;
	spi_new_buffer_prepare();
	err = k_msgq_put(&sensor_msgq, &samples, K_NO_WAIT);
	if (err) {
		LOG_ERR("k_msgq_put() failed. (err: %d)", err);
	}
}

/*
 * Set up counter to keep track of the number of SPI transfers and to trigger an interrupt when the
 * SPI dma buffer has been filled.
 */
static void timer_counter_init(void)
{
	int err;
	nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG(NRFX_MHZ_TO_HZ(1));
	config.mode = NRF_TIMER_MODE_LOW_POWER_COUNTER;

	/* Connect TIMER COUNTER instance IRQ to irq handler */
	IRQ_CONNECT(DT_IRQN(HW_COUNTER_NODE), DT_IRQ(HW_COUNTER_NODE, priority),
		    nrfx_timer_irq_handler, &counter_timer_inst, 0);

	err = nrfx_timer_init(&counter_timer_inst, &config, batch_samples_complete);
	if (err) {
		LOG_ERR("nrfx_timer_init() for HW counter failed. (err: %d)", err);
		return;
	}

	nrfx_timer_extended_compare(&counter_timer_inst, NRF_TIMER_CC_CHANNEL0,
				    SENSOR_SAMPLES_PER_REPORT, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
				    true);
}
/*
 * Allocate GRTC channel 0 for periodic triggering.
 */
static void timer_trigger_init(void)
{
	grtc_channel = z_nrf_grtc_timer_ext_chan_alloc();
	__ASSERT_NO_MSG(grtc_channel > 0);
}

/* Use DPPI with gppi helper API HW to connect peripheral tasks and events.
 *   GRTC compare event -> SPIM START task
 *       Each periodic tick triggers an SPI transaction.
 *
 *   SPIM END event ->  TIMER COUNT task
 *       Each completed transaction increments the batch counter.
 */
static void hw_link_init(void)
{
	int err;
	nrfx_gppi_handle_t gppi_handles[2];

	/* Connection 1: GRTC tick -> start SPI transfer */
	err = nrfx_gppi_conn_alloc(z_nrf_grtc_timer_compare_evt_address_get(grtc_channel),
				   nrfx_spim_start_task_address_get(&spim_inst), &gppi_handles[0]);
	if (err) {
		LOG_ERR("gnrfx_gppi_conn_alloc() failed. (err: %d)", err);
		return;
	}

	/* Connection 2: SPI transfer complete -> increment counter */
	err = nrfx_gppi_conn_alloc(
		nrfx_spim_end_event_address_get(&spim_inst),
		nrfx_timer_task_address_get(&counter_timer_inst, NRF_TIMER_TASK_COUNT),
		&gppi_handles[1]);
	if (err) {
		LOG_ERR("gnrfx_gppi_conn_alloc() failed. (err: %d)", err);
		return;
	}

	nrfx_gppi_conn_enable(gppi_handles[0]);
	nrfx_gppi_conn_enable(gppi_handles[1]);
}

void sensor_sampling_start_single(void)
{
	int err;
	err = nrfx_spim_xfer(&spim_inst, &xfer_desc, 0);
	if (err) {
		LOG_ERR("nrfx_spim_xfer() failed. (err: %d)", err);
	}
}

void sensor_sampling_start_continuous(void)
{
	spi_new_buffer_prepare();
	nrfx_grtc_syscounter_cc_interval_reset(grtc_channel);
	nrfx_grtc_syscounter_cc_interval_set(grtc_channel, 1, SENSOR_SAMPLE_PERIOD_US);
}

void sensor_sampling_stop_continuous(void)
{
	nrfx_grtc_syscounter_cc_interval_reset(grtc_channel);
}

static void flash_init(void)
{
	int err;

	if (!device_is_ready(flash_dev)) {
		LOG_ERR("%s: device not ready.\n", flash_dev->name);
		return;
	}

	err = flash_erase(flash_dev, 0, CONFIG_SPI_NOR_FLASH_LAYOUT_PAGE_SIZE);
	if (err) {
		LOG_ERR("Failed to erase flash. (err: %d)", err);
	} else {
		LOG_INF("Succesfully erased %d kB", CONFIG_SPI_NOR_FLASH_LAYOUT_PAGE_SIZE / 1024);
	}
}

/**
 * Program sequence:
 *   1. Prepare one flash page for sensor data by erasing it
 *   2. Configure SPI, GRTC trigger, batch counter, and DPPI connections.
 *   3. Start BLE advertising.
 *   4. Run a single SPI transfer.
 *   5. Enter autonomous sampling mode.
 *   6. Main loop: dequeue completed batches and write them to flash.
 *   7. Stop sampling once the flash page is full.
 */
int main(void)
{
	int err;
	size_t write_offset = 0;
	size_t write_len = SENSOR_SAMPLES_PER_REPORT * SENSOR_SAMPLE_SIZE;
	struct sensor_sample_batch samples;

	flash_init();
	spi_init();
	timer_trigger_init();
	timer_counter_init();
	hw_link_init();

	bt_enable(NULL);
	err = bt_le_adv_start(BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN, ADVERTISING_INTERVAL,
					      ADVERTISING_INTERVAL, NULL),
			      ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("bt_le_adv_start() failed. (err: %d)", err);
	}
	/* Test a single CPU triggered transfer */
	sensor_sampling_start_single();
	k_msleep(100);
	/*
	 * Start repeated transfers which are autonomously triggered by the GRTC
	 * compare event.
	 */
	sensor_sampling_start_continuous();
	while (1) {
		k_msgq_get(&sensor_msgq, &samples, K_FOREVER);
		LOG_INF("Current write offset: 0x%x", write_offset);
		err = flash_write(flash_dev, write_offset, samples.data, write_len);
		if (err) {
			LOG_ERR("flash_write() failed to write to 0x%x. (err: %d)", write_offset,
				err);
		}
		write_offset += write_len;
		if (write_offset > CONFIG_SPI_NOR_FLASH_LAYOUT_PAGE_SIZE) {
			LOG_INF("Test completed. %d samples were written to FLASH",
				(write_offset - write_len) / SENSOR_SAMPLE_SIZE);
			sensor_sampling_stop_continuous();
			break;
		}
	}
	return 0;
}
