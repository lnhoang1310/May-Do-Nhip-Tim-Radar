#include "BGT60TR13C_API_HeartRate.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "bgt60tr13c_driver.h"
#include "bgt60tr13c_regs.h"
#include <math.h>
#include "Service_MQTT.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "radar-vital";

#define MIN_HEART_RATE 60
#define MAX_HEART_RATE 100

#define NUM_SAMPLES_PER_CHIRP 128
#define NUM_CHIRPS_PER_FRAME 64
#define NUM_RX 1
#define FRAME_RATE_HZ 25

#define HR_BUFFER_LEN 128
#define RANGE_BIN_START 10
#define RANGE_BIN_END 50

#define SPI_HOST SPI2_HOST
#define SPI_CLK_SPEED 20

#define SPI_CS_PIN GPIO_NUM_42
#define SPI_SCK_PIN GPIO_NUM_41
#define SPI_MOSI_PIN GPIO_NUM_40
#define SPI_MISO_PIN GPIO_NUM_39
#define RADAR_IRQ_PIN GPIO_NUM_6
#define RADAR_RESET_PIN GPIO_NUM_4

#define BUZZER_PIN GPIO_NUM_7

SemaphoreHandle_t xSemaphore = NULL;
static float phase_buffer[HR_BUFFER_LEN];
static uint32_t phase_idx = 0;
static float prev_phase = 0.0f;
static bool is_measuring = false;

static uint64_t measure_start_time = 0;
static uint64_t last_publish_time = 0;
static float minute_sum = 0.0f;
static uint32_t minute_count = 0;
static float current_minute_average = 0.0f;

typedef enum
{
    BUZZER_NONE = 0,
    BUZZER_LOW_HR,
    BUZZER_HIGH_HR
} buzzer_cmd_t;
static QueueHandle_t buzzer_queue;

typedef struct
{
    float re;
    float im;
} complex_t;

void IRAM_ATTR gpio_radar_isr_handler(void *arg)
{
    xSemaphoreGiveFromISR(xSemaphore, NULL);
}

static uint32_t bit_reverse(uint32_t x, uint32_t log2n)
{
    uint32_t n = 0;
    for (uint32_t i = 0; i < log2n; i++)
    {
        n <<= 1;
        n |= (x & 1);
        x >>= 1;
    }
    return n;
}

static void fft_complex(complex_t *buf, uint32_t n)
{
    uint32_t log2n = 0;
    while ((1U << log2n) < n)
        log2n++;

    for (uint32_t i = 0; i < n; i++)
    {
        uint32_t j = bit_reverse(i, log2n);
        if (j > i)
        {
            complex_t tmp = buf[i];
            buf[i] = buf[j];
            buf[j] = tmp;
        }
    }

    for (uint32_t s = 1; s <= log2n; s++)
    {
        uint32_t m = 1U << s;
        uint32_t m2 = m >> 1;
        float theta = -2.0f * M_PI / m;
        complex_t wm = {cosf(theta), sinf(theta)};

        for (uint32_t k = 0; k < n; k += m)
        {
            complex_t w = {1.0f, 0.0f};
            for (uint32_t j = 0; j < m2; j++)
            {
                complex_t t = {
                    w.re * buf[k + j + m2].re - w.im * buf[k + j + m2].im,
                    w.re * buf[k + j + m2].im + w.im * buf[k + j + m2].re};
                complex_t u = buf[k + j];

                buf[k + j].re = u.re + t.re;
                buf[k + j].im = u.im + t.im;
                buf[k + j + m2].re = u.re - t.re;
                buf[k + j + m2].im = u.im - t.im;

                float wr = w.re * wm.re - w.im * wm.im;
                w.im = w.re * wm.im + w.im * wm.re;
                w.re = wr;
            }
        }
    }
}

static void range_fft(int16_t *input, complex_t *output, int n)
{
    for (int i = 0; i < n; i++)
    {
        output[i].re = (float)input[i];
        output[i].im = 0.0f;
    }
    fft_complex(output, n);
}

static int find_target_bin(complex_t *range_profile)
{
    float max_mag = 0.0f;
    int max_bin = RANGE_BIN_START;
    for (int bin = RANGE_BIN_START; bin < RANGE_BIN_END && bin < NUM_SAMPLES_PER_CHIRP; bin++)
    {
        float mag = range_profile[bin].re * range_profile[bin].re + range_profile[bin].im * range_profile[bin].im;
        if (mag > max_mag)
        {
            max_mag = mag;
            max_bin = bin;
        }
    }
    return max_bin;
}

float diff;
static float unwrap_phase(float current_phase)
{
    diff = current_phase - prev_phase;

    while (diff > M_PI)
        diff -= 2.0f * M_PI;
    while (diff < -M_PI)
        diff += 2.0f * M_PI;

    prev_phase += diff;
    return prev_phase;
}

static float smoothed_bpm = 0.0f;
static float previous_bpm = 70.0f;
void calculate_heart_rate(void)
{

    float spectrum[HR_BUFFER_LEN / 2];
    complex_t buf[HR_BUFFER_LEN];

    for (int i = 0; i < HR_BUFFER_LEN; i++)
    {
        buf[i].re = phase_buffer[i];
        buf[i].im = 0.0f;
    }
    fft_complex(buf, HR_BUFFER_LEN);

    for (int i = 0; i < HR_BUFFER_LEN / 2; i++)
    {
        spectrum[i] = sqrtf(buf[i].re * buf[i].re + buf[i].im * buf[i].im);
    }

    float max_val = 0.0f;
    int max_bin = 0;
    for (int i = 2; i < HR_BUFFER_LEN / 2; i++)
    {
        float freq = (float)i * FRAME_RATE_HZ / HR_BUFFER_LEN;
        if (freq >= 0.8f && freq <= 2.5f)
        {
            if (spectrum[i] > max_val)
            {
                max_val = spectrum[i];
                max_bin = i;
            }
        }
    }

    float raw_bpm = (float)max_bin * FRAME_RATE_HZ * 60.0f / HR_BUFFER_LEN;
    float raw_freq = (float)max_bin * FRAME_RATE_HZ / HR_BUFFER_LEN;
    float limited_bpm = raw_bpm;

    if (smoothed_bpm == 0.0f)
    {
        limited_bpm = raw_bpm;
    }
    else
    {
        float diff = raw_bpm - previous_bpm;
        if (diff > 40.0f)
            limited_bpm = previous_bpm;
    }

    previous_bpm = limited_bpm;
    if (smoothed_bpm == 0.0f)
        smoothed_bpm = limited_bpm;
    else
        smoothed_bpm = 0.8f * smoothed_bpm + 0.2f * limited_bpm;
    uint64_t current_time = esp_timer_get_time() / 1000;
    minute_sum += smoothed_bpm;
    minute_count++;
    if (current_time - last_publish_time >= 60000)
    {
        if (minute_count > 0)
        {
            current_minute_average = minute_sum / minute_count;
            MQTT_Publish((current_minute_average < MIN_HEART_RATE) ? "L" : ((current_minute_average > MAX_HEART_RATE) ? "H" : "M"), current_minute_average);
            buzzer_cmd_t cmd = BUZZER_NONE;

            if (current_minute_average < MIN_HEART_RATE)
                cmd = BUZZER_LOW_HR;
            else if (current_minute_average > MAX_HEART_RATE)
                cmd = BUZZER_HIGH_HR;

            if (cmd != BUZZER_NONE)
                xQueueSend(buzzer_queue, &cmd, 0);

            uint64_t elapsed_minutes = (current_time - measure_start_time) / 60000 + 1;
            ESP_LOGI(TAG, "Published MINUTE %llu average: %.1f BPM (%lu samples)",
                     elapsed_minutes, current_minute_average, minute_count);
        }
        minute_sum = 0.0f;
        minute_count = 0;
        last_publish_time = current_time;
    }
    ESP_LOGI(TAG, "Nhịp tim: %.1f BPM (tức thì: %.1f BPM | peak: %.2f Hz)",
             smoothed_bpm, limited_bpm, raw_freq);
}

void radar_task(void *arg)
{
    uint8_t *fifo_raw = malloc(16384);
    if (!fifo_raw)
    {
        ESP_LOGE(TAG, "Malloc fifo_raw failed");
        vTaskDelete(NULL);
    }

    int16_t adc_samples[NUM_SAMPLES_PER_CHIRP];
    complex_t range_profile[NUM_SAMPLES_PER_CHIRP];

    uint32_t frame_count = 0;

    while (1)
    {
        xSemaphoreTake(xSemaphore, portMAX_DELAY);
        if (!is_measuring)
        {
            xensiv_bgt60tr13c_soft_reset(XENSIV_BGT60TR13C_RESET_FIFO);
            xensiv_bgt60tr13c_start_frame_capture();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        frame_count++;
        uint32_t full_fifo_bytes = (NUM_SAMPLES_PER_CHIRP * NUM_CHIRPS_PER_FRAME * 3) / 2;
        esp_err_t ret = xensiv_bgt60tr13c_fifo_read(fifo_raw, full_fifo_bytes, 1);

        if (ret != ESP_OK)
            ESP_LOGW(TAG, "FIFO read failed this frame, continuing...");

        uint32_t sample_idx = 0;
        for (uint32_t i = 0; i < full_fifo_bytes && sample_idx < NUM_SAMPLES_PER_CHIRP; i += 3)
        {
            uint16_t s0 = (fifo_raw[i] << 4) | (fifo_raw[i + 1] >> 4);
            uint16_t s1 = ((fifo_raw[i + 1] & 0x0F) << 8) | fifo_raw[i + 2];

            if (sample_idx < NUM_SAMPLES_PER_CHIRP)
                adc_samples[sample_idx++] = (int16_t)s0 - 2048;

            if (sample_idx < NUM_SAMPLES_PER_CHIRP)
                adc_samples[sample_idx++] = (int16_t)s1 - 2048;
        }

        range_fft(adc_samples, range_profile, NUM_SAMPLES_PER_CHIRP);
        int target_bin = find_target_bin(range_profile);

        float max_mag = 0.0f;
        for (int b = RANGE_BIN_START; b < RANGE_BIN_END && b < NUM_SAMPLES_PER_CHIRP; b++)
        {
            float mag = range_profile[b].re * range_profile[b].re + range_profile[b].im * range_profile[b].im;
            if (mag > max_mag)
                max_mag = mag;
        }

        if (target_bin >= RANGE_BIN_START && max_mag > 1e6)
        {
            float phase = atan2f(range_profile[target_bin].im, range_profile[target_bin].re);
            unwrap_phase(phase);

            phase_buffer[phase_idx++] = diff;
            if (phase_idx >= HR_BUFFER_LEN)
            {
                phase_idx = 0;
                calculate_heart_rate();
                prev_phase = 0.0f;
            }
        }
        xensiv_bgt60tr13c_start_frame_capture();
    }

    free(fifo_raw);
    vTaskDelete(NULL);
}

static void buzzer_beep(uint8_t count)
{
    for (int i = 0; i < count; i++)
    {
        gpio_set_level(BUZZER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(BUZZER_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(800)); // tổng 1s
    }
}

static void buzzer_init()
{
    gpio_config_t conf = {
        .pin_bit_mask = (1UL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&conf);
    gpio_set_level(BUZZER_PIN, 0);
}

static void buzzer_task(void *arg)
{
    buzzer_cmd_t cmd;

    while (1)
    {
        if (xQueueReceive(buzzer_queue, &cmd, portMAX_DELAY))
        {
            int beep_count = 0;

            if (cmd == BUZZER_LOW_HR)
                beep_count = 3;
            else if (cmd == BUZZER_HIGH_HR)
                beep_count = 5;
            else
                continue;

            // 🔁 3 ĐỢT / PHÚT
            for (int round = 0; round < 3; round++)
            {
                buzzer_beep(beep_count);
                vTaskDelay(pdMS_TO_TICKS(20000)); // cách nhau ~20s
            }
        }
    }
}

void BGT60TR13C_Init(spi_host_device_t spi)
{
    buzzer_init();
    buzzer_queue = xQueueCreate(4, sizeof(buzzer_cmd_t));
    is_measuring = false;
    spi_bus_config_t bus_config = {
        .miso_io_num = SPI_MISO_PIN,
        .mosi_io_num = SPI_MOSI_PIN,
        .sclk_io_num = SPI_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16384};

    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &bus_config, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = SPI_CLK_SPEED * 1000 * 1000,
        .mode = 0,
        .spics_io_num = SPI_CS_PIN,
        .queue_size = 7,
    };

    ESP_ERROR_CHECK(xensiv_bgt60tr13c_init(SPI_HOST, &dev_config));
    ESP_ERROR_CHECK(xensiv_bgt60tr13c_configure());

    // Cấu hình endless frames nếu cần (giữ code cũ của bạn)
    ESP_LOGI(TAG, "Setting CCR2 for endless frames.");
    uint32_t ccr2_val = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_CCR2);
    ESP_LOGI(TAG, "Original CCR2 value: 0x%08lX", ccr2_val);

    uint32_t ccr2_frame_len_part = (ccr2_val & 0x00FFF000);
    uint32_t ccr2_to_write = ccr2_frame_len_part;

    ESP_LOGI(TAG, "CCR2 value to write for endless frames (MAX_FRAME_CNT=0): 0x%08lX", ccr2_to_write);
    esp_err_t ret = xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_CCR2, ccr2_to_write, true);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set CCR2 for endless frames: %s.", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Successfully set CCR2 for endless frames.");
    }
    uint32_t new_ccr2_val = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_CCR2);
    ESP_LOGI(TAG, "New CCR2 value read back: 0x%08lX", new_ccr2_val); // This is always different than what I set for some reason

    if ((new_ccr2_val & 0x00000FFF) != 0)
    {
        ESP_LOGW(TAG, "MAX_FRAME_CNT in CCR2 (0x%08lX) is not 0 after setting! Value: 0x%lX. Continuous mode might not be enabled.", new_ccr2_val, (new_ccr2_val & 0xFFF));
    }
    else
    {
        ESP_LOGI(TAG, "MAX_FRAME_CNT in CCR2 is 0. Configured for endless frames.");
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RADAR_IRQ_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE};
    gpio_config(&io_conf);

    xSemaphore = xSemaphoreCreateBinary();
    gpio_install_isr_service(0);
    gpio_isr_handler_add(RADAR_IRQ_PIN, gpio_radar_isr_handler, NULL);

    xTaskCreate(radar_task, "radar_task", 16384, NULL, 5, NULL);
    xTaskCreate(buzzer_task, "buzzer_task", 2048, NULL, 3, NULL);
}
void Start_Measuring(void)
{
    if (!is_measuring)
    {
        is_measuring = true;
        ESP_LOGI(TAG, "Measuring STARTED by MQTT");

        uint64_t now = esp_timer_get_time() / 1000;
        measure_start_time = now;
        last_publish_time = now;
        minute_sum = 0.0f;
        minute_count = 0;
        current_minute_average = 0.0f;
        phase_idx = 0;
        xensiv_bgt60tr13c_start_frame_capture();
    }
}

void Stop_Measuring(void)
{
    if (is_measuring)
    {
        is_measuring = false;
        ESP_LOGI(TAG, "Measuring STOPPED by MQTT");
        xensiv_bgt60tr13c_start_frame_capture();
    }
}
