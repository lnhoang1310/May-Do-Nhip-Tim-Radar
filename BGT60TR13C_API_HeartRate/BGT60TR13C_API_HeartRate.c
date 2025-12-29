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

#define TAG "radar-hr"

#define NUM_SAMPLES_PER_CHIRP 128
#define NUM_CHIRPS_PER_FRAME 1
#define FRAME_RATE_HZ 200.0f // 1 / 0.005s

#define HR_BUFFER_LEN  1024// ~2.56s window
#define RANGE_BIN_START 10
#define RANGE_BIN_END 50

#define MIN_HEART_RATE 60.0f
#define MAX_HEART_RATE 100.0f

#define RANGE_ENERGY_THRESHOLD 4e6f
#define PHASE_VAR_HUMAN_MIN 0.05f
#define PHASE_VAR_HUMAN_MAX 2.0f
#define HUMAN_SCORE_MAX 100
#define HUMAN_SCORE_TH 5
#define HUMAN_SCORE_DECAY 1

#define PHASE_WINDOW 64         // 320 ms @200Hz


#define SPI_HOST SPI2_HOST
#define SPI_CLK_SPEED 20

#define SPI_CS_PIN GPIO_NUM_42
#define SPI_SCK_PIN GPIO_NUM_41
#define SPI_MOSI_PIN GPIO_NUM_40
#define SPI_MISO_PIN GPIO_NUM_39
#define RADAR_IRQ_PIN GPIO_NUM_6
#define BUZZER_PIN GPIO_NUM_7

/* ================= TYPES ================= */
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

/* ================= GLOBAL ================= */

static SemaphoreHandle_t radar_sem;

static float phase_buffer[HR_BUFFER_LEN];
static uint32_t phase_idx = 0;

static bool is_measuring = false;
static bool human_present = false;
static uint64_t measure_start_time = 0;
static uint64_t last_publish_time = 0;
static float minute_sum = 0.0f;
static uint32_t minute_count = 0;
static float current_minute_average = 0.0f;
uint16_t measure_count = 0;
float last_hr_average = 0.0f;

/* ================= ISR ================= */

void IRAM_ATTR gpio_radar_isr_handler(void *arg)
{
    xSemaphoreGiveFromISR(radar_sem, NULL);
}

/* ================= FFT ================= */

static uint32_t bit_reverse(uint32_t x, uint32_t log2n)
{
    uint32_t n = 0;
    for (uint32_t i = 0; i < log2n; i++)
    {
        n = (n << 1) | (x & 1);
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

/* ================= RANGE FFT ================= */

static void range_fft(int16_t *input, complex_t *output)
{
    for (int i = 0; i < NUM_SAMPLES_PER_CHIRP; i++)
    {
        output[i].re = (float)input[i];
        output[i].im = 0.0f;
    }
    fft_complex(output, NUM_SAMPLES_PER_CHIRP);
}

static int find_target_bin(complex_t *range)
{
    float max_mag = 0.0f;
    int max_bin = RANGE_BIN_START;

    for (int i = RANGE_BIN_START; i < RANGE_BIN_END; i++)
    {
        float mag = range[i].re * range[i].re +
                    range[i].im * range[i].im;
        if (mag > max_mag)
        {
            max_mag = mag;
            max_bin = i;
        }
    }
    return max_bin;
}

/*================== DETECT HUMAN================*/
static bool detect_human(
    complex_t *range,
    int target_bin,
    float current_phase)
{
    static float phase_hist[PHASE_WINDOW];
    static uint16_t phase_idx = 0;
    static float stable_bin = -1;      // Dùng float để trung bình
    static float bin_history[5] = {0}; // Lưu 5 frame gần nhất để smooth bin
    static int human_score = 0;

    /* ===== Energy gate ===== */
    float energy = range[target_bin].re * range[target_bin].re +
                   range[target_bin].im * range[target_bin].im;

    if (energy < RANGE_ENERGY_THRESHOLD)
    {
        human_score -= 1; // giảm nhẹ hơn
        if (human_score < 0)
            human_score = 0;
        stable_bin = -1;
        //return false;
    }

    /* ===== Bin smoothing ===== */
    for (int i = 4; i > 0; i--)
        bin_history[i] = bin_history[i - 1];
    bin_history[0] = (float)target_bin;

    float avg_bin = 0;
    for (int i = 0; i < 5; i++)
        avg_bin += bin_history[i];
    avg_bin /= 5.0f;

    if (stable_bin < 0)
        stable_bin = avg_bin;

    if (fabs(avg_bin - stable_bin) > 2.0f)
    {
        stable_bin = avg_bin;
        human_score -= 1; // giảm nhẹ hơn
        if (human_score < 0)
            human_score = 0;
        //return false;
    }

    /* ===== Phase variance ===== */
    phase_hist[phase_idx++] = current_phase;
    if (phase_idx >= PHASE_WINDOW)
        phase_idx = 0;

    float mean = 0.0f;
    for (int i = 0; i < PHASE_WINDOW; i++)
        mean += phase_hist[i];
    mean /= PHASE_WINDOW;

    float var = 0.0f;
    for (int i = 0; i < PHASE_WINDOW; i++)
    {
        float d = phase_hist[i] - mean;
        var += d * d;
    }
    var /= PHASE_WINDOW;

    /* ===== Scoring logic ===== */
    if (var > PHASE_VAR_HUMAN_MIN && var < PHASE_VAR_HUMAN_MAX)
    {
        // Khi phase variance hợp lý, tăng score mượt theo biên sin
        human_score += 1;
        if (human_score > HUMAN_SCORE_MAX)
            human_score = HUMAN_SCORE_MAX;
    }
    else if (var < PHASE_VAR_HUMAN_MIN)
    {
        // Không có người, giữ score gần 0, chỉ nhích 0-1 do nhiễu
        human_score -= 1;
        if (human_score < 0)
            human_score = 0;
    }
    else
    {
        // Phase variance quá lớn (nhiễu mạnh), giảm mạnh
        human_score -= 2;
        if (human_score < 0)
            human_score = 0;
    }

    return (human_score >= HUMAN_SCORE_TH);
}

/* ================= HEART RATE ================= */

static void calculate_heart_rate(void)
{
    static float bpm_smooth = 0.0f;
    const float fs = FRAME_RATE_HZ;

    /* Remove DC */
    float mean = 0.0f;
    for (int i = 0; i < HR_BUFFER_LEN; i++)
        mean += phase_buffer[i];
    mean /= HR_BUFFER_LEN;

    complex_t fft_buf[HR_BUFFER_LEN];

    for (int i = 0; i < HR_BUFFER_LEN; i++)
    {
        float x = phase_buffer[i] - mean;
        float w = 0.5f * (1.0f - cosf(2 * M_PI * i / (HR_BUFFER_LEN - 1)));
        fft_buf[i].re = x * w;
        fft_buf[i].im = 0.0f;
    }

    fft_complex(fft_buf, HR_BUFFER_LEN);

    float max_mag = 0.0f;
    int max_bin = -1;

    for (int i = 1; i < HR_BUFFER_LEN / 2; i++)
    {
        float freq = fs * i / HR_BUFFER_LEN;
        if (freq < 0.8f || freq > 2.2f)
            continue;

        float mag = fft_buf[i].re * fft_buf[i].re +
                    fft_buf[i].im * fft_buf[i].im;

        if (mag > max_mag)
        {
            max_mag = mag;
            max_bin = i;
        }
    }
    
    if (max_bin < 0 || max_mag < 1e-3f){
        return;
    }
    ESP_LOGW(TAG, "max_bin = %d, max_mag = %.2e", max_bin, max_mag);
    float bpm = (fs * max_bin / HR_BUFFER_LEN) * 60.0f;
    if(bpm_smooth == 0.0f){
        bpm_smooth = bpm;
    }
    bpm_smooth = 0.3*bpm + 0.7*bpm_smooth;

    uint64_t current_time = esp_timer_get_time() / 1000;
    minute_sum += bpm_smooth;
    minute_count++;
    if (current_time - last_publish_time >= 60000)
    {
        if (minute_count > 0)
        {
            current_minute_average = minute_sum / minute_count;
            current_minute_average = (last_hr_average * measure_count + current_minute_average) / (measure_count + 1);
            measure_count++;
            last_hr_average = current_minute_average;
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
}

/* ================= RADAR TASK ================= */

static void radar_task(void *arg)
{
    uint8_t *fifo = malloc(16384);
    int16_t adc[NUM_SAMPLES_PER_CHIRP];
    complex_t range[NUM_SAMPLES_PER_CHIRP];

    static float prev_phase = 0.0f;
    static float phase_acc = 0.0f;

    while (1)
    {
        xSemaphoreTake(radar_sem, portMAX_DELAY);

        if (!is_measuring)
        {
            xensiv_bgt60tr13c_soft_reset(XENSIV_BGT60TR13C_RESET_FIFO);
            xensiv_bgt60tr13c_start_frame_capture();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        uint32_t bytes = (NUM_SAMPLES_PER_CHIRP * 3) / 2;
        xensiv_bgt60tr13c_fifo_read(fifo, bytes, 1);

        uint32_t i = 0, s = 0;
        while (i + 2 < bytes && s < NUM_SAMPLES_PER_CHIRP)
        {
            uint16_t a = (fifo[i] << 4) | (fifo[i + 1] >> 4);
            uint16_t b = ((fifo[i + 1] & 0x0F) << 8) | fifo[i + 2];
            i += 3;

            adc[s++] = (int16_t)a - 2048;
            if (s < NUM_SAMPLES_PER_CHIRP)
                adc[s++] = (int16_t)b - 2048;
        }

        range_fft(adc, range);
        int bin = find_target_bin(range);

        float re = range[bin].re;
        float im = range[bin].im;

        float phase = atan2f(im, re);
        float dphi = phase - prev_phase;
        if (dphi > M_PI)
            dphi -= 2 * M_PI;
        if (dphi < -M_PI)
            dphi += 2 * M_PI;
        prev_phase = phase;

        /* ===== Human detection ===== */
        human_present = detect_human(range, bin, phase);

        /* ===== Only accumulate phase if human ===== */
        if (human_present)
        {
            phase_acc += dphi;
            phase_buffer[phase_idx++] = phase_acc;

            if (phase_idx >= HR_BUFFER_LEN)
            {
                phase_idx = 0;
                calculate_heart_rate();
            }
        }
        else
        {
            prev_phase = phase;
        }

        xensiv_bgt60tr13c_start_frame_capture();
    }
    free(fifo);
    vTaskDelete(NULL);
}

/* ================= INIT ================= */

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

            for (int round = 0; round < 3; round++)
            {
                buzzer_beep(beep_count);
                vTaskDelay(pdMS_TO_TICKS(20000));
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

    radar_sem = xSemaphoreCreateBinary();
    gpio_install_isr_service(0);
    gpio_isr_handler_add(RADAR_IRQ_PIN, gpio_radar_isr_handler, NULL);

    xTaskCreate(radar_task, "radar_task", 16384, NULL, 5, NULL);
    xTaskCreate(buzzer_task, "buzzer_task", 2048, NULL, 3, NULL);
    // buzzer_cmd_t cmd = BUZZER_HIGH_HR;
    // xQueueSend(buzzer_queue, &cmd, 0);
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
        measure_count = 0;
        last_hr_average = 0.0f;
        ESP_LOGI(TAG, "Measuring STOPPED by MQTT");
        xensiv_bgt60tr13c_start_frame_capture();
    }
}
