//Main utilizado en el esp32 para entrega 3 del proyecto UVGrow
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "cJSON.h"
#include "config_uvgrow.h"
#include "dht11.h"
#include "rc522.h"
#include "driver/rc522_spi.h"
#include "rc522_picc.h"
#ifndef UV_ENABLE_DHT11
#define UV_ENABLE_DHT11 0
#endif
#ifndef UV_ENABLE_LM35
#define UV_ENABLE_LM35 0
#endif
#ifndef UV_ENABLE_SOIL
#define UV_ENABLE_SOIL 0
#endif
#ifndef UV_ENABLE_WATER
#define UV_ENABLE_WATER 0
#endif
#ifndef UV_ENABLE_LDR
#define UV_ENABLE_LDR 0
#endif
#ifndef UV_ENABLE_PIR
#define UV_ENABLE_PIR 0
#endif
#ifndef UV_ENABLE_MQ2
#define UV_ENABLE_MQ2 0
#endif
#ifndef UV_ENABLE_RFID
#define UV_ENABLE_RFID 0
#endif
#ifndef UV_ENABLE_REED
#define UV_ENABLE_REED 0
#endif
#ifndef UV_REED_GPIO
#define UV_REED_GPIO 16
#endif
#ifndef UV_REED_INTERVAL_MS
#define UV_REED_INTERVAL_MS 100
#endif
#define UV_PUMP_COOLDOWN_SAFE_MS 300000
#ifndef UV_PUMP_MAX_MS
#define UV_PUMP_MAX_MS 2000
#endif
#define UV_MAX_PULSE_MS 2000
#define UV_DEMO_QUEUE_LENGTH 4
#define UV_DEMO_CMD_MAX 192
#define UV_HAS_ADC (UV_ENABLE_LM35 || UV_ENABLE_SOIL || UV_ENABLE_WATER || UV_ENABLE_LDR || UV_ENABLE_MQ2)
#define DHT_INTERVAL_MS 5000
#define SOIL_INTERVAL_MS 30000
#define WATER_INTERVAL_MS 5000
#define LDR_INTERVAL_MS 5000
#define MQ2_INTERVAL_MS 10000
#define LM35_INTERVAL_MS 10000
#define PIR_INTERVAL_MS 200
#define PIR_WARMUP_MS 60000
static const char *TAG = "UVGrow";
static esp_mqtt_client_handle_t mqtt = NULL;
static volatile bool mqtt_ready = false;
static bool mqtt_started = false;
#define UV_TELEMETRY_INTERVAL_MS 30000
#define UV_WATER_MIN_ADC 750
#define UV_WATER_REARM_ADC 800
#define UV_SOIL_DRY_ADC 4059
#define UV_SOIL_WET_ADC 1220
#define UV_LDR_DARK_ADC 4095
#define UV_LDR_BRIGHT_ADC 0
#define UV_MQ2_HIGH_ADC 300
#define UV_MQ2_CLEAR_ADC 260
#define UV_SENSOR_STALE_MS 90000

typedef struct {
    const char *id;
    float soil_min, soil_max, temp_min, temp_max, rh_max, light_min;
} uv_profile_t;
static const uv_profile_t profiles[] = {
    {"espada_africana", 25, 65, 21, 27, 50, 30},
    {"siempreverde_china", 45, 85, 20, 27, 60, 25}
};
static int profile_index = -1;
static bool water_lock = true;
static int64_t water_sample_ms = -1;
static int64_t soil_sample_ms = -1;
static float soil_pct = 0;
static float water_pct = 0;
static float light_pct = 0;
static float clamp_pct(float value) { return fmaxf(0, fminf(100, value)); }
static void restore_profile(void) {
    nvs_handle_t h;
    if (nvs_open("uvgrow", NVS_READONLY, &h) != ESP_OK) return;
    char id[40] = {0}; size_t len = sizeof(id);
    if (nvs_get_str(h, "profile", id, &len) == ESP_OK) {
        for (int i = 0; i < 2; ++i)
            if (strcmp(id, profiles[i].id) == 0) profile_index = i;
    }
    nvs_close(h);
}
static bool select_profile(const char *id) {
    if (!id) return false;
    for (int i = 0; i < 2; ++i) {
        if (strcmp(id, profiles[i].id) != 0) continue;
        nvs_handle_t h;
        if (nvs_open("uvgrow", NVS_READWRITE, &h) != ESP_OK) return false;
        esp_err_t err = nvs_set_str(h, "profile", id);
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
        if (err != ESP_OK) return false;
        profile_index = i;
        ESP_LOGI("UVGrow", "Perfil activo: %s", id);
        return true;
    }
    return false;
}

#if UV_HAS_ADC
static adc_oneshot_unit_handle_t adc = NULL;
#endif
#if UV_ENABLE_LM35
static adc_cali_handle_t lm35_cali = NULL;
static bool lm35_cali_valida = false;
static bool lm35_valido = false;
static float temperatura_lm35 = 0.0f;
static int raw_lm35 = 0;
static int lm35_mv = 0;
#endif
#if UV_ENABLE_DHT11
static float temperatura_ambiente = 0.0f;
static float humedad_ambiente = 0.0f;
static bool dht11_valido = false;
#endif
#if UV_ENABLE_SOIL
static int raw_soil = 0;
static bool soil_valido = false;
#endif
#if UV_ENABLE_WATER
static int raw_water = 0;
static bool water_valido = false;
#endif
#if UV_ENABLE_LDR
static int raw_ldr = 0;
static bool ldr_valido = false;
#endif
#if UV_ENABLE_PIR
static bool motion = false;
static bool pir_ready = false;
static bool pir_estado_anterior = false;
static int64_t pir_inicio_ms = 0;
#endif
#if UV_ENABLE_MQ2
static int raw_mq2 = 0;
static bool mq2_valido = false;
static int64_t mq2_inicio_ms = 0;
#endif
#if UV_ENABLE_REED
#define UV_REED_DEBOUNCE_MS 100
static bool tapa_cerrada = false;
static bool reed_inicializado = false;
static bool reed_candidato = false;
static int64_t reed_candidato_desde = 0;
#endif
#if UV_ENABLE_RFID
#define UV_RFID_MISO GPIO_NUM_19
#define UV_RFID_MOSI GPIO_NUM_23
#define UV_RFID_SCK GPIO_NUM_18
#define UV_RFID_SS GPIO_NUM_17
static rc522_driver_handle_t rfid_driver;
static rc522_handle_t rfid_scanner;
static rc522_spi_config_t rfid_spi_config = {
    .host_id = SPI3_HOST,
    .bus_config = &(spi_bus_config_t) {
        .miso_io_num = UV_RFID_MISO,
        .mosi_io_num = UV_RFID_MOSI,
        .sclk_io_num = UV_RFID_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    },
    .dev_config = {
        .spics_io_num = UV_RFID_SS
    },
    .rst_io_num = -1
};
static void on_rfid(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    (void)id;
    rc522_picc_state_changed_event_t *event = (rc522_picc_state_changed_event_t *)data;
    if (event == NULL || event->picc == NULL ||
        event->picc->state != RC522_PICC_STATE_ACTIVE) return;
    char uid[RC522_PICC_UID_STR_BUFFER_SIZE_MAX] = {0};
    esp_err_t err = rc522_picc_uid_to_str(&event->picc->uid, uid, sizeof(uid));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo leer UID RFID: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "RFID UID: %s", uid);
    if (!mqtt_ready || mqtt == NULL) return;
    cJSON *root = cJSON_CreateObject();
    if (!root) return;
    cJSON_AddStringToObject(root, "device_id", "uvgrow_nodo01");
    cJSON_AddStringToObject(root, "event", "rfid_detectado");
    cJSON_AddStringToObject(root, "uid", uid);
    cJSON_AddNumberToObject(root, "uptime_ms", (double)(esp_timer_get_time() / 1000));
    char *payload = cJSON_PrintUnformatted(root);
    if (payload) {
        int msg_id = esp_mqtt_client_publish(mqtt, UV_TOPIC_EVENTS, payload, 0, 1, 0);
        if (msg_id < 0) ESP_LOGW(TAG, "No se pudo publicar evento RFID");
        free(payload);
    }
    cJSON_Delete(root);
}
static void init_rfid(void)
{
    ESP_ERROR_CHECK(rc522_spi_create(&rfid_spi_config, &rfid_driver));
    ESP_ERROR_CHECK(rc522_driver_install(rfid_driver));
    rc522_config_t cfg = { .driver = rfid_driver };
    ESP_ERROR_CHECK(rc522_create(&cfg, &rfid_scanner));
    ESP_ERROR_CHECK(rc522_register_events(
        rfid_scanner, RC522_EVENT_PICC_STATE_CHANGED, on_rfid, NULL));
    ESP_ERROR_CHECK(rc522_start(rfid_scanner));
    ESP_LOGI(TAG, "RFID RC522 inicializado");
}
#endif
static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}
#if UV_ENABLE_REED
static void init_reed(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << UV_REED_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    reed_candidato = gpio_get_level((gpio_num_t)UV_REED_GPIO) == 0;
    reed_candidato_desde = now_ms();
    reed_inicializado = false;
    ESP_LOGI(TAG, "Reed switch inicializado");
}
static void actualizar_reed(void)
{
    int64_t ahora = now_ms();
    bool lectura = gpio_get_level((gpio_num_t)UV_REED_GPIO) == 0;
    if (lectura != reed_candidato) {
        reed_candidato = lectura;
        reed_candidato_desde = ahora;
        return;
    }
    if (ahora - reed_candidato_desde < UV_REED_DEBOUNCE_MS) return;
    if (!reed_inicializado) {
        tapa_cerrada = lectura;
        reed_inicializado = true;
        ESP_LOGI(TAG, "Reed estable: tapa %s", tapa_cerrada ? "cerrada" : "abierta");
        return;
    }
    if (lectura == tapa_cerrada) return;
    tapa_cerrada = lectura;
    ESP_LOGI(TAG, "Tapa del deposito %s", tapa_cerrada ? "cerrada" : "abierta");
    if (!mqtt_ready || mqtt == NULL) return;
    cJSON *evento = cJSON_CreateObject();
    if (evento == NULL) return;
    cJSON_AddStringToObject(evento, "device_id", "uvgrow_nodo01");
    cJSON_AddStringToObject(evento, "event", tapa_cerrada ? "tapa_deposito_cerrada" : "tapa_deposito_abierta");
    cJSON_AddBoolToObject(evento, "tapa_deposito_cerrada", tapa_cerrada);
    cJSON_AddNumberToObject(evento, "uptime_ms", (double)ahora);
    char *payload = cJSON_PrintUnformatted(evento);
    if (payload != NULL) {
        int id = esp_mqtt_client_publish(mqtt, UV_TOPIC_EVENTS, payload, 0, 1, 0);
        if (id < 0) ESP_LOGW(TAG, "Error publicando evento reed");
        free(payload);
    }
    cJSON_Delete(evento);
}
#endif
static void init_outputs(void)
{
    gpio_set_level((gpio_num_t)UV_RELAY_PUMP_GPIO, 1);
    gpio_set_level((gpio_num_t)UV_RELAY_FAN_GPIO, 1);
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << UV_RELAY_PUMP_GPIO) | (1ULL << UV_RELAY_FAN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)UV_RELAY_PUMP_GPIO, 1));
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)UV_RELAY_FAN_GPIO, 1));
    ESP_LOGI(TAG, "Reles apagados");
}
typedef struct {
    char payload[UV_DEMO_CMD_MAX];
} uv_command_t;
static QueueHandle_t command_queue = NULL;
static bool pump_on = false;
static bool fan_on = false;
static int64_t pump_until_ms = 0;
static int64_t fan_until_ms = 0;
static int64_t pump_last_stop_ms = 0;
static bool pump_has_stopped = false;
typedef struct { const char *name; bool active; unsigned hits; unsigned clears; } uv_alert_t;
enum { ALERT_WATER, ALERT_MQ2, ALERT_SOIL_DRY, ALERT_SOIL_WET, ALERT_TEMP_HIGH, ALERT_TEMP_LOW, ALERT_RH_HIGH, ALERT_LIGHT_LOW, ALERT_COUNT };
static uv_alert_t alerts[ALERT_COUNT] = {
    {"agua_insuficiente",false,0,0}, {"variacion_mq2",false,0,0},
    {"suelo_seco",false,0,0}, {"suelo_humedo",false,0,0},
    {"temperatura_alta",false,0,0}, {"temperatura_baja",false,0,0},
    {"humedad_ambiental_alta",false,0,0}, {"luz_insuficiente",false,0,0}
};
static void reset_profile_alerts(void) {
    for (int i = ALERT_SOIL_DRY; i < ALERT_COUNT; ++i) {
        alerts[i].active = false;
        alerts[i].hits = alerts[i].clears = 0;
    }
}
static void update_alert(int index, bool triggered, bool cleared, double value) {
    uv_alert_t *a = &alerts[index];
    if (!a->active) {
        a->clears = 0;
        a->hits = triggered ? a->hits + 1 : 0;
        if (a->hits < 2) return;
        a->active = true; a->hits = 0;
    } else {
        a->hits = 0;
        a->clears = cleared ? a->clears + 1 : 0;
        if (a->clears < 2) return;
        a->active = false; a->clears = 0;
    }
    ESP_LOGW(TAG, "ALERTA %s: %s (%.1f)", a->name, a->active ? "ACTIVA" : "RESUELTA", value);
    if (!mqtt_ready || mqtt == NULL) return;
    cJSON *j = cJSON_CreateObject();
    if (!j) return;
    cJSON_AddStringToObject(j, "device_id", "uvgrow_nodo01");
    cJSON_AddStringToObject(j, "event", a->active ? "alerta_activada" : "alerta_resuelta");
    cJSON_AddStringToObject(j, "alert", a->name);
    cJSON_AddNumberToObject(j, "value", value);
    cJSON_AddStringToObject(j, "profile_id", profile_index >= 0 ? profiles[profile_index].id : "sin_perfil");
    cJSON_AddNumberToObject(j, "uptime_ms", (double)now_ms());
    char *payload = cJSON_PrintUnformatted(j);
    if (payload) { esp_mqtt_client_publish(mqtt, UV_TOPIC_EVENTS, payload, 0, 1, 0); free(payload); }
    cJSON_Delete(j);
}
static void publish_actuator_event(const char *event_name, const char *request_id, const char *reason)
{
    ESP_LOGI(TAG, "ACTUADOR: %s motivo=%s", event_name, reason ? reason : "ninguno");
    if (!mqtt_ready || mqtt == NULL) return;
    cJSON *root = cJSON_CreateObject();
    if (!root) return;
    cJSON_AddStringToObject(root, "device_id", "uvgrow_nodo01");
    cJSON_AddStringToObject(root, "event", event_name);
    cJSON_AddStringToObject(root, "request_id", request_id ? request_id : "");
    cJSON_AddStringToObject(root, "reason", reason ? reason : "");
    cJSON_AddBoolToObject(root, "pump_on", pump_on);
    cJSON_AddBoolToObject(root, "fan_on", fan_on);
    cJSON_AddNumberToObject(root, "uptime_ms", (double)now_ms());
    char *payload = cJSON_PrintUnformatted(root);
    if (payload) {
        esp_mqtt_client_publish(mqtt, UV_TOPIC_EVENTS, payload, 0, 1, 0);
        free(payload);
    }
    cJSON_Delete(root);
}
static void pump_stop(const char *reason)
{
    gpio_set_level((gpio_num_t)UV_RELAY_PUMP_GPIO, 1);
    if (!pump_on) return;
    pump_on = false;
    pump_until_ms = 0;
    pump_last_stop_ms = now_ms();
    pump_has_stopped = true;
    publish_actuator_event("bomba_apagada", "", reason);
}
static void fan_stop(const char *reason)
{
    gpio_set_level((gpio_num_t)UV_RELAY_FAN_GPIO, 1);
    if (!fan_on) return;
    fan_on = false;
    fan_until_ms = 0;
    publish_actuator_event("ventilador_apagado", "", reason);
}
static const char *pump_block_reason(void) {
    if (profile_index < 0) return "sin_perfil";
#if UV_ENABLE_WATER
    if (!water_valido || water_sample_ms < 0 || now_ms() - water_sample_ms > UV_SENSOR_STALE_MS)
        return "agua_sin_lectura_valida";
    if (water_lock || raw_water < UV_WATER_MIN_ADC) return "agua_insuficiente";
#else
    return "sensor_agua_deshabilitado";
#endif
#if UV_ENABLE_SOIL
    if (!soil_valido || soil_sample_ms < 0 || now_ms() - soil_sample_ms > UV_SENSOR_STALE_MS)
        return "suelo_sin_lectura_valida";
    if (soil_pct > profiles[profile_index].soil_max) return "suelo_demasiado_humedo";
#else
    return "sensor_suelo_deshabilitado";
#endif
    return NULL;
}
static void service_actuators(void)
{
    int64_t t = now_ms();
    if (pump_on && t >= pump_until_ms) pump_stop("tiempo_maximo");
    if (pump_on && pump_block_reason() != NULL) pump_stop("proteccion_sensor");
    if (fan_on && t >= fan_until_ms) fan_stop("tiempo_maximo");
    if (!mqtt_ready) {
        if (pump_on) pump_stop("mqtt_desconectado");
        if (fan_on) fan_stop("mqtt_desconectado");
    }
}
static void process_command(const char *payload)
{
    cJSON *root = cJSON_Parse(payload);
    if (!root || !cJSON_IsObject(root)) {
        publish_actuator_event("comando_rechazado", "", "json_invalido");
        cJSON_Delete(root);
        return;
    }
    const cJSON *cmd = cJSON_GetObjectItemCaseSensitive(root, "command");
    const cJSON *duration = cJSON_GetObjectItemCaseSensitive(root, "duration_ms");
    const cJSON *request = cJSON_GetObjectItemCaseSensitive(root, "request_id");
    const char *rid = cJSON_IsString(request) && request->valuestring && strlen(request->valuestring) <= 48 ? request->valuestring : "";
    if (!cJSON_IsString(cmd) || !cmd->valuestring) {
        publish_actuator_event("comando_rechazado", rid, "comando_invalido");
        cJSON_Delete(root);
        return;
    }
    if (strcmp(cmd->valuestring, "all_off") == 0) {
        pump_stop("comando");
        fan_stop("comando");
        publish_actuator_event("comando_aceptado", rid, "all_off");
        cJSON_Delete(root);
        return;
    }
    if (strcmp(cmd->valuestring, "set_profile") == 0) {
        const cJSON *id = cJSON_GetObjectItemCaseSensitive(root, "profile_id");
        if (cJSON_IsString(id) && select_profile(id->valuestring)) {
            reset_profile_alerts();
            if (pump_on) pump_stop("cambio_perfil");
            publish_actuator_event("perfil_actualizado", rid, id->valuestring);
        } else publish_actuator_event("comando_rechazado", rid, "perfil_invalido");
        cJSON_Delete(root);
        return;
    }
    if (!cJSON_IsNumber(duration) || !isfinite(duration->valuedouble) ||
        duration->valuedouble < 1 || duration->valuedouble > UV_MAX_PULSE_MS ||
        (double)(int)duration->valuedouble != duration->valuedouble) {
        publish_actuator_event("comando_rechazado", rid, "duracion_invalida");
        cJSON_Delete(root);
        return;
    }
    int ms = duration->valueint;
    if (strcmp(cmd->valuestring, "fan_pulse") == 0) {
        if (fan_on || pump_on) publish_actuator_event("comando_rechazado", rid, "actuador_ocupado");
        else {
            fan_until_ms = now_ms() + ms;
            fan_on = true;
            gpio_set_level((gpio_num_t)UV_RELAY_FAN_GPIO, 0);
            publish_actuator_event("ventilador_encendido", rid, "comando_mqtt");
        }
    } else if (strcmp(cmd->valuestring, "pump_pulse") == 0) {
        const char *reason = NULL;
        if (pump_on || fan_on) reason = "actuador_ocupado";
        else if ((reason = pump_block_reason()) != NULL) { }
        else if (pump_has_stopped && now_ms() - pump_last_stop_ms < UV_PUMP_COOLDOWN_SAFE_MS)
            reason = "cooldown_bomba";
        if (reason) publish_actuator_event("comando_rechazado", rid, reason);
        else {
            pump_until_ms = now_ms() + (ms < UV_PUMP_MAX_MS ? ms : UV_PUMP_MAX_MS);
            pump_on = true;
            gpio_set_level((gpio_num_t)UV_RELAY_PUMP_GPIO, 0);
            publish_actuator_event("bomba_encendida", rid, "comando_mqtt_validado");
        }
    } else {
        publish_actuator_event("comando_rechazado", rid, "comando_desconocido");
    }
    cJSON_Delete(root);
}
#if UV_HAS_ADC
static void init_adc(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc));
#if UV_ENABLE_LM35
    adc_oneshot_chan_cfg_t lm35_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_0
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, (adc_channel_t)UV_ADC_LM35, &lm35_cfg));
    ESP_LOGI(TAG, "ADC LM35 inicializado");
#endif
#if UV_ENABLE_SOIL
    adc_oneshot_chan_cfg_t soil_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, (adc_channel_t)UV_ADC_SOIL, &soil_cfg));
    ESP_LOGI(TAG, "ADC YL-69 inicializado");
#endif
#if UV_ENABLE_WATER
    adc_oneshot_chan_cfg_t water_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, (adc_channel_t)UV_ADC_WATER, &water_cfg));
    ESP_LOGI(TAG, "ADC agua inicializado");
#endif
#if UV_ENABLE_LDR
    adc_oneshot_chan_cfg_t ldr_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, (adc_channel_t)UV_ADC_LDR, &ldr_cfg));
    ESP_LOGI(TAG, "ADC LDR inicializado");
#endif
#if UV_ENABLE_MQ2
    adc_oneshot_chan_cfg_t mq2_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12
    };
    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc,
            (adc_channel_t)UV_ADC_MQ2,
            &mq2_cfg
        )
    );
    ESP_LOGI(TAG, "ADC MQ-2 inicializado");
#endif
}
#endif
#if UV_ENABLE_LM35
static void init_lm35_calibration(void)
{
    adc_cali_line_fitting_config_t cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_0,
        .bitwidth = ADC_BITWIDTH_12
    };
    esp_err_t err = adc_cali_create_scheme_line_fitting(&cfg, &lm35_cali);
    lm35_cali_valida = (err == ESP_OK);
    if (!lm35_cali_valida) ESP_LOGW(TAG, "Calibracion LM35 no disponible: %s", esp_err_to_name(err));
}
static void actualizar_lm35(void)
{
    int raw = 0;
    if (adc_oneshot_read(adc, (adc_channel_t)UV_ADC_LM35, &raw) != ESP_OK) {
        lm35_valido = false;
        ESP_LOGW(TAG, "Error ADC LM35");
        return;
    }
    raw_lm35 = raw;
    if (!lm35_cali_valida || adc_cali_raw_to_voltage(lm35_cali, raw, &lm35_mv) != ESP_OK) {
        lm35_valido = false;
        ESP_LOGW(TAG, "Conversion LM35 no disponible");
        return;
    }
    temperatura_lm35 = lm35_mv / 10.0f;
    lm35_valido = raw > 0 && lm35_mv >= 100 && lm35_mv <= 600 && isfinite(temperatura_lm35);
    if (lm35_valido) ESP_LOGI(TAG, "LM35: %.1f C", temperatura_lm35);

}
#endif
#if UV_ENABLE_DHT11
static void actualizar_dht11(void)
{
    float t = 0.0f, h = 0.0f;
    if (dht11_leer(&t, &h) && isfinite(t) && isfinite(h) && t >= 0 && t <= 50 && h >= 0 && h <= 100) {
        temperatura_ambiente = t;
        humedad_ambiente = h;
        dht11_valido = true;
        ESP_LOGI(TAG, "DHT11: %.1f C | %.1f %%RH", t, h);
    } else {
        dht11_valido = false;
        ESP_LOGW(TAG, "Error leyendo DHT11");
    }
}
#endif
#if UV_ENABLE_SOIL
static void actualizar_suelo(void)
{
    soil_valido = adc_oneshot_read(adc, (adc_channel_t)UV_ADC_SOIL, &raw_soil) == ESP_OK;
    if (soil_valido) { soil_pct = clamp_pct((UV_SOIL_DRY_ADC - raw_soil) * 100.0f / (UV_SOIL_DRY_ADC - UV_SOIL_WET_ADC)); soil_sample_ms = now_ms(); ESP_LOGI(TAG, "YL-69: ADC=%d | %.1f %% relativo", raw_soil, soil_pct); }
    else ESP_LOGW(TAG, "Error leyendo YL-69");
}
#endif
#if UV_ENABLE_WATER
static void actualizar_agua(void)
{
    water_valido = adc_oneshot_read(adc, (adc_channel_t)UV_ADC_WATER, &raw_water) == ESP_OK;

    if (water_valido) {
        water_pct = clamp_pct(raw_water * 100.0f / 1535.0f);
        water_sample_ms = now_ms();

        if (raw_water < UV_WATER_MIN_ADC) {
            water_lock = true;
        } else if (raw_water >= UV_WATER_REARM_ADC) {
            water_lock = false;
        }

        ESP_LOGI(TAG, "Agua: ADC=%d | %.1f %% relativo", raw_water, water_pct);

    } else {
        water_lock = true;
        ESP_LOGW(TAG, "Error leyendo agua");
    }
}
#endif
#if UV_ENABLE_LDR
static void actualizar_ldr(void)
{
    ldr_valido = adc_oneshot_read(adc, (adc_channel_t)UV_ADC_LDR, &raw_ldr) == ESP_OK;
    if (ldr_valido) { light_pct = clamp_pct((UV_LDR_DARK_ADC - raw_ldr) * 100.0f / (UV_LDR_DARK_ADC - UV_LDR_BRIGHT_ADC)); ESP_LOGI(TAG, "LDR: ADC=%d | %.1f %% relativo", raw_ldr, light_pct); }
    else ESP_LOGW(TAG, "Error leyendo LDR");
}
#endif
#if UV_ENABLE_PIR
static void init_pir(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << UV_PIR_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    pir_inicio_ms = now_ms();
    pir_ready = false;
    motion = false;
    pir_estado_anterior = false;
    ESP_LOGI(TAG, "PIR inicializado");
}
static void actualizar_pir(void)
{
    if (!pir_ready) {
        if (now_ms() - pir_inicio_ms < PIR_WARMUP_MS) return;
        motion = gpio_get_level((gpio_num_t)UV_PIR_GPIO) == 1;
        pir_estado_anterior = motion;
        pir_ready = true;
        ESP_LOGI(TAG, "PIR estabilizado, estado=%d", motion);
        return;
    }
    motion = gpio_get_level((gpio_num_t)UV_PIR_GPIO) == 1;
    if (motion == pir_estado_anterior) return;
    pir_estado_anterior = motion;
    ESP_LOGI(TAG, "PIR: movimiento=%s", motion ? "SI" : "NO");
    if (!mqtt_ready || mqtt == NULL) return;
    cJSON *evento = cJSON_CreateObject();
    if (!evento) return;
    cJSON_AddStringToObject(evento, "device_id", "uvgrow_nodo01");
    cJSON_AddStringToObject(evento, "event", motion ? "movimiento_detectado" : "movimiento_finalizado");
    cJSON_AddBoolToObject(evento, "motion", motion);
    cJSON_AddNumberToObject(evento, "uptime_ms", (double)now_ms());
    char *mensaje = cJSON_PrintUnformatted(evento);
    if (mensaje) {
        int id = esp_mqtt_client_publish(mqtt, UV_TOPIC_EVENTS, mensaje, 0, 1, 0);
        if (id < 0) ESP_LOGW(TAG, "Error publicando evento PIR");
        free(mensaje);
    }
    cJSON_Delete(evento);
}
#endif
#if UV_ENABLE_MQ2
static void actualizar_mq2(void)
{
    if (now_ms() - mq2_inicio_ms < 60000)
    {
        mq2_valido = false;
        return;
    }
    int raw = 0;
    esp_err_t err = adc_oneshot_read(
        adc,
        (adc_channel_t)UV_ADC_MQ2,
        &raw
    );
    if (err != ESP_OK)
    {
        mq2_valido = false;
        ESP_LOGW(TAG, "Error leyendo MQ-2");
        return;
    }
    raw_mq2 = raw;
    mq2_valido = true;
    ESP_LOGI(
        TAG,
        "MQ-2: ADC=%d",
        raw_mq2
    );
}
#endif
static void publish_json(void)
{
    if (!mqtt_ready || mqtt == NULL) return;
    cJSON *root = cJSON_CreateObject();
    if (!root) return;
    cJSON_AddStringToObject(root, "device_id", "uvgrow_nodo01");
    cJSON_AddNumberToObject(root, "uptime_ms", (double)now_ms());
#if UV_ENABLE_DHT11
    if (dht11_valido) {
        cJSON_AddNumberToObject(root, "temperatura_ambiente_c", temperatura_ambiente);
        cJSON_AddNumberToObject(root, "humedad_ambiente_pct", humedad_ambiente);
    } else {
        cJSON_AddNullToObject(root, "temperatura_ambiente_c");
        cJSON_AddNullToObject(root, "humedad_ambiente_pct");
    }
#endif
#if UV_ENABLE_LM35
    if (lm35_valido) {
        cJSON_AddNumberToObject(root, "temperatura_suelo_c", temperatura_lm35);
        cJSON_AddNumberToObject(root, "lm35_mv", lm35_mv);
        cJSON_AddNumberToObject(root, "lm35_adc", raw_lm35);
    } else {
        cJSON_AddNullToObject(root, "temperatura_suelo_c");
        cJSON_AddNullToObject(root, "lm35_mv");
        cJSON_AddNullToObject(root, "lm35_adc");
    }
#endif
#if UV_ENABLE_SOIL
    if (soil_valido) { cJSON_AddNumberToObject(root, "soil_adc", raw_soil); cJSON_AddNumberToObject(root, "soil_pct", soil_pct); }
    else { cJSON_AddNullToObject(root, "soil_adc"); cJSON_AddNullToObject(root, "soil_pct"); }
#endif
#if UV_ENABLE_WATER
    if (water_valido) { cJSON_AddNumberToObject(root, "water_adc", raw_water); cJSON_AddNumberToObject(root, "water_pct", water_pct); }
    else { cJSON_AddNullToObject(root, "water_adc"); cJSON_AddNullToObject(root, "water_pct"); }
    cJSON_AddBoolToObject(root, "water_calibrated", true);
    cJSON_AddBoolToObject(root, "water_pump_lock", water_lock);
#endif
#if UV_ENABLE_LDR
    if (ldr_valido) { cJSON_AddNumberToObject(root, "ldr_adc", raw_ldr); cJSON_AddNumberToObject(root, "light_pct", light_pct); }
    else { cJSON_AddNullToObject(root, "ldr_adc"); cJSON_AddNullToObject(root, "light_pct"); }
#endif
#if UV_ENABLE_PIR
    if (pir_ready) cJSON_AddBoolToObject(root, "motion", motion);
    else cJSON_AddNullToObject(root, "motion");
#endif
#if UV_ENABLE_REED
    if (reed_inicializado) cJSON_AddBoolToObject(root, "tapa_deposito_cerrada", tapa_cerrada);
    else cJSON_AddNullToObject(root, "tapa_deposito_cerrada");
#endif
    cJSON_AddBoolToObject(root, "pump_on", pump_on);
    cJSON_AddBoolToObject(root, "fan_on", fan_on);
    cJSON_AddStringToObject(root, "status", "perfil_y_alertas");
    cJSON_AddStringToObject(root, "profile_id", profile_index >= 0 ? profiles[profile_index].id : "sin_perfil");
    cJSON *active = cJSON_AddArrayToObject(root, "active_alerts");
    if (active) for (int i = 0; i < ALERT_COUNT; ++i)
        if (alerts[i].active) cJSON_AddItemToArray(active, cJSON_CreateString(alerts[i].name));
	#if UV_ENABLE_MQ2
	    if (mq2_valido)
	    {
	        cJSON_AddNumberToObject(
	            root,
	            "mq2_adc",
	            raw_mq2
	        );
	    }
	    else
	    {
	        cJSON_AddNullToObject(
	            root,
	            "mq2_adc"
	        );
	    }
	    cJSON_AddBoolToObject(
	        root,
	        "mq2_calibrated",
	        false
	    );
	#endif
	    char *json = cJSON_PrintUnformatted(root);
    if (json) {
        int id = esp_mqtt_client_publish(mqtt, UV_TOPIC_TELEMETRY, json, 0, 1, 0);
        if (id >= 0) ESP_LOGI(TAG, "MQTT publicado: %s", json);
        else ESP_LOGW(TAG, "Error publicando MQTT");
        free(json);
    }
    cJSON_Delete(root);
}
static void on_mqtt(void *args, esp_event_base_t base, int32_t id, void *data)
{
    (void)args;
    (void)base;
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)data;
    if (id == MQTT_EVENT_CONNECTED) {
        mqtt_ready = true;
        esp_mqtt_client_subscribe(mqtt, UV_TOPIC_COMMANDS, 1);
        ESP_LOGI(TAG, "MQTT conectado");
    } else if (id == MQTT_EVENT_DISCONNECTED) {
        mqtt_ready = false;
        if (command_queue) xQueueReset(command_queue);
        ESP_LOGW(TAG, "MQTT desconectado");
    } else if (id == MQTT_EVENT_ERROR) {
        ESP_LOGW(TAG, "Error MQTT");
    } else if (id == MQTT_EVENT_DATA && event != NULL) {
        if (event->retain) { ESP_LOGW(TAG, "Comando retenido rechazado"); return; }
        if (event->topic_len != strlen(UV_TOPIC_COMMANDS) ||
            memcmp(event->topic, UV_TOPIC_COMMANDS, event->topic_len) != 0) return;
        if (event->current_data_offset != 0 || event->total_data_len != event->data_len ||
            event->data_len <= 0 || event->data_len >= UV_DEMO_CMD_MAX) {
            ESP_LOGW(TAG, "Comando MQTT fragmentado o demasiado grande; rechazado");
            return;
        }
        uv_command_t cmd = {0};
        memcpy(cmd.payload, event->data, event->data_len);
        if (!command_queue || xQueueSend(command_queue, &cmd, 0) != pdTRUE)
            ESP_LOGW(TAG, "Cola de comandos llena o no disponible");
    }
}
static void start_mqtt(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = UV_MQTT_URI,
        .credentials.client_id = "uvgrow_nodo01"
    };
    mqtt = esp_mqtt_client_init(&cfg);
    if (!mqtt) {
        ESP_LOGE(TAG, "Error inicializando MQTT");
        return;
    }
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt, ESP_EVENT_ANY_ID, on_mqtt, NULL));
}
static void on_wifi(void *args, esp_event_base_t base, int32_t id, void *data)
{
    (void)args;
    (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        mqtt_ready = false;
        ESP_LOGW(TAG, "Wi-Fi desconectado");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Wi-Fi conectado, IP asignada");
        if (mqtt != NULL && !mqtt_started) {
            esp_err_t err = esp_mqtt_client_start(mqtt);
            if (err == ESP_OK) mqtt_started = true;
            else ESP_LOGE(TAG, "Error iniciando MQTT: %s", esp_err_to_name(err));
        }
    }
}
static void start_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi, NULL));
    wifi_config_t cfg = {0};
    snprintf((char *)cfg.sta.ssid, sizeof(cfg.sta.ssid), "%s", UV_WIFI_SSID);
    snprintf((char *)cfg.sta.password, sizeof(cfg.sta.password), "%s", UV_WIFI_PASSWORD);
    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
}
void app_main(void)
{
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES ||
        r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);
    init_outputs();
    restore_profile();
    ESP_LOGI(TAG, "Perfil al iniciar: %s", profile_index < 0 ? "sin_perfil" : profiles[profile_index].id);
    command_queue = xQueueCreate(UV_DEMO_QUEUE_LENGTH, sizeof(uv_command_t));
    ESP_ERROR_CHECK(command_queue ? ESP_OK : ESP_ERR_NO_MEM);
#if UV_ENABLE_REED
    init_reed();
#endif
#if UV_ENABLE_PIR
    init_pir();
#endif
#if UV_HAS_ADC
    init_adc();
#endif
#if UV_ENABLE_LM35
    init_lm35_calibration();
#endif
#if UV_ENABLE_MQ2
    mq2_inicio_ms = now_ms();
#endif
    start_mqtt();
    start_wifi();
#if UV_ENABLE_RFID
    init_rfid();
#endif
    int64_t last_publish = -UV_TELEMETRY_INTERVAL_MS;
#if UV_ENABLE_REED
    int64_t last_reed = -UV_REED_INTERVAL_MS;
#endif
#if UV_ENABLE_LM35
    int64_t last_lm35 = -LM35_INTERVAL_MS;
#endif
#if UV_ENABLE_SOIL
    int64_t last_soil = -SOIL_INTERVAL_MS;
#endif
#if UV_ENABLE_WATER
    int64_t last_water = -WATER_INTERVAL_MS;
#endif
#if UV_ENABLE_LDR
    int64_t last_ldr = -LDR_INTERVAL_MS;
#endif
#if UV_ENABLE_MQ2
    int64_t last_mq2 = -MQ2_INTERVAL_MS;
#endif
#if UV_ENABLE_DHT11
    int64_t last_dht = now_ms();
#endif
#if UV_ENABLE_PIR
    int64_t last_pir = -PIR_INTERVAL_MS;
#endif
    while (true) {
        service_actuators();
        uv_command_t queued;
        if (command_queue && xQueueReceive(command_queue, &queued, 0) == pdTRUE) {
            if (mqtt_ready) process_command(queued.payload);
        }
        int64_t t = now_ms();
#if UV_ENABLE_REED
        if (t - last_reed >= UV_REED_INTERVAL_MS) {
            last_reed = t;
            actualizar_reed();
        }
#endif
#if UV_ENABLE_LM35
        if (t - last_lm35 >= LM35_INTERVAL_MS) {
            last_lm35 = t;
            actualizar_lm35();
        }
#endif
#if UV_ENABLE_SOIL
        if (t - last_soil >= SOIL_INTERVAL_MS) {
            last_soil = t;
            actualizar_suelo();
            if (soil_valido && profile_index >= 0) {
                const uv_profile_t *p = &profiles[profile_index];
                update_alert(ALERT_SOIL_DRY, soil_pct < p->soil_min, soil_pct > p->soil_min + 5, soil_pct);
                update_alert(ALERT_SOIL_WET, soil_pct > p->soil_max, soil_pct < p->soil_max - 5, soil_pct);
            }
        }
#endif
#if UV_ENABLE_WATER
        if (t - last_water >= WATER_INTERVAL_MS) {
            last_water = t;
            actualizar_agua();
            if (water_valido) update_alert(ALERT_WATER, raw_water < 400, raw_water > 500, raw_water);
        }
#endif
#if UV_ENABLE_LDR
        if (t - last_ldr >= LDR_INTERVAL_MS) {
            last_ldr = t;
            actualizar_ldr();
            if (ldr_valido && profile_index >= 0) {
                float min = profiles[profile_index].light_min;
                update_alert(ALERT_LIGHT_LOW, light_pct < min, light_pct > min + 5, light_pct);
            }
        }
#endif
#if UV_ENABLE_MQ2
        if (t - last_mq2 >= MQ2_INTERVAL_MS) {
            last_mq2 = t;
            actualizar_mq2();
            if (mq2_valido) update_alert(ALERT_MQ2, raw_mq2 > UV_MQ2_HIGH_ADC, raw_mq2 < UV_MQ2_CLEAR_ADC, raw_mq2);
        }
#endif
#if UV_ENABLE_DHT11
        if (t - last_dht >= DHT_INTERVAL_MS) {
            last_dht = t;
            actualizar_dht11();
            if (dht11_valido && profile_index >= 0) {
                const uv_profile_t *p = &profiles[profile_index];
                update_alert(ALERT_TEMP_HIGH, temperatura_ambiente > p->temp_max, temperatura_ambiente < p->temp_max - 1, temperatura_ambiente);
                update_alert(ALERT_TEMP_LOW, temperatura_ambiente < p->temp_min, temperatura_ambiente > p->temp_min + 1, temperatura_ambiente);
                update_alert(ALERT_RH_HIGH, humedad_ambiente > p->rh_max, humedad_ambiente < p->rh_max - 5, humedad_ambiente);
            }
        }
#endif
#if UV_ENABLE_PIR
        if (t - last_pir >= PIR_INTERVAL_MS) {
            last_pir = t;
            actualizar_pir();
        }
#endif
        if (t - last_publish >= UV_TELEMETRY_INTERVAL_MS) {
            last_publish = t;
            publish_json();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
