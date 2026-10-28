//Main para configurar la esp32 - CAM
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_http_server.h"

#include "esp_camera.h"
#include "camera_pins.h"
#include "config_cam.h"

static const char *TAG = "UVGrow_CAM";

static httpd_handle_t servidor = NULL;

static void iniciar_camara(void)
{
    camera_config_t config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,

        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,

        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,

        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_QVGA,
        .jpeg_quality = 12,
        .fb_count = 1,
        .fb_location = CAMERA_FB_IN_DRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
    };

    ESP_ERROR_CHECK(esp_camera_init(&config));

    ESP_LOGI(TAG, "OV2640 inicializada correctamente");
}

static esp_err_t pagina_handler(httpd_req_t *req)
{
    const char *html =
        "<!DOCTYPE html>"
        "<html lang='es'>"
        "<head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>UVGrow CAM</title>"
        "</head>"
        "<body style='font-family:Arial;text-align:center;padding:20px'>"
        "<h2>UVGrow - Camara IoT</h2>"
        "<p>ESP32-CAM OV2640</p>"
        "<img src='/capture' style='max-width:100%;height:auto'>"
        "<p><a href='/capture'>Abrir fotografia</a></p>"
        "<button onclick='location.reload()'>Actualizar imagen</button>"
        "</body>"
        "</html>";

    httpd_resp_set_type(req, "text/html; charset=utf-8");

    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t captura_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();

    if (fb == NULL) {
        ESP_LOGE(TAG, "Error capturando fotografia");
        httpd_resp_send_err(
            req,
            HTTPD_500_INTERNAL_SERVER_ERROR,
            "Error capturando imagen"
        );
        return ESP_FAIL;
    }

    esp_err_t err = httpd_resp_set_type(req, "image/jpeg");

    if (err == ESP_OK) {
        err = httpd_resp_set_hdr(
            req,
            "Cache-Control",
            "no-store"
        );
    }

    if (err == ESP_OK) {
        err = httpd_resp_send(
            req,
            (const char *)fb->buf,
            fb->len
        );
    }

    ESP_LOGI(
        TAG,
        "Fotografia HTTP: %zu bytes",
        fb->len
    );

    esp_camera_fb_return(fb);

    return err;
}

static void iniciar_servidor(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.server_port = 80;
    config.stack_size = 8192;

    ESP_ERROR_CHECK(
        httpd_start(&servidor, &config)
    );

    httpd_uri_t pagina = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = pagina_handler,
        .user_ctx = NULL
    };

    httpd_uri_t captura = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = captura_handler,
        .user_ctx = NULL
    };

    ESP_ERROR_CHECK(
        httpd_register_uri_handler(servidor, &pagina)
    );

    ESP_ERROR_CHECK(
        httpd_register_uri_handler(servidor, &captura)
    );

    ESP_LOGI(TAG, "Servidor HTTP iniciado");
}

static void wifi_event_handler(
    void *arg,
    esp_event_base_t base,
    int32_t id,
    void *data
)
{
    (void)arg;

    if (base == WIFI_EVENT &&
        id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }

    else if (base == WIFI_EVENT &&
             id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi desconectado");
        esp_wifi_connect();
    }

    else if (base == IP_EVENT &&
             id == IP_EVENT_STA_GOT_IP) {

        ip_event_got_ip_t *event =
            (ip_event_got_ip_t *)data;

        ESP_LOGI(
            TAG,
            "Wi-Fi conectado. IP: " IPSTR,
            IP2STR(&event->ip_info.ip)
        );
    }
}

static void iniciar_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(
        esp_event_loop_create_default()
    );

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init =
        WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&init));

    ESP_ERROR_CHECK(
        esp_event_handler_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            wifi_event_handler,
            NULL
        )
    );

    ESP_ERROR_CHECK(
        esp_event_handler_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            wifi_event_handler,
            NULL
        )
    );

    wifi_config_t wifi_config = {0};

    snprintf(
        (char *)wifi_config.sta.ssid,
        sizeof(wifi_config.sta.ssid),
        "%s",
        CAM_WIFI_SSID
    );

    snprintf(
        (char *)wifi_config.sta.password,
        sizeof(wifi_config.sta.password),
        "%s",
        CAM_WIFI_PASSWORD
    );

    wifi_config.sta.threshold.authmode =
        WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(
        esp_wifi_set_mode(WIFI_MODE_STA)
    );

    ESP_ERROR_CHECK(
        esp_wifi_set_config(
            WIFI_IF_STA,
            &wifi_config
        )
    );

    ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK(err);

    iniciar_camara();

    iniciar_wifi();

    iniciar_servidor();

    ESP_LOGI(TAG, "UVGrow CAM lista");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
