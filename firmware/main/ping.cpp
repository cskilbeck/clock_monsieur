#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_ping.h"
#include "ping/ping_sock.h"
#include "wifi.h"
#include "ping.h"
#include "util.h"
#include "main.h"

LOG_CONTEXT("ping");

static bool done;
static int pings;
static int fails;

void ping_end(esp_ping_handle_t hdl, void *args)
{
    done += 1;
    LOG_INFO("END %d");
}

void ping_success(esp_ping_handle_t hdl, void *args)
{
    pings += 1;
    LOG_INFO("SUCCESS %d", pings);
}

void ping_timeout(esp_ping_handle_t hdl, void *args)
{
    fails += 1;
    LOG_INFO("TIMEOUT %d", fails);
}

esp_err_t do_ping_check()
{
    LOG_INFO("starting");

    xEventGroupWaitBits(system_events, SYS_EVENT_WIFI_CONNECTED, false, true, portMAX_DELAY);

    esp_ping_config_t config{};
    ipaddr_aton("8.8.8.8", &config.target_addr);
    config.count = 3;    // MUST BE ODD to avoid pass/fail tie
    config.interval_ms = 100;
    config.timeout_ms = 1000;
    config.data_size = 64;
    config.tos = 0;
    config.ttl = IP_DEFAULT_TTL;
    config.task_stack_size = ESP_TASK_PING_STACK;
    config.task_prio = 2;
    config.interface = 0;

    esp_ping_callbacks_t callbacks{};
    callbacks.on_ping_end = ping_end;
    callbacks.on_ping_success = ping_success;
    callbacks.on_ping_timeout = ping_timeout;

    esp_ping_handle_t ping_handle{};
    ESP_CHECK(esp_ping_new_session(&config, &callbacks, &ping_handle));

    DEFER(esp_ping_delete_session(ping_handle));

    done = false;
    pings = 0;
    fails = 0;

    ESP_CHECK(esp_ping_start(ping_handle));

    while(!done) {
        LOG_INFO("Waiting for pings");
        delay_secs(1);
    }
    if(fails > pings) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}