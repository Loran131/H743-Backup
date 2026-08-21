#include "tcp_control_server.h"

#include "cmsis_os2.h"
#include "lwip/errno.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "network_config.h"
#include "remote_control.h"
#include "stm32h7xx_hal.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define TCP_RX_BUFFER_SIZE       256U
#define TCP_RECEIVE_TIMEOUT_MS   100
#define TCP_SEND_TIMEOUT_MS      1000
#define TCP_LISTEN_RETRY_MS      1000U
#define TCP_STATUS_PERIOD_MS     1000U
#define TCP_SEND_CHUNK_SIZE      536U
#define TCP_SEND_TOTAL_TIMEOUT_MS 3000U

static uint8_t g_rx_buffer[TCP_RX_BUFFER_SIZE];
static char g_rx_line[REMOTE_JSON_LINE_SIZE];
static char g_tx_response[REMOTE_JSON_LINE_SIZE];

static int send_all(int socket_fd, const uint8_t *data, size_t length)
{
    size_t sent_total = 0U;
    uint32_t deadline = HAL_GetTick() + TCP_SEND_TOTAL_TIMEOUT_MS;
    while (sent_total < length) {
        size_t remaining = length - sent_total;
        size_t chunk = (remaining > TCP_SEND_CHUNK_SIZE) ?
                       TCP_SEND_CHUNK_SIZE : remaining;
        int sent = lwip_send(socket_fd, &data[sent_total], chunk, 0);
        if (sent == 0) return -1;
        if (sent < 0) {
            if (((errno != EWOULDBLOCK) && (errno != EAGAIN)) ||
                ((int32_t)(HAL_GetTick() - deadline) >= 0)) return -1;
            osDelay(1U);
            continue;
        }
        sent_total += (size_t)sent;
    }
    return 0;
}

static int send_line(int socket_fd, const char *text)
{
    static const uint8_t newline = '\n';
    if (send_all(socket_fd, (const uint8_t *)text, strlen(text)) != 0)
        return -1;
    return send_all(socket_fd, &newline, 1U);
}

static int send_protocol_error(int socket_fd, const char *reason,
                               char *response, size_t response_size)
{
    RemoteControl_FormatError(response, response_size, reason,
                              HAL_GetTick());
    return send_line(socket_fd, response);
}

static int process_line(int socket_fd, const char *json,
                        char *response, size_t response_size)
{
    RemoteSubmitResult result = RemoteControl_SubmitJson(
        json, response, response_size, HAL_GetTick());
    if (result == REMOTE_SUBMIT_NO_RESPONSE) return 0;
    if (result != REMOTE_SUBMIT_RESPONSE)
        RemoteControl_FormatError(response, response_size,
                                  "internal_protocol_error", HAL_GetTick());
    return send_line(socket_fd, response);
}

static int send_pending_events(int socket_fd, char *response,
                               size_t response_size)
{
    while (RemoteControl_PeekEvent(response, response_size,
                                   HAL_GetTick()) != 0U) {
        if (send_line(socket_fd, response) != 0) return -1;
        RemoteControl_ConfirmEvent();
    }
    return 0;
}

static int recv_timed_out(void)
{
    return ((errno == EWOULDBLOCK) || (errno == EAGAIN)) ? 1 : 0;
}

static void serve_client(int client_fd)
{
    size_t line_length = 0U;
    uint8_t line_overflow = 0U;
    uint32_t next_heartbeat;
    uint32_t next_status;
    struct timeval receive_timeout = {
        .tv_sec = 0,
        .tv_usec = TCP_RECEIVE_TIMEOUT_MS * 1000
    };
    struct timeval send_timeout = {
        .tv_sec = TCP_SEND_TIMEOUT_MS / 1000,
        .tv_usec = (TCP_SEND_TIMEOUT_MS % 1000) * 1000
    };
    int keepalive = 1;
    int no_delay = 1;

    if ((lwip_setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO,
                         &receive_timeout, sizeof(receive_timeout)) != 0) ||
        (lwip_setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO,
                         &send_timeout, sizeof(send_timeout)) != 0)) return;
    (void)lwip_setsockopt(client_fd, SOL_SOCKET, SO_KEEPALIVE,
                          &keepalive, sizeof(keepalive));
    (void)lwip_setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY,
                          &no_delay, sizeof(no_delay));

    RemoteControl_FormatHello(g_tx_response, sizeof(g_tx_response),
                              HAL_GetTick());
    if (send_line(client_fd, g_tx_response) != 0) return;
    RemoteControl_FormatStatus(g_tx_response, sizeof(g_tx_response), 0xFFU,
                               HAL_GetTick());
    if (send_line(client_fd, g_tx_response) != 0) return;
    next_heartbeat = HAL_GetTick() + NETWORK_HEARTBEAT_PERIOD_MS;
    next_status = HAL_GetTick() + TCP_STATUS_PERIOD_MS;

    for (;;) {
        uint32_t now = HAL_GetTick();
        int received;
        uint8_t config_task;
        if (RemoteControl_TakeStatusRequest(&config_task) != 0U) {
            RemoteControl_FormatStatus(g_tx_response, sizeof(g_tx_response),
                                       config_task, now);
            if (send_line(client_fd, g_tx_response) != 0) return;
        }
        if (send_pending_events(client_fd, g_tx_response,
                                sizeof(g_tx_response)) != 0)
            return;
        if ((int32_t)(now - next_heartbeat) >= 0) {
            RemoteControl_FormatHeartbeat(g_tx_response,
                                          sizeof(g_tx_response), now);
            if (send_line(client_fd, g_tx_response) != 0) return;
            next_heartbeat = now + NETWORK_HEARTBEAT_PERIOD_MS;
        }
        if ((int32_t)(now - next_status) >= 0) {
            RemoteControl_FormatStatus(g_tx_response, sizeof(g_tx_response),
                                       0xFFU, now);
            if (send_line(client_fd, g_tx_response) != 0) return;
            next_status = now + TCP_STATUS_PERIOD_MS;
        }

        received = lwip_recv(client_fd, g_rx_buffer, sizeof(g_rx_buffer), 0);
        if (received == 0) return;
        if (received < 0) {
            if (recv_timed_out() != 0) continue;
            return;
        }
        for (int i = 0; i < received; ++i) {
            uint8_t byte = g_rx_buffer[i];
            if (byte == '\r') continue;
            if (byte == '\n') {
                if (line_overflow != 0U) {
                    if (send_protocol_error(client_fd, "line_too_long",
                                            g_tx_response,
                                            sizeof(g_tx_response)) != 0)
                        return;
                } else if (line_length == 0U) {
                    if (send_protocol_error(client_fd, "empty_message",
                                            g_tx_response,
                                            sizeof(g_tx_response)) != 0)
                        return;
                } else {
                    g_rx_line[line_length] = '\0';
                    if (process_line(client_fd, g_rx_line, g_tx_response,
                                     sizeof(g_tx_response)) != 0) return;
                }
                line_length = 0U;
                line_overflow = 0U;
                continue;
            }
            if (line_overflow != 0U) continue;
            if ((byte < 0x20U) && (byte != '\t')) {
                line_overflow = 1U;
            } else if (line_length < (sizeof(g_rx_line) - 1U)) {
                g_rx_line[line_length++] = (char)byte;
            } else {
                line_overflow = 1U;
                line_length = 0U;
            }
        }
    }
}

void TcpControlServer_Run(void)
{
    struct sockaddr_in address;
    int reuse_addr = 1;
    for (;;) {
        int listen_fd = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (listen_fd < 0) {
            osDelay(TCP_LISTEN_RETRY_MS);
            continue;
        }
        (void)lwip_setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR,
                              &reuse_addr, sizeof(reuse_addr));
        memset(&address, 0, sizeof(address));
        address.sin_family = AF_INET;
        address.sin_port = PP_HTONS(NETWORK_TCP_PORT);
        address.sin_addr.s_addr = PP_HTONL(INADDR_ANY);
        if ((lwip_bind(listen_fd, (const struct sockaddr *)&address,
                       sizeof(address)) != 0) ||
            (lwip_listen(listen_fd, 1) != 0)) {
            (void)lwip_close(listen_fd);
            osDelay(TCP_LISTEN_RETRY_MS);
            continue;
        }
        for (;;) {
            int client_fd = lwip_accept(listen_fd, NULL, NULL);
            if (client_fd < 0) break;
            serve_client(client_fd);
            (void)lwip_shutdown(client_fd, SHUT_RDWR);
            (void)lwip_close(client_fd);
        }
        (void)lwip_close(listen_fd);
        osDelay(100U);
    }
}
