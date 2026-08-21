#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#define REMOTE_JSON_LINE_SIZE    2048U
#define REMOTE_COMMAND_ID_SIZE   48U
#define REMOTE_COMMAND_HISTORY   32U

typedef enum {
    REMOTE_SUBMIT_RESPONSE = 0,
    REMOTE_SUBMIT_NO_RESPONSE,
    REMOTE_SUBMIT_INVALID
} RemoteSubmitResult;

void RemoteControl_Init(void);
void RemoteControl_Poll(uint32_t now);
RemoteSubmitResult RemoteControl_SubmitJson(const char *json,
                                            char *response,
                                            size_t response_size,
                                            uint32_t now);
uint8_t RemoteControl_PeekEvent(char *response, size_t response_size,
                                uint32_t now);
void RemoteControl_ConfirmEvent(void);
void RemoteControl_FormatHello(char *response, size_t response_size,
                               uint32_t now);
void RemoteControl_FormatHeartbeat(char *response, size_t response_size,
                                   uint32_t now);
void RemoteControl_FormatStatus(char *response, size_t response_size,
                                uint8_t config_task, uint32_t now);
uint8_t RemoteControl_TakeStatusRequest(uint8_t *config_task);
void RemoteControl_FormatError(char *response, size_t response_size,
                               const char *reason, uint32_t now);

#ifdef __cplusplus
}
#endif

#endif
