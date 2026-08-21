#ifndef NETWORK_CONFIG_H
#define NETWORK_CONFIG_H

#define NETWORK_PROFILE_DIRECT_PC  1U
#define NETWORK_PROFILE_ROUTER     2U

#ifndef NETWORK_ACTIVE_PROFILE
#define NETWORK_ACTIVE_PROFILE NETWORK_PROFILE_ROUTER
#endif

#define NETWORK_TCP_PORT             5000U
#define NETWORK_PROTOCOL_VERSION     1U
#define NETWORK_HEARTBEAT_PERIOD_MS  2000U
#define NETWORK_DEVICE_ID            "gantry"
#define NETWORK_CENTER_ID            "center"
#define NETWORK_DEVICE_TYPE          "gantry"

#if NETWORK_ACTIVE_PROFILE == NETWORK_PROFILE_DIRECT_PC
#define NETWORK_IP_0       192U
#define NETWORK_IP_1       168U
#define NETWORK_IP_2       137U
#define NETWORK_IP_3       10U
#define NETWORK_GATEWAY_0  0U
#define NETWORK_GATEWAY_1  0U
#define NETWORK_GATEWAY_2  0U
#define NETWORK_GATEWAY_3  0U
#elif NETWORK_ACTIVE_PROFILE == NETWORK_PROFILE_ROUTER
#define NETWORK_IP_0       192U
#define NETWORK_IP_1       168U
#define NETWORK_IP_2       10U
#define NETWORK_IP_3       111U
/* Override these four macros if the deployment requires a gateway. */
#ifndef NETWORK_GATEWAY_0
#define NETWORK_GATEWAY_0  0U
#define NETWORK_GATEWAY_1  0U
#define NETWORK_GATEWAY_2  0U
#define NETWORK_GATEWAY_3  0U
#endif
#else
#error "Unsupported NETWORK_ACTIVE_PROFILE"
#endif

#define NETWORK_NETMASK_0  255U
#define NETWORK_NETMASK_1  255U
#define NETWORK_NETMASK_2  255U
#define NETWORK_NETMASK_3  0U

#endif
