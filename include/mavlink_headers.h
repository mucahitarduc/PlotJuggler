#ifndef MAVLINK_HEADERS_H_
#define MAVLINK_HEADERS_H_

#define MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_EXTERNAL_RX_STATUS

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include <stddef.h>
#include <mavlink_types.h>
extern mavlink_system_t mavlink_system;
extern mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];

#define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes
extern void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t * ch, uint16_t length);

#include <common/mavlink.h>
#include <common/common.h>
#include <mavlink_get_info.h>

#include <mavlink_helpers.h>

#endif // MAVLINK_HEADERS_H_
