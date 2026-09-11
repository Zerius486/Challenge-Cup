#include "udp_transport.h"

#include <string.h>

#include "app_bridge.h"
#include "app_config.h"
#include "board.h"
#include "legacy_can_bridge.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "stm32f4xx_hal.h"
#include "udp_protocol.h"

static struct udp_pcb *server;
static struct udp_pcb *legacy_can_rx;
static struct udp_pcb *legacy_can_tx;
static ip_addr_t peer_address;
static uint16_t peer_port;
static bool have_peer;
static uint32_t telemetry_sequence;
static uint32_t last_telemetry_ms;
static ip_addr_t legacy_peer_address;
static bool have_legacy_peer;

#define CAN_MIRROR_QUEUE_SIZE 16U
static CanFrame can_mirror_queue[CAN_MIRROR_QUEUE_SIZE];
static volatile uint8_t can_mirror_head;
static volatile uint8_t can_mirror_tail;

static bool send_buffer(struct udp_pcb *pcb, const uint8_t *data,
                        uint16_t length, const ip_addr_t *address,
                        uint16_t port) {
  err_t result = ERR_ARG;
  if (pcb == NULL || data == NULL || length == 0U || address == NULL) {
    return false;
  }
  struct pbuf *packet = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_RAM);
  if (packet == NULL) {
    return false;
  }
  if (pbuf_take(packet, data, length) == ERR_OK) {
    result = udp_sendto(pcb, packet, address, port);
  }
  pbuf_free(packet);
  return result == ERR_OK;
}

static void receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                             const ip_addr_t *address, uint16_t port) {
  uint8_t datagram[UDP_PROTOCOL_HEADER_SIZE + UDP_PROTOCOL_MAX_PAYLOAD +
                   UDP_PROTOCOL_CRC_SIZE];
  uint8_t response[64];
  size_t response_length;
  UdpPacketView packet;
  UdpDecodeStatus decode_status;
  (void)arg;
  (void)pcb;

  if (p == NULL) {
    return;
  }
  if (p->tot_len <= sizeof(datagram) &&
      pbuf_copy_partial(p, datagram, p->tot_len, 0U) == p->tot_len) {
    decode_status = udp_protocol_decode(datagram, p->tot_len, &packet);
    response_length = app_bridge_handle_datagram(
        datagram, p->tot_len, HAL_GetTick(), response, sizeof(response));
    if (response_length > 0U) {
      (void)send_buffer(server, response, (uint16_t)response_length, address,
                        port);
    }
    if (decode_status == UDP_DECODE_OK ||
        (response_length >= 4U && memcmp(response, "ARM:", 4U) == 0)) {
      ip_addr_copy(peer_address, *address);
      peer_port = port;
      have_peer = true;
    }
  }
  pbuf_free(p);
}

static void legacy_can_receive_callback(void *arg, struct udp_pcb *pcb,
                                        struct pbuf *p,
                                        const ip_addr_t *address,
                                        uint16_t port) {
  uint8_t datagram[LEGACY_CAN_BRIDGE_FRAME_SIZE];
  CanFrame frame;
  bool allowed;
  (void)arg;
  (void)pcb;
  (void)port;

  if (p == NULL) {
    return;
  }
  if (p->tot_len == sizeof(datagram) &&
      pbuf_copy_partial(p, datagram, sizeof(datagram), 0U) ==
          sizeof(datagram)) {
    if (legacy_can_bridge_decode(datagram, sizeof(datagram), &frame)) {
#if APP_LEGACY_RAW_CAN_ALLOWED
      allowed = true;
#else
      allowed = legacy_can_bridge_is_safe_stop(&frame);
#endif
      if (allowed && board_can_send(APP_LEGACY_RAW_CAN_BUS, &frame, 3U)) {
        ip_addr_copy(legacy_peer_address, *address);
        have_legacy_peer = true;
      }
    }
  }
  pbuf_free(p);
}

/** @brief 创建主 UDP、兼容 CAN 桥接端口并注册收包回调。 */
bool udp_transport_init(void) {
  server = udp_new_ip_type(IPADDR_TYPE_V4);
  if (server == NULL) {
    return false;
  }
  if (udp_bind(server, IP_ANY_TYPE, APP_UDP_PORT) != ERR_OK) {
    udp_remove(server);
    server = NULL;
    return false;
  }
  udp_recv(server, receive_callback, NULL);

  legacy_can_rx = udp_new_ip_type(IPADDR_TYPE_V4);
  legacy_can_tx = udp_new_ip_type(IPADDR_TYPE_V4);
  if (legacy_can_rx == NULL || legacy_can_tx == NULL ||
      udp_bind(legacy_can_rx, IP_ANY_TYPE, APP_LEGACY_CAN_RX_PORT) != ERR_OK ||
      udp_bind(legacy_can_tx, IP_ANY_TYPE, APP_LEGACY_CAN_TX_PORT) != ERR_OK) {
    if (legacy_can_rx != NULL) {
      udp_remove(legacy_can_rx);
    }
    if (legacy_can_tx != NULL) {
      udp_remove(legacy_can_tx);
    }
    udp_remove(server);
    server = NULL;
    return false;
  }
  udp_recv(legacy_can_rx, legacy_can_receive_callback, NULL);
  have_peer = false;
  have_legacy_peer = false;
  can_mirror_head = 0U;
  can_mirror_tail = 0U;
  telemetry_sequence = 0U;
  last_telemetry_ms = 0U;
  return true;
}

/** @brief 发送 CAN 镜像和周期遥测/机械臂状态。 */
void udp_transport_poll(uint32_t now_ms) {
  uint8_t payload[68];
  uint8_t arm_payload[64];
  uint8_t datagram[UDP_PROTOCOL_HEADER_SIZE + sizeof(payload) +
                   UDP_PROTOCOL_CRC_SIZE];
  uint8_t arm_datagram[UDP_PROTOCOL_HEADER_SIZE + sizeof(arm_payload) +
                       UDP_PROTOCOL_CRC_SIZE];
  size_t payload_length;
  size_t datagram_length;

  for (uint32_t sent = 0U;
       sent < APP_CAN_MIRROR_TX_BUDGET &&
       can_mirror_tail != can_mirror_head;
       ++sent) {
    uint8_t encoded[LEGACY_CAN_BRIDGE_FRAME_SIZE];
    CanFrame frame = can_mirror_queue[can_mirror_tail];
    can_mirror_tail =
        (uint8_t)((can_mirror_tail + 1U) % CAN_MIRROR_QUEUE_SIZE);
    if (have_legacy_peer &&
        legacy_can_bridge_encode(&frame, encoded) == sizeof(encoded)) {
      (void)send_buffer(legacy_can_tx, encoded, sizeof(encoded),
                        &legacy_peer_address, APP_LEGACY_CAN_TX_PORT);
    }
  }

  if (!have_peer || (uint32_t)(now_ms - last_telemetry_ms) <
                        APP_TELEMETRY_PERIOD_MS) {
    return;
  }
  last_telemetry_ms = now_ms;
  payload_length = app_bridge_build_telemetry(now_ms, payload,
                                               sizeof(payload));
  datagram_length = udp_protocol_encode(
      UDP_MSG_TELEMETRY, 0U, telemetry_sequence++, payload,
      (uint16_t)payload_length, datagram, sizeof(datagram));
  if (datagram_length > 0U) {
    (void)send_buffer(server, datagram, (uint16_t)datagram_length,
                      &peer_address, peer_port);
  }

  payload_length = app_bridge_build_arm_status(now_ms, arm_payload,
                                               sizeof(arm_payload));
  datagram_length = udp_protocol_encode(
      UDP_MSG_ARM_STATUS, 0U, telemetry_sequence++, arm_payload,
      (uint16_t)payload_length, arm_datagram, sizeof(arm_datagram));
  if (datagram_length > 0U) {
    (void)send_buffer(server, arm_datagram, (uint16_t)datagram_length,
                      &peer_address, peer_port);
  }
}

/** @brief 从 CAN 接收中断排队待发送的兼容镜像帧。 */
void udp_transport_queue_can_from_isr(uint8_t bus, const CanFrame *frame) {
  uint8_t next;
  if (bus != APP_LEGACY_RAW_CAN_BUS || frame == NULL) {
    return;
  }
  next = (uint8_t)((can_mirror_head + 1U) % CAN_MIRROR_QUEUE_SIZE);
  if (next == can_mirror_tail) {
    return;
  }
  can_mirror_queue[can_mirror_head] = *frame;
  __DMB();
  can_mirror_head = next;
}
