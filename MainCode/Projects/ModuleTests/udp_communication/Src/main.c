#include <stdbool.h>
#include <stdint.h>

#include "board.h"
#include "ethernetif.h"
#include "lwip/init.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/timeouts.h"
#include "lwip/udp.h"
#include "main.h"
#include "netif/ethernet.h"
#include "standalone_runtime.h"
#include "test_config.h"

volatile uint32_t g_udp_rx_packets;
volatile uint32_t g_udp_tx_packets;
volatile uint32_t g_udp_rx_bytes;
volatile uint32_t g_udp_tx_bytes;
volatile uint32_t g_udp_error_count;
volatile uint32_t g_udp_last_peer_ipv4;
volatile uint16_t g_udp_last_peer_port;
volatile uint16_t g_udp_last_payload_length;
volatile uint8_t g_udp_link_up;

static struct netif network_interface;
static struct udp_pcb *echo_server;
static uint8_t echo_buffer[UDP_TEST_MAX_PAYLOAD];

static void echo_receive(void *arg, struct udp_pcb *pcb, struct pbuf *packet,
                         const ip_addr_t *address, uint16_t port) {
  struct pbuf *reply;
  err_t result;
  (void)arg;

  if (packet == NULL) {
    return;
  }
  if (packet->tot_len == 0U || packet->tot_len > sizeof(echo_buffer) ||
      pbuf_copy_partial(packet, echo_buffer, packet->tot_len, 0U) !=
          packet->tot_len) {
    ++g_udp_error_count;
    pbuf_free(packet);
    return;
  }

  ++g_udp_rx_packets;
  g_udp_rx_bytes += packet->tot_len;
  g_udp_last_peer_ipv4 = ip4_addr_get_u32(ip_2_ip4(address));
  g_udp_last_peer_port = port;
  g_udp_last_payload_length = packet->tot_len;

  reply = pbuf_alloc(PBUF_TRANSPORT, packet->tot_len, PBUF_RAM);
  if (reply == NULL ||
      pbuf_take(reply, echo_buffer, packet->tot_len) != ERR_OK) {
    ++g_udp_error_count;
    if (reply != NULL) {
      pbuf_free(reply);
    }
    pbuf_free(packet);
    return;
  }
  result = udp_sendto(pcb, reply, address, port);
  pbuf_free(reply);
  pbuf_free(packet);
  if (result == ERR_OK) {
    ++g_udp_tx_packets;
    g_udp_tx_bytes += g_udp_last_payload_length;
  } else {
    ++g_udp_error_count;
  }
}

static void network_init(void) {
  ip4_addr_t address;
  ip4_addr_t netmask;
  ip4_addr_t gateway;

  IP4_ADDR(&address, UDP_TEST_IP_ADDR0, UDP_TEST_IP_ADDR1,
           UDP_TEST_IP_ADDR2, UDP_TEST_IP_ADDR3);
  IP4_ADDR(&netmask, UDP_TEST_NETMASK_ADDR0, UDP_TEST_NETMASK_ADDR1,
           UDP_TEST_NETMASK_ADDR2, UDP_TEST_NETMASK_ADDR3);
  IP4_ADDR(&gateway, UDP_TEST_GATEWAY_ADDR0, UDP_TEST_GATEWAY_ADDR1,
           UDP_TEST_GATEWAY_ADDR2, UDP_TEST_GATEWAY_ADDR3);

  lwip_init();
  if (netif_add(&network_interface, &address, &netmask, &gateway, NULL,
                ethernetif_init, ethernet_input) == NULL) {
    Error_Handler();
  }
  netif_set_default(&network_interface);
  ethernet_link_check_state(&network_interface);
}

static void echo_init(void) {
  echo_server = udp_new_ip_type(IPADDR_TYPE_V4);
  if (echo_server == NULL ||
      udp_bind(echo_server, IP_ANY_TYPE, UDP_TEST_PORT) != ERR_OK) {
    if (echo_server != NULL) {
      udp_remove(echo_server);
    }
    Error_Handler();
  }
  udp_recv(echo_server, echo_receive, NULL);
}

int main(void) {
  uint32_t last_link_check_ms = 0U;

  standalone_runtime_init(BOARD_PERIPHERAL_NONE);
  if (UDP_TEST_MAX_PAYLOAD == 0U || UDP_TEST_MAX_PAYLOAD > 1472U) {
    Error_Handler();
  }
  board_phy_reset_release();
  network_init();
  echo_init();

  while (1) {
    uint32_t now_ms = HAL_GetTick();
    ethernetif_input(&network_interface);
    sys_check_timeouts();
    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - last_link_check_ms) >= 100U) {
      last_link_check_ms = now_ms;
      ethernet_link_check_state(&network_interface);
      g_udp_link_up = netif_is_link_up(&network_interface) != 0 ? 1U : 0U;
      board_led_set(BOARD_LED_NETWORK, g_udp_link_up != 0U);
    }
    standalone_set_fault(g_udp_error_count != 0U);
    standalone_heartbeat_poll(now_ms);
  }
}
