/* Very basic LWIP & FreeRTOS-based DHCP server
 *
 * Header file contains default configuration for the DHCP server.
 *
 *
 * Part of esp-open-rtos
 * Copyright (C) 2015 Superhouse Automation Pty Ltd
 * BSD Licensed as described in the file LICENSE
 */
#ifndef _DHCPSERVER_H
#define _DHCPSERVER_H

// Set the below options to 1 to add router and DNS server fields to the DHCP
// responses. This allows you to use the ESP8266 AP as a DNS server (e.g. to
// implement a captive portal)
#define DHCPSERVER_ROUTER_OPTION 1
#define DHCPSERVER_DNS_SERVER_OPTION 1

#ifndef DHCPSERVER_LEASE_TIME
#define DHCPSERVER_LEASE_TIME 3600
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Start DHCP server.

   Static IP of server should already be set and network interface enabled.

   first_client_addr is the IP address of the first lease to be handed
   to a client.  Subsequent lease addresses are calculated by
   incrementing the final octet of the IPv4 address, up to max_leases.
*/
void dhcpserver_start(const ip_addr_t *first_client_addr, uint8_t max_leases);

void dhcpserver_get_lease(const ip_addr_t *first_client_addr, uint8_t max_leases);

/* Stop DHCP server.
 */
void dhcpserver_stop(void);

#ifdef __cplusplus
}
#endif

#endif
