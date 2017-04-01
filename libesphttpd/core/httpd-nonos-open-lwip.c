/*
ESP8266 web server - platform-dependent routines, nonos version
*/

#if !defined(FREERTOS) && defined(LWIP_OPEN_SRC)

#include <esp8266.h>
#include <lwip/tcp.h>

#include "httpd.h"
#include "platform.h"
#include "httpd-platform.h"

struct remote_info {
	ip_addr_t remote_ip;
	uint16_t remote_port;
	bool is_idle;
};

void ICACHE_FLASH_ATTR DEBUG_printf(char *format, ...) {
	return;
}

//Set/clear global httpd lock.
//Not needed on nonoos.
void ICACHE_FLASH_ATTR httpdPlatLock() {
}
void ICACHE_FLASH_ATTR httpdPlatUnlock() {
}

static void ICACHE_FLASH_ATTR platSetIsIdle(void *arg, bool is_idle) {
	if (arg == NULL) return;
	struct remote_info *info = (struct remote_info *)arg;
	info->is_idle = is_idle;
}

static void ICACHE_FLASH_ATTR platErrorCb(void *arg, err_t err) {
	//From ESP8266 SDK
	//If still no response, considers it as TCP connection broke, goes into espconn_reconnect_callback.
	//Just call disconnect to clean up pool and close connection.
	if (arg == NULL) return;
	struct remote_info *rinfo = (struct remote_info *)arg;
	httpdDisconCb(NULL, (char*)&rinfo->remote_ip.addr, rinfo->remote_port);
	os_free(rinfo);
}

static err_t ICACHE_FLASH_ATTR platRecvCb(void *arg, struct tcp_pcb *conn,
	struct pbuf *pb, err_t err) {
	if (pb == NULL) {
		if (arg != NULL) {
			os_free(arg);
			tcp_arg(conn, NULL);
		}
		httpdPlatDisconnect(conn);
	} else {
		httpdRecvCb(conn, (char*)&conn->remote_ip.addr, conn->remote_port,
			pb->payload, pb->len);
		tcp_recved(conn, pb->len);
		pbuf_free(pb);
		platSetIsIdle(arg, false);
	}
	return ERR_OK;
}

static err_t ICACHE_FLASH_ATTR platSentCb(void *arg, struct tcp_pcb *conn,
	uint16_t len) {
	httpdSentCb(conn, (char*)&conn->remote_ip.addr, conn->remote_port);
	platSetIsIdle(arg, false);
	return ERR_OK;
}

static err_t ICACHE_FLASH_ATTR platPollCb(void *arg, struct tcp_pcb *conn) {
	os_printf("Poll for %u\n", conn->remote_port);
	if (arg == NULL) return ERR_OK;
	struct remote_info *info = (struct remote_info *)arg;
	if (info->is_idle) {
		httpdPlatDisconnect(conn);
		os_free(info);
		return ERR_OK;
	}
	info->is_idle = true;
	return ERR_OK;
}

static err_t ICACHE_FLASH_ATTR platConnCb(void *arg, ConnTypePtr conn, err_t err) {
	if (httpdConnectCb(conn, (char*)&conn->remote_ip.addr, conn->remote_port)) {
		tcp_err(conn, platErrorCb);
		tcp_sent(conn, platSentCb);
		tcp_recv(conn, platRecvCb);
		tcp_poll(conn, platPollCb, 10);
		struct remote_info *rinfo = os_malloc(sizeof(struct remote_info));
		rinfo->remote_ip = conn->remote_ip;
		rinfo->remote_port = conn->remote_port;
		rinfo->is_idle = false;
		tcp_arg(conn, rinfo);
		tcp_accepted(conn);
		return ERR_OK;
	} else {
		tcp_abort(conn);
		return ERR_ABRT;
	}
}

int ICACHE_FLASH_ATTR httpdPlatSendData(ConnTypePtr conn, char *buff, int len) {
	if (tcp_sndbuf(conn) < len ||
		tcp_write(conn, buff, len, TCP_WRITE_FLAG_COPY) != ERR_OK) {
		return 0;
	}
	tcp_output(conn);
	return 1;
}

void ICACHE_FLASH_ATTR httpdPlatDisconnect(ConnTypePtr conn) {
	httpdDisconCb(conn, (char*)&conn->remote_ip.addr, conn->remote_port);
	tcp_sent(conn, NULL);
	tcp_recv(conn, NULL);
	tcp_err(conn, NULL);
	tcp_poll(conn, NULL, 0);
	tcp_close(conn);
}

void ICACHE_FLASH_ATTR httpdPlatDisableTimeout(ConnTypePtr conn) {
	tcp_poll(conn, NULL, 0);
}

//Initialize listening socket, do general initialization
void ICACHE_FLASH_ATTR httpdPlatInit(int port, int maxConnCt) {
	struct tcp_pcb *listen_pcb = tcp_new();
	tcp_bind(listen_pcb, IP_ADDR_ANY, port);
	listen_pcb = tcp_listen_with_backlog(listen_pcb, maxConnCt);
	tcp_accept(listen_pcb, platConnCb);
}

#endif
