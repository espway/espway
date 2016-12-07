#include <string.h>

#include "ets_sys.h"
#include "osapi.h"
#include "mem.h"
#include "user_interface.h"
#include "espconn.h"
#include "espmissingincludes.h"

#include "robotd.h"

#define RESPONSE_BUF_SIZE 1460
#define URI_MAX_LENGTH 128
LOCAL struct espconn esp_conn;
LOCAL esp_tcp esptcp;

typedef enum { REQ_GET_FILE, REQ_WS_UPGRADE, REQ_IGNORE } req_type;

typedef struct {
    req_type type;
    char uri[URI_MAX_LENGTH];
} parsed_request;

LOCAL parsed_request gReq;

char response_buf[RESPONSE_BUF_SIZE];

// FIXME send character encoding in header

void ICACHE_FLASH_ATTR parse_http_request(char *data, unsigned short length,
    parsed_request *pReq) {
    // FIXME prefer os methods over strtok
    char *method = strtok(data, " ");
    if (os_strcmp(method, "GET") == 0) {
        os_printf("got a GET request\n");
        char *uri = strtok(NULL, " ");
        os_printf("URI: %s\n", uri);
        char *version = strtok(NULL, "\r");
        os_printf("Protocol version: %s\n", version);

        if (os_strcmp(version, "HTTP/1.1") == 0 &&
            os_strlen(uri) < URI_MAX_LENGTH) {
            os_printf("Can handle the request\n");
            pReq->type = REQ_GET_FILE;
            strncpy(pReq->uri, uri, URI_MAX_LENGTH);
            return;
        }
    }

    pReq->type = REQ_IGNORE;
    os_printf("got ignored request\n");
}

/******************************************************************************
 * FunctionName : tcp_server_sent_cb
 * Description  : data sent callback.
 * Parameters   : arg -- Additional argument to pass to the callback function
 * Returns      : none
*******************************************************************************/
LOCAL void ICACHE_FLASH_ATTR
tcp_server_sent_cb(void *arg)
{
   //data sent successfully

    os_printf("tcp sent cb \r\n");
}

static const char TEST_OK_RESPONSE[] ICACHE_RODATA_ATTR = "HTTP/1.1 200 OK\r\nContent-Length: 11\r\nContent-Type: text/plain; charset=us-ascii\r\nConnection: close\r\n\r\nHello, ESP!";
static const char NOT_FOUND_RESPONSE[] ICACHE_RODATA_ATTR = "HTTP/1.1 404 Not Found\r\nContent-Length: 14\r\nContent-Type: text/plain; charset=us-ascii\r\nConnection: close\r\n\r\nFile not found";
static const char NOT_IMPLEMENTED_RESPONSE[] ICACHE_RODATA_ATTR = "HTTP/1.1 501 Not Implemented\r\nContent-Length: 23\r\nContent-Type: text/plain; charset=us-ascii\r\nConnection: close\r\n\r\nThis is not implemented";

/******************************************************************************
 * FunctionName : tcp_server_recv_cb
 * Description  : receive callback.
 * Parameters   : arg -- Additional argument to pass to the callback function
 * Returns      : none
*******************************************************************************/
LOCAL void ICACHE_FLASH_ATTR
tcp_server_recv_cb(void *arg, char *pusrdata, unsigned short length)
{
   //received some data from tcp connection
   
   struct espconn *pespconn = arg;
   os_printf("tcp recv : %s \r\n", pusrdata);

   parse_http_request(pusrdata, length, &gReq);

   const char *response;
   size_t res_len = 0;
   if (gReq.type == REQ_GET_FILE) {
       if (os_strcmp(gReq.uri, "/") == 0) {
           response = TEST_OK_RESPONSE;
           os_printf("Sent test content, size %u\n", res_len);
       } else {
           response = NOT_FOUND_RESPONSE;
           os_printf("Sent 404 not found, size %u\n", res_len);
       }
   } else {
       response = NOT_IMPLEMENTED_RESPONSE;
   }

   res_len = GET_ALIGN_STRING_LEN(response);
   espconn_send(pespconn, (char *)response, res_len);
}

/******************************************************************************
 * FunctionName : tcp_server_discon_cb
 * Description  : disconnect callback.
 * Parameters   : arg -- Additional argument to pass to the callback function
 * Returns      : none
*******************************************************************************/
LOCAL void ICACHE_FLASH_ATTR
tcp_server_discon_cb(void *arg)
{
   //tcp disconnect successfully
   
    os_printf("tcp disconnect succeed !!! \r\n");
}

/******************************************************************************
 * FunctionName : tcp_server_recon_cb
 * Description  : reconnect callback, error occured in TCP connection.
 * Parameters   : arg -- Additional argument to pass to the callback function
 * Returns      : none
*******************************************************************************/
LOCAL void ICACHE_FLASH_ATTR
tcp_server_recon_cb(void *arg, sint8 err)
{
   //error occured , tcp connection broke.
   
    os_printf("reconnect callback, error code %d !!! \r\n",err);
}

/******************************************************************************
 * FunctionName : tcp_server_listen
 * Description  : TCP server listened a connection successfully
 * Parameters   : arg -- Additional argument to pass to the callback function
 * Returns      : none
*******************************************************************************/
LOCAL void ICACHE_FLASH_ATTR
tcp_server_listen(void *arg)
{
    struct espconn *pesp_conn = arg;
    os_printf("tcp_server_listen !!! \r\n");

    espconn_regist_recvcb(pesp_conn, tcp_server_recv_cb);
    espconn_regist_reconcb(pesp_conn, tcp_server_recon_cb);
    espconn_regist_disconcb(pesp_conn, tcp_server_discon_cb);
    espconn_regist_sentcb(pesp_conn, tcp_server_sent_cb);
}

void ICACHE_FLASH_ATTR
robotd_init(uint32_t port)
{
    esp_conn.type = ESPCONN_TCP;
    esp_conn.state = ESPCONN_NONE;
    esp_conn.proto.tcp = &esptcp;
    esp_conn.proto.tcp->local_port = port;
    espconn_regist_connectcb(&esp_conn, tcp_server_listen);

    sint8 ret = espconn_accept(&esp_conn);

    os_printf("espconn_accept [%d] !!! \r\n", ret);
}

