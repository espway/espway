#include "ets_sys.h"
#include "osapi.h"
#include "mem.h"
#include "user_interface.h"
#include "espconn.h"
#include "espmissingincludes.h"

LOCAL struct espconn esp_conn;
LOCAL esp_tcp esptcp;

typedef enum { REQ_GET_FILE, REQ_WS_UPGRADE, REQ_IGNORE } req_type;

typedef struct {
    req_type type;
    char uri[128];
} parsed_request;

LOCAL parsed_request gReq;

const size_t RESPONSE_BUF_SIZE = 20480;
char *response_buf;

char *NOT_IMPLEMENTED_RESPONSE = "HTTP/1.1 501 Not Implemented\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";

void ICACHE_FLASH_ATTR send_not_implemented() {
}

void ICACHE_FLASH_ATTR parse_http_request(char *data, unsigned short length,
    parsed_request *pReq) {
    char *method = strtok(data, " ");
    if (os_strcmp(method, "GET") == 0) {
        os_printf("got a GET request\n");
        char *uri = strtok(NULL, " ");
        os_printf("URI: %s\n", uri);
        char *version = strtok(NULL, "\r");
        os_printf("Protocol version: %s\n", version);

        if (os_strcmp(version, "HTTP/1.1") == 0) {
            os_printf("Can handle the request\n");
            pReq->type = REQ_GET_FILE;
            os_strncpy(pReq->uri, uri, 128);
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

char *test_data = "HTTP/1.1 200 OK\r\nContent-Length: 11\r\nContent-Encoding: text/plain\r\nConnection: close\r\n\r\nHello, ESP!";

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

   if (gReq.type == REQ_GET_FILE) {
       espconn_send(pespconn, test_data, os_strlen(test_data));
   } else {
       espconn_send(pespconn, NOT_IMPLEMENTED_RESPONSE, os_strlen(NOT_IMPLEMENTED_RESPONSE));
   }
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

    response_buf = (char *)os_malloc(sizeof(char) * RESPONSE_BUF_SIZE);

    os_printf("espconn_accept [%d] !!! \r\n", ret);
}

