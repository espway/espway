#include <string.h>

#include "ets_sys.h"
#include "osapi.h"
#include "mem.h"
#include "user_interface.h"
#include "espconn.h"
#include "espmissingincludes.h"

#include "indexhtml.inc"

#include "robotd.h"

#define RESPONSE_BUF_SIZE 1024  // MUST be a multiple of 4
#define URI_MAX_LENGTH 128

LOCAL struct espconn esp_conn;
LOCAL esp_tcp esptcp;

typedef enum { REQ_GET_FILE, REQ_WS_UPGRADE, REQ_IGNORE } req_type;

typedef struct {
    req_type type;
    char uri[URI_MAX_LENGTH];
} parsed_request;

typedef struct {
    const char *data;
    const char *mimetype;
    size_t data_len;
} rodata_file;

LOCAL parsed_request gReq;

static char response_buf[RESPONSE_BUF_SIZE];
static const char *send_file_pointer = NULL;
static size_t to_be_sent = 0;

static const char RESPONSE_OK[] = "200 OK",
                  NOT_FOUND[] = "404 Not Found",
                  NOT_IMPLEMENTED[] = "501 Not Implemented";
static const char MIME_TEXT_PLAIN[] = "text/plain; charset=us-ascii";
static const char MIME_TEXT_HTML[] = "text/html; charset=us-ascii";

static const char HEADER_FORMAT[] =
    "HTTP/1.1 %s\r\n"
    "Content-Length: %u\r\n"
    "Content-Type: %s\r\n"
    "Connection: close\r\n\r\n";

static const char TEST_FILE_DATA[] ICACHE_RODATA_ATTR =
    "Hello, World!";
static const rodata_file TEST_FILE =
    { INDEX_HTML, MIME_TEXT_HTML, sizeof(INDEX_HTML) - 1 };
static const rodata_file NOT_IMPLEMENTED_FILE =
    { NOT_IMPLEMENTED, MIME_TEXT_PLAIN, sizeof(NOT_IMPLEMENTED) - 1 };
static const rodata_file NOT_FOUND_FILE =
    { NOT_FOUND, MIME_TEXT_PLAIN, sizeof(NOT_FOUND) - 1 };

void ICACHE_FLASH_ATTR send_header(struct espconn *pespconn,
    const char *res_code, const char *mimetype, size_t data_len) {
    os_sprintf(response_buf, HEADER_FORMAT, res_code, data_len, mimetype);
    os_printf(response_buf);
    espconn_send(pespconn, response_buf, os_strlen(response_buf));
}

void ICACHE_FLASH_ATTR send_file(struct espconn *pespconn,
    const char *res_code, const rodata_file *file) {
    to_be_sent = file->data_len;
    send_file_pointer = file->data;
    os_printf("Going to send %u bytes of data\n", file->data_len);
    send_header(pespconn, res_code, file->mimetype, file->data_len);
}

void ICACHE_FLASH_ATTR send_404(struct espconn *pespconn) {
    send_file(pespconn, NOT_FOUND, &NOT_FOUND_FILE);
}

void ICACHE_FLASH_ATTR send_501(struct espconn *pespconn) {
    send_file(pespconn, NOT_IMPLEMENTED, &NOT_IMPLEMENTED_FILE);
}

void ICACHE_FLASH_ATTR parse_http_request(char *data, unsigned short length,
    parsed_request *pReq) {
    char *method = strtok(data, " ");
    if (os_strcmp(method, "GET") == 0) {
        char *uri = strtok(NULL, " ");
        char *version = strtok(NULL, "\r");

        if (os_strcmp(version, "HTTP/1.1") == 0 &&
            os_strlen(uri) < URI_MAX_LENGTH) {
            pReq->type = REQ_GET_FILE;
            strncpy(pReq->uri, uri, URI_MAX_LENGTH);
            return;
        }
    }

    pReq->type = REQ_IGNORE;
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
   struct espconn *pespconn = (struct espconn *)arg;

    os_printf("tcp sent cb \r\n");

    if (send_file_pointer != NULL && to_be_sent != 0) {
        os_printf("%u bytes of data to be sent\n", to_be_sent);

        size_t data_len = to_be_sent > RESPONSE_BUF_SIZE ? RESPONSE_BUF_SIZE :
            to_be_sent;
        os_memcpy(response_buf, send_file_pointer, GET_ALIGNED_SIZE(data_len));
        espconn_send(pespconn, response_buf, data_len);

        to_be_sent -= data_len;
        send_file_pointer += data_len;

        if (to_be_sent == 0) {
            send_file_pointer = NULL;
        }
    }
}

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

    parse_http_request(pusrdata, length, &gReq);

    const char *response;
    size_t res_len = 0;
    if (gReq.type == REQ_GET_FILE) {
        if (os_strcmp(gReq.uri, "/") == 0) {
            send_file(pespconn, RESPONSE_OK, &TEST_FILE);
            os_printf("Sent test content\n", res_len);
        } else {
            send_404(pespconn);
            os_printf("Sent 404\n", res_len);
        }
    } else {
        send_501(pespconn);
        os_printf("Sent 501\n", res_len);
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

void ICACHE_FLASH_ATTR
robotd_init_ap(char *ssid) {
    struct softap_config conf;
    wifi_softap_get_config(&conf);

    os_memset(conf.ssid, 0, 32);
    os_memset(conf.password, 0, 64);
    os_memcpy(conf.ssid, ssid, strlen(ssid));
    conf.ssid_len = 0;
    conf.beacon_interval = 100;
    conf.max_connection = 1;
    conf.authmode = AUTH_OPEN;

    wifi_softap_set_config(&conf);
    wifi_set_opmode(SOFTAP_MODE);
}

void ICACHE_FLASH_ATTR
robotd_deinit() {
}

