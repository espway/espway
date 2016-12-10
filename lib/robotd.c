#include <string.h>

#include "ets_sys.h"
#include "osapi.h"
#include "mem.h"
#include "user_interface.h"
#include "espconn.h"
#include "espmissingincludes.h"

#include "indexhtml.inc"

#include "robotd.h"

#define TMP_BUF_SIZE 1024  // MUST be a multiple of 4
#define REQ_DATA_MAX_LENGTH 128
#define MAX_NUM_CLIENTS 4

LOCAL struct espconn esp_conn;
LOCAL esp_tcp esptcp;

typedef enum { REQ_GET_FILE, REQ_WS_UPGRADE, REQ_IGNORE } req_type;
typedef enum { CLIENT_NONE, CLIENT_FILE, CLIENT_WS } client_type;

typedef struct {
    req_type type;
    char data[REQ_DATA_MAX_LENGTH];
} parsed_request;

typedef struct {
    const char *data;
    const char *mimetype;
    size_t data_len;
} rodata_file;

typedef struct {
    client_type type;
    uint8_t ip[4];
    int port;
    size_t to_be_sent;
    const char *data_pointer;
} robotd_client;

LOCAL parsed_request gReq;

static char tmp_buf[TMP_BUF_SIZE];
static robotd_client clients[MAX_NUM_CLIENTS];

static const char RESPONSE_OK[] = "200 OK",
                  NOT_FOUND[] = "404 Not Found",
                  NOT_IMPLEMENTED[] = "501 Not Implemented",
                  SERVICE_UNAVAILABLE[] = "503 Service Unavailable";
static const char MIME_TEXT_PLAIN[] = "text/plain; charset=us-ascii";
static const char MIME_TEXT_HTML[] = "text/html; charset=us-ascii";

static const char HEADER_FORMAT[] =
    "HTTP/1.1 %s\r\n"
    "Content-Length: %u\r\n"
    "Content-Type: %s\r\n"
    "Connection: close\r\n\r\n%s";

static const rodata_file TEST_FILE =
    { INDEX_HTML, MIME_TEXT_HTML, sizeof(INDEX_HTML) - 1 };

static const char WEBSOCK_MAGIC_STRING[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

void ICACHE_FLASH_ATTR robotd_send_header(struct espconn *pespconn,
    const char *res_code, const char *mimetype, size_t data_len,
    const char *data) {
    if (data == NULL) {
        data = "";
    }
    os_sprintf(tmp_buf, HEADER_FORMAT, res_code, data_len, mimetype, data);
    os_printf(tmp_buf);
    espconn_send(pespconn, tmp_buf, os_strlen(tmp_buf));
}

void ICACHE_FLASH_ATTR send_404(struct espconn *pespconn) {
    robotd_send_header(pespconn, NOT_FOUND, MIME_TEXT_PLAIN,
        sizeof(NOT_FOUND - 1), NOT_FOUND);
}

void ICACHE_FLASH_ATTR send_501(struct espconn *pespconn) {
    robotd_send_header(pespconn, NOT_IMPLEMENTED, MIME_TEXT_PLAIN,
        sizeof(NOT_IMPLEMENTED - 1), NOT_IMPLEMENTED);
}

void ICACHE_FLASH_ATTR send_503(struct espconn *pespconn) {
    robotd_send_header(pespconn, SERVICE_UNAVAILABLE, MIME_TEXT_PLAIN,
        sizeof(SERVICE_UNAVAILABLE - 1), SERVICE_UNAVAILABLE);
}

robotd_client * ICACHE_FLASH_ATTR robotd_insert_client(struct espconn *pespconn) {
    robotd_client *pret = NULL;
    for (size_t i = 0; i < MAX_NUM_CLIENTS; ++i) {
        if (clients[i].type == CLIENT_NONE) {
            pret = &clients[i];
            pret->port = pespconn->proto.tcp->remote_port;
            os_memcpy(&pret->ip, &pespconn->proto.tcp->remote_ip, 4);
            break;
        }
    }

    return pret;
}

void ICACHE_FLASH_ATTR robotd_send_file(struct espconn *pespconn,
    const char *res_code, const rodata_file *file) {
    robotd_client *pclient = robotd_insert_client(pespconn);
    if (pclient == NULL) {
        send_503(pespconn);
        return;
    }

    pclient->type = CLIENT_FILE;
    pclient->to_be_sent = file->data_len;
    pclient->data_pointer = file->data;
    os_printf("Going to send %u bytes of data\n", file->data_len);
    robotd_send_header(pespconn, res_code, file->mimetype, file->data_len, NULL);
}

robotd_client * ICACHE_FLASH_ATTR robotd_find_client(struct espconn *pespconn) {
    robotd_client *ret_client = NULL;
    robotd_client *curr_client;

    for (size_t i = 0; i < MAX_NUM_CLIENTS; ++i) {
        curr_client = &clients[i];
        if (curr_client->type == CLIENT_NONE) continue;

        if (curr_client->port == pespconn->proto.tcp->remote_port &&
            os_memcmp(&curr_client->ip[0], &pespconn->proto.tcp->remote_ip[0], 4) == 0) {
            ret_client = curr_client;
            break;
        }
    }

    return ret_client;
}

void ICACHE_FLASH_ATTR robotd_delete_client(struct espconn *pespconn) {
    robotd_client *pclient = robotd_find_client(pespconn);
    if (pclient != NULL) {
        pclient->type = CLIENT_NONE;
    }
}

void ICACHE_FLASH_ATTR robotd_do_websocket_handshake(struct espconn *pespconn,
    const char *ws_key) {
}

void ICACHE_FLASH_ATTR robotd_parse_http_request(char *data,
    unsigned short length, parsed_request *pReq) {
    char *method = strtok(data, " ");
    if (os_strcmp(method, "GET") == 0) {
        char *uri = strtok(NULL, " ");
        char *version = strtok(NULL, "\r");

        if (os_strcmp(version, "HTTP/1.1") == 0 &&
            os_strlen(uri) < REQ_DATA_MAX_LENGTH) {
            if (os_strcmp(uri, "/ws") == 0) {
                os_printf("websock handler\n");
                char *row = strtok(NULL, "\n");
                bool found_key = false;
                bool version_13 = false;
                while (row != NULL) {
                    if (!found_key && os_strlen(row) == 44 &&
                        os_strncmp(row, "Sec-WebSocket-Key: ", 19) == 0) {
                        os_memcpy(pReq->data, &row[19], 24);
                        pReq->data[24] = '\0';
                        found_key = true;
                        os_printf("Found websock key: %s\n", pReq->data);
                    }
                    if (!version_13 &&
                        os_strcmp(row, "Sec-WebSocket-Version: 13\r") == 0) {
                        os_printf("Right websock version\n");
                        version_13 = true;
                    }

                    if (found_key && version_13) {
                        pReq->type = REQ_WS_UPGRADE;
                        break;
                    }
                    row = strtok(NULL, "\n");
                }
            } else {
                pReq->type = REQ_GET_FILE;
                strncpy(pReq->data, uri, REQ_DATA_MAX_LENGTH);
            }
            return;
        }
    }

    pReq->type = REQ_IGNORE;
}

LOCAL void ICACHE_FLASH_ATTR robotd_sent_cb(void *arg) {
    //data sent successfully
    struct espconn *pespconn = (struct espconn *)arg;

    robotd_client *pclient = robotd_find_client(pespconn);
    if (pclient == NULL) return;

    if (pclient->type == CLIENT_FILE &&
        pclient->data_pointer != NULL && pclient->to_be_sent != 0) {
        os_printf("%u bytes of data to be sent\n", pclient->to_be_sent);

        size_t data_len = pclient->to_be_sent > TMP_BUF_SIZE ?
            TMP_BUF_SIZE : pclient->to_be_sent;
        os_memcpy(tmp_buf, pclient->data_pointer,
            GET_ALIGNED_SIZE(data_len));
        espconn_send(pespconn, tmp_buf, data_len);

        pclient->to_be_sent -= data_len;
        pclient->data_pointer += data_len;

        if (pclient->to_be_sent == 0) {
            pclient->data_pointer = NULL;
        }
    }
}

LOCAL void ICACHE_FLASH_ATTR
robotd_recv_cb(void *arg, char *pusrdata, unsigned short length)
{
    struct espconn *pespconn = arg;

    robotd_parse_http_request(pusrdata, length, &gReq);

    const char *response;
    size_t res_len = 0;
    if (gReq.type == REQ_GET_FILE) {
        if (os_strcmp(gReq.data, "/") == 0) {
            robotd_send_file(pespconn, RESPONSE_OK, &TEST_FILE);
            os_printf("Sent test content\n", res_len);
        } else {
            send_404(pespconn);
            os_printf("Sent 404\n", res_len);
        }
    } else if (gReq.type == REQ_WS_UPGRADE) {
        robotd_do_websocket_handshake(pespconn, gReq.data);
    } else {
        send_501(pespconn);
        os_printf("Sent 501\n", res_len);
    }
}

LOCAL void ICACHE_FLASH_ATTR
robotd_discon_cb(void *arg)
{
    //tcp disconnect successfully
    robotd_delete_client((struct espconn *)arg);

    os_printf("tcp disconnect succeed !!! \r\n");
}

LOCAL void ICACHE_FLASH_ATTR
robotd_recon_cb(void *arg, sint8 err)
{
    //error occured , tcp connection broke.
    robotd_delete_client((struct espconn *)arg);

    os_printf("reconnect callback, error code %d !!! \r\n",err);
}

LOCAL void ICACHE_FLASH_ATTR
robotd_listen(void *arg)
{
    struct espconn *pesp_conn = arg;

    espconn_regist_recvcb(pesp_conn, robotd_recv_cb);
    espconn_regist_reconcb(pesp_conn, robotd_recon_cb);
    espconn_regist_disconcb(pesp_conn, robotd_discon_cb);
    espconn_regist_sentcb(pesp_conn, robotd_sent_cb);
}

void ICACHE_FLASH_ATTR
robotd_init(uint32_t port)
{
    for (size_t i = 0; i < MAX_NUM_CLIENTS; ++i) {
        os_memset(&clients[i], 0, sizeof(robotd_client));
    }

    esp_conn.type = ESPCONN_TCP;
    esp_conn.state = ESPCONN_NONE;
    esp_conn.proto.tcp = &esptcp;
    esp_conn.proto.tcp->local_port = port;
    espconn_regist_connectcb(&esp_conn, robotd_listen);

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

