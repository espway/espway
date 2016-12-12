#include <string.h>

#include "ets_sys.h"
#include "osapi.h"
#include "mem.h"
#include "user_interface.h"
#include "espconn.h"
#include "espmissingincludes.h"
#include "sha1.h"
#include "base64.h"
#include "robotd.h"

#include "indexhtml.inc"

#define GET_ALIGNED_STRING_LEN(str) ((os_strlen(str) + 3) & ~3)
#define GET_ALIGNED_SIZE(s) (((s) + 3) & ~3)

static struct espconn esp_conn;
static esp_tcp esptcp;

typedef enum { REQ_GET_FILE, REQ_WS_UPGRADE, REQ_IGNORE } req_type;

typedef struct {
    req_type type;
    char data[REQ_DATA_MAX_LENGTH];
} parsed_request;

typedef struct {
    const char *data;
    const char *mimetype;
    size_t data_len;
} rodata_file;

static parsed_request gReq;

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

static const char WEBSOCKET_MAGIC_STRING[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

static SHA1_CTX sha1_ctx;

static void ICACHE_FLASH_ATTR
robotd_send_header(struct espconn *pespconn,
    const char *res_code, const char *mimetype, size_t data_len,
    const char *data) {
    if (data == NULL) {
        data = "";
    }
    os_sprintf(tmp_buf, HEADER_FORMAT, res_code, data_len, mimetype, data);
    os_printf(tmp_buf);
    espconn_send(pespconn, tmp_buf, os_strlen(tmp_buf));
}

static void ICACHE_FLASH_ATTR
send_404(struct espconn *pespconn) {
    robotd_send_header(pespconn, NOT_FOUND, MIME_TEXT_PLAIN,
        sizeof(NOT_FOUND - 1), NOT_FOUND);
}

static void ICACHE_FLASH_ATTR
send_501(struct espconn *pespconn) {
    robotd_send_header(pespconn, NOT_IMPLEMENTED, MIME_TEXT_PLAIN,
        sizeof(NOT_IMPLEMENTED - 1), NOT_IMPLEMENTED);
}

static void ICACHE_FLASH_ATTR
send_503(struct espconn *pespconn) {
    robotd_send_header(pespconn, SERVICE_UNAVAILABLE, MIME_TEXT_PLAIN,
        sizeof(SERVICE_UNAVAILABLE - 1), SERVICE_UNAVAILABLE);
}

static robotd_client * ICACHE_FLASH_ATTR
robotd_insert_client(struct espconn *pespconn) {
    robotd_client *pret = NULL;
    for (size_t i = 0; i < MAX_NUM_CLIENTS; ++i) {
        if (clients[i].type == CLIENT_NONE) {
            os_printf("inserted client at %u\n", i);
            pret = &clients[i];
            pret->conn = pespconn;
            pret->port = pespconn->proto.tcp->remote_port;
            os_memcpy(&pret->ip, &pespconn->proto.tcp->remote_ip, 4);
            break;
        }
    }

    return pret;
}

static void ICACHE_FLASH_ATTR
robotd_send_file(struct espconn *pespconn,
    const char *res_code, const rodata_file *file) {
    robotd_client *pclient = robotd_insert_client(pespconn);
    if (pclient == NULL) {
        send_503(pespconn);
        return;
    }

    pclient->type = CLIENT_FILE;
    pclient->send_data_length = file->data_len;
    pclient->send_data_pointer = file->data;
    os_printf("Going to send %u bytes of data\n", file->data_len);
    robotd_send_header(pespconn, res_code, file->mimetype, file->data_len, NULL);
}

static robotd_client * ICACHE_FLASH_ATTR
robotd_find_client(struct espconn *pespconn) {
    robotd_client *ret_client = NULL;
    robotd_client *curr_client;

    for (size_t i = 0; i < MAX_NUM_CLIENTS; ++i) {
        curr_client = &clients[i];
        if (curr_client->type == CLIENT_NONE) continue;

        if (curr_client->port == pespconn->proto.tcp->remote_port &&
            os_memcmp(&curr_client->ip[0], &pespconn->proto.tcp->remote_ip[0], 4) == 0) {
            os_printf("found client at slot %u\n", i);
            ret_client = curr_client;
            break;
        }
    }

    return ret_client;
}

static void ICACHE_FLASH_ATTR robotd_delete_client(struct espconn *pespconn) {
    os_printf("deleting client...\n");
    robotd_client *pclient = robotd_find_client(pespconn);
    if (pclient != NULL) {
        pclient->type = CLIENT_NONE;
    }
}

static void ICACHE_FLASH_ATTR
robotd_do_websocket_handshake(struct espconn *pespconn, const char *ws_key) {
    os_memcpy(tmp_buf, ws_key, 24);
    os_memcpy(&tmp_buf[24], WEBSOCKET_MAGIC_STRING, 36);

    sha1_init(&sha1_ctx);
    sha1_update(&sha1_ctx, tmp_buf, 60);
    sha1_final(&sha1_ctx, &tmp_buf[60]);

    char b64_buf[29];
    size_t hash_size = base64_encode(&tmp_buf[60], b64_buf, 20, 0);
    b64_buf[28] = '\0';
    os_sprintf(tmp_buf,
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: upgrade\r\n"
        "Sec-Websocket-Accept: %s\r\n\r\n", b64_buf);
    os_printf(tmp_buf);

    robotd_client *pclient = robotd_insert_client(pespconn);
    pclient->type = CLIENT_WS;
    espconn_regist_time(pespconn, 7200, 1);
    espconn_send(pespconn, tmp_buf, os_strlen(tmp_buf));
}

static void ICACHE_FLASH_ATTR robotd_parse_http_request(char *data,
    unsigned short length, parsed_request *pReq) {
    char *method = strtok(data, " ");
    if (os_strcmp(method, "GET") == 0) {
        char *uri = strtok(NULL, " ");
        char *version = strtok(NULL, "\r");

        if (os_strcmp(version, "HTTP/1.1") == 0 &&
            os_strlen(uri) < REQ_DATA_MAX_LENGTH) {
            if (os_strcmp(uri, "/ws") == 0) {
                char *row = strtok(NULL, "\n");
                bool found_key = false;
                bool version_13 = false;
                bool found_connection_upgrade = false;
                bool found_upgrade_websocket = false;
                while (row != NULL) {
                    if (!found_key && os_strlen(row) == 44 &&
                        os_strncmp(row, "Sec-WebSocket-Key: ", 19) == 0) {
                        os_memcpy(pReq->data, &row[19], 24);
                        found_key = true;
                    } else if (!version_13 &&
                        os_strcmp(row, "Sec-WebSocket-Version: 13\r") == 0) {
                        version_13 = true;
                    } else if (!found_connection_upgrade &&
                        os_strcmp(row, "Connection: upgrade\r")) {
                        found_connection_upgrade = true;
                    } else if (!found_upgrade_websocket &&
                        os_strcmp(row, "Upgrade: websocket\r")) {
                        found_upgrade_websocket = true;
                    }

                    if (found_key && version_13 && found_upgrade_websocket &&
                        found_connection_upgrade) {
                        pReq->type = REQ_WS_UPGRADE;
                        return;
                    }
                    row = strtok(NULL, "\n");
                }
            } else {
                pReq->type = REQ_GET_FILE;
                strncpy(pReq->data, uri, REQ_DATA_MAX_LENGTH);
                return;
            }
        }
    }

    pReq->type = REQ_IGNORE;
}

static void ICACHE_FLASH_ATTR robotd_sent_cb(void *arg) {
    //data sent successfully
    struct espconn *pespconn = (struct espconn *)arg;

    robotd_client *pclient = robotd_find_client(pespconn);
    if (pclient == NULL) return;

    if (pclient->type == CLIENT_FILE &&
        pclient->send_data_pointer != NULL && pclient->send_data_length != 0) {

        size_t data_len = pclient->send_data_length > TMP_BUF_SIZE ?
            TMP_BUF_SIZE : pclient->send_data_length;
        os_memcpy(tmp_buf, pclient->send_data_pointer,
            GET_ALIGNED_SIZE(data_len));
        espconn_send(pespconn, tmp_buf, data_len);

        pclient->send_data_length -= data_len;
        pclient->send_data_pointer += data_len;

        if (pclient->send_data_length == 0) {
            pclient->send_data_pointer = NULL;
        }
    }
}

static size_t ICACHE_FLASH_ATTR
robotd_websocket_prepare_header(uint8_t opcode, size_t len) {
    uint8_t *tmp = (uint8_t *)tmp_buf;
    tmp[0] = opcode & 0x0f;
    tmp[0] |= 0x80;
    size_t offset = 2;
    if (len > 125) {
        if (len > UINT16_MAX) {
            os_printf("Websocket: too long data buffer\n");
        }
        tmp[1] = 126;
        tmp[2] = len && 0xff;
        tmp[3] = (len && 0xff00) >> 8;
        offset = 4;
    } else {
        tmp_buf[1] = len;
    }

    return offset;
}

void ICACHE_FLASH_ATTR
robotd_websocket_send_all(uint8_t opcode, char *data, size_t len) {
    size_t offset = robotd_websocket_prepare_header(opcode, len);
    os_memcpy(tmp_buf, data, offset + len);
    for (size_t i = 0; i < MAX_NUM_CLIENTS; ++i) {
        if (clients[i].type == CLIENT_WS) {
            espconn_send(clients[i].conn, tmp_buf, offset + len);
        }
    }
}

void ICACHE_FLASH_ATTR
robotd_websocket_send(robotd_client *pclient, uint8_t opcode,
    char *data, size_t len) {
    size_t offset = robotd_websocket_prepare_header(opcode, len);
    os_memcpy(tmp_buf, data, offset + len);
    espconn_send(pclient->conn, tmp_buf, offset + len);
}

static void ICACHE_FLASH_ATTR
robotd_handle_websocket_frame(robotd_client *pclient, char *data,
    unsigned short length) {
    size_t data_offset = 0;

    if (pclient->expected_data_length == 0) {
        if (length < 2) {
            os_printf("Received data too short\n");
            return;
        }

        bool fin = (data[0] & 0x80) >> 7;
        uint8_t opcode = data[0] & 0x0f;
        bool mask = (data[1] & 0x80) >> 7;
        uint8_t len1 = data[1] & 0x7f;
        size_t len = 0;
        if (len1 == 126) {
            if (length < 8) {
                os_printf("WS: received data too short\n");
                return;
            }
            len = data[3];
            len |= data[2] << 8;
            data_offset += 4;
        } else {
            if (length < 6) {
                os_printf("WS: received data too short\n");
                return;
            }
            len = len1;
            data_offset += 2;
        }

        if (len1 == 127) {
            os_printf("Websocket error: too long data\n", len);
            return;
        }

        os_printf(
            "Decoded ws frame:\n"
            "FIN: %d\n"
            "opcode: 0x%x\n"
            "mask: %d\n"
            "length: %u\n", fin, opcode, mask, len);

        if (mask) {
            os_memcpy(pclient->recv_mask, &data[data_offset], 4);
            data_offset += 4;
        } else {
            os_printf("Got unmasked data, something is wrong\n");
        }

        pclient->expected_data_length += len;
        pclient->fin = fin;

        if (opcode != WS_OPCODE_CONTINUATION) {
            pclient->recv_opcode = opcode;
            pclient->recv_data_length = 0;
            pclient->recv_buf_data_length = 0;
        }
    }

    int imod4 = 0;
    for (size_t i = data_offset; i < length; ++i) {
        // Leave space for string termination
        if (pclient->recv_buf_data_length >= RECV_BUF_SIZE - 1) {
            os_printf("Receive buffer size exceeded\n");
            break;
        }

        pclient->recv_buf[pclient->recv_buf_data_length] =
            data[i] ^ pclient->recv_mask[imod4];
        pclient->recv_buf_data_length += 1;

        if (++imod4 == 4) {
            imod4 = 0;
        }
    }

    pclient->recv_data_length += length - data_offset;

    if (pclient->recv_data_length == pclient->expected_data_length &&
        pclient->fin) {
        pclient->expected_data_length = 0;
        if (pclient->recv_opcode == WS_OPCODE_TEXT) {
            pclient->recv_buf[pclient->recv_buf_data_length] = '\0';
            os_printf("Received text: %s\n", pclient->recv_buf);
        } else if (pclient->recv_opcode == WS_OPCODE_BIN) {
            os_printf("Received binary data: ");
            for (size_t i = 0; i < pclient->recv_buf_data_length; ++i) {
                os_printf("0x%x ", pclient->recv_buf[i]);
            }
            os_printf("\n");
        } else if (pclient->recv_opcode == WS_OPCODE_PING) {
            os_printf("Received ping, sent pong\n");
            robotd_websocket_send(pclient, WS_OPCODE_PONG,
                pclient->recv_buf, pclient->recv_buf_data_length);
        } else if (pclient->recv_opcode == WS_OPCODE_PONG) {
            os_printf("Received pong\n");
        } else if (pclient->recv_opcode == WS_OPCODE_CONTINUATION) {
            os_printf("Received continuation\n");
        } else if (pclient->recv_opcode == WS_OPCODE_CLOSE) {
            os_printf("Received close, sending close frame back...\n");
            robotd_websocket_send(pclient, WS_OPCODE_CLOSE, pclient->recv_buf,
                pclient->recv_buf_data_length >= 2 ? 2 : 0);
        } else {
            os_printf("Opcode not recognized, something is wrong\n");
        }
    }
}

static void ICACHE_FLASH_ATTR
robotd_recv_cb(void *arg, char *pusrdata, unsigned short length)
{
    struct espconn *pespconn = arg;

    robotd_client *pclient = robotd_find_client(pespconn);

    if (pclient == NULL) {
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
            // TODO what if all the slots are filled?
            robotd_do_websocket_handshake(pespconn, gReq.data);
        } else {
            send_501(pespconn);
            os_printf("Sent 501\n", res_len);
        }
    } else if (pclient->type == CLIENT_WS) {
        robotd_handle_websocket_frame(pclient, pusrdata, length);
    }
}

static void ICACHE_FLASH_ATTR
robotd_discon_cb(void *arg)
{
    os_printf("discon_cb\n");
    robotd_delete_client((struct espconn *)arg);
}

static void ICACHE_FLASH_ATTR
robotd_recon_cb(void *arg, sint8 err)
{
    os_printf("recon_cb\n");
    robotd_delete_client((struct espconn *)arg);
}

static void ICACHE_FLASH_ATTR
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
    espconn_tcp_set_max_con_allow(&esp_conn, MAX_NUM_CLIENTS);
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

