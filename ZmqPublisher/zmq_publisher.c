#include "zmq_publisher.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zmq.h>

static void *g_ctx    = NULL;
static void *g_socket = NULL;

int zmq_pub_init(const char *endpoint)
{
    g_ctx = zmq_ctx_new();
    if (!g_ctx) {
        fprintf(stderr, "[ZmqPub] zmq_ctx_new failed\n");
        return -1;
    }

    g_socket = zmq_socket(g_ctx, ZMQ_PUB);
    if (!g_socket) {
        fprintf(stderr, "[ZmqPub] zmq_socket failed\n");
        zmq_ctx_destroy(g_ctx);
        g_ctx = NULL;
        return -1;
    }

    int sndhwm = 4; // drop old frames rather than block
    zmq_setsockopt(g_socket, ZMQ_SNDHWM, &sndhwm, sizeof(sndhwm));

    if (zmq_bind(g_socket, endpoint) != 0) {
        fprintf(stderr, "[ZmqPub] zmq_bind(%s) failed: %s\n", endpoint, zmq_strerror(zmq_errno()));
        zmq_close(g_socket);
        zmq_ctx_destroy(g_ctx);
        g_socket = NULL;
        g_ctx    = NULL;
        return -1;
    }

    printf("[ZmqPub] bound to %s\n", endpoint);
    return 0;
}

int zmq_pub_depth_frame(uint64_t timestamp_ns,
                        uint16_t width, uint16_t height,
                        const int16_t  *depth,
                        const uint8_t  *color)
{
    if (!g_socket) return -1;

    /* Part 1: topic */
    static const char TOPIC[] = "depth_frame";
    const size_t topic_len = sizeof(TOPIC) - 1; /* no null */

    /* Part 2: header [ts_ns:u64][width:u16][height:u16] = 12 bytes */
    uint8_t hdr_buf[12];
    memcpy(hdr_buf,      &timestamp_ns, 8);
    memcpy(hdr_buf + 8,  &width,        2);
    memcpy(hdr_buf + 10, &height,       2);

    /* Part 3: depth (i16 x w*h) + color (u8 x w*h*3) */
    size_t depth_bytes = (size_t)width * height * 2;
    size_t color_bytes = (size_t)width * height * 3;
    size_t data_len    = depth_bytes + color_bytes;
    uint8_t *data = (uint8_t *)malloc(data_len);
    if (!data) return -1;
    memcpy(data,               depth, depth_bytes);
    memcpy(data + depth_bytes, color, color_bytes);

    zmq_send(g_socket, TOPIC,   topic_len,      ZMQ_SNDMORE | ZMQ_DONTWAIT);
    zmq_send(g_socket, hdr_buf, sizeof(hdr_buf),ZMQ_SNDMORE | ZMQ_DONTWAIT);
    zmq_send(g_socket, data,    data_len,        ZMQ_DONTWAIT);

    free(data);
    return 0;
}

void zmq_pub_destroy(void)
{
    if (g_socket) { zmq_close(g_socket);    g_socket = NULL; }
    if (g_ctx)    { zmq_ctx_destroy(g_ctx); g_ctx    = NULL; }
}
