#ifndef ZMQ_PUBLISHER_H
#define ZMQ_PUBLISHER_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initializes ZMQ PUB socket and binds to endpoint.
 * endpoint: e.g. "tcp://*:5558"
 * Returns 0 on success, -1 on failure.
 */
int zmq_pub_init(const char *endpoint);

/**
 * Publishes a depth frame.
 * Sends two-part ZMQ message:
 *   Part 1 (topic+header): "depth_frame" + [ts_ns:u64][width:u16][height:u16]
 *   Part 2 (data):         depth(i16 x w*h) then color(u8 x w*h*3)
 */
int zmq_pub_depth_frame(uint64_t timestamp_ns,
                        uint16_t width, uint16_t height,
                        const int16_t  *depth,  /* w*h samples */
                        const uint8_t  *color); /* w*h*3 bytes  */

/**
 * Closes and cleans up the ZMQ publisher.
 */
void zmq_pub_destroy(void);

#ifdef __cplusplus
}
#endif

#endif /* ZMQ_PUBLISHER_H */
