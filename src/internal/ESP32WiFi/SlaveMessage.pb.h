/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.8-dev */

#ifndef PB_SLAVEMESSAGE_PB_H_INCLUDED
#define PB_SLAVEMESSAGE_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _SlaveDetails {
    pb_callback_t id;
    pb_callback_t mac;
    int32_t rssi;
    uint32_t free_heap;
    uint32_t loop_duration;
    uint32_t cpu_freq;
    uint32_t flash_size;
} SlaveDetails;

typedef struct _SlaveMessage {
    bool has_slave;
    SlaveDetails slave;
    pb_callback_t type;
    pb_callback_t data;
    uint32_t seq;
} SlaveMessage;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define SlaveMessage_init_default                {false, SlaveDetails_init_default, {{NULL}, NULL}, {{NULL}, NULL}, 0}
#define SlaveDetails_init_default                {{{NULL}, NULL}, {{NULL}, NULL}, 0, 0, 0, 0, 0}
#define SlaveMessage_init_zero                   {false, SlaveDetails_init_zero, {{NULL}, NULL}, {{NULL}, NULL}, 0}
#define SlaveDetails_init_zero                   {{{NULL}, NULL}, {{NULL}, NULL}, 0, 0, 0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define SlaveDetails_id_tag                      1
#define SlaveDetails_mac_tag                     2
#define SlaveDetails_rssi_tag                    3
#define SlaveDetails_free_heap_tag               4
#define SlaveDetails_loop_duration_tag           5
#define SlaveDetails_cpu_freq_tag                6
#define SlaveDetails_flash_size_tag              7
#define SlaveMessage_slave_tag                   1
#define SlaveMessage_type_tag                    2
#define SlaveMessage_data_tag                    3
#define SlaveMessage_seq_tag                     4

/* Struct field encoding specification for nanopb */
#define SlaveMessage_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  slave,             1) \
X(a, CALLBACK, SINGULAR, STRING,   type,              2) \
X(a, CALLBACK, SINGULAR, BYTES,    data,              3) \
X(a, STATIC,   SINGULAR, UINT32,   seq,               4)
#define SlaveMessage_CALLBACK pb_default_field_callback
#define SlaveMessage_DEFAULT NULL
#define SlaveMessage_slave_MSGTYPE SlaveDetails

#define SlaveDetails_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, STRING,   id,                1) \
X(a, CALLBACK, SINGULAR, STRING,   mac,               2) \
X(a, STATIC,   SINGULAR, INT32,    rssi,              3) \
X(a, STATIC,   SINGULAR, UINT32,   free_heap,         4) \
X(a, STATIC,   SINGULAR, UINT32,   loop_duration,     5) \
X(a, STATIC,   SINGULAR, UINT32,   cpu_freq,          6) \
X(a, STATIC,   SINGULAR, UINT32,   flash_size,        7)
#define SlaveDetails_CALLBACK pb_default_field_callback
#define SlaveDetails_DEFAULT NULL

extern const pb_msgdesc_t SlaveMessage_msg;
extern const pb_msgdesc_t SlaveDetails_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define SlaveMessage_fields &SlaveMessage_msg
#define SlaveDetails_fields &SlaveDetails_msg

/* Maximum encoded size of messages (where known) */
/* SlaveMessage_size depends on runtime parameters */
/* SlaveDetails_size depends on runtime parameters */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
