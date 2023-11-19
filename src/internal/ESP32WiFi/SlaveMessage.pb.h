/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.8-dev */

#ifndef PB_SLAVEMESSAGE_PB_H_INCLUDED
#define PB_SLAVEMESSAGE_PB_H_INCLUDED
#ifdef PLATFORMIO
#include <pb.h>
#else
#include "nanopb/pb.h"
#endif

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _SlaveMessage {
    pb_callback_t type;
    pb_callback_t data;
    uint32_t seq;
    pb_callback_t id;
    pb_callback_t mac;
    int32_t rssi;
    uint32_t free_heap;
    uint32_t loop_duration;
    uint32_t cpu_freq;
    uint32_t flash_size;
} SlaveMessage;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define SlaveMessage_init_default                {{{NULL}, NULL}, {{NULL}, NULL}, 0, {{NULL}, NULL}, {{NULL}, NULL}, 0, 0, 0, 0, 0}
#define SlaveMessage_init_zero                   {{{NULL}, NULL}, {{NULL}, NULL}, 0, {{NULL}, NULL}, {{NULL}, NULL}, 0, 0, 0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define SlaveMessage_type_tag                    1
#define SlaveMessage_data_tag                    2
#define SlaveMessage_seq_tag                     3
#define SlaveMessage_id_tag                      4
#define SlaveMessage_mac_tag                     5
#define SlaveMessage_rssi_tag                    6
#define SlaveMessage_free_heap_tag               7
#define SlaveMessage_loop_duration_tag           8
#define SlaveMessage_cpu_freq_tag                9
#define SlaveMessage_flash_size_tag              10

/* Struct field encoding specification for nanopb */
#define SlaveMessage_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, STRING,   type,              1) \
X(a, CALLBACK, SINGULAR, BYTES,    data,              2) \
X(a, STATIC,   SINGULAR, UINT32,   seq,               3) \
X(a, CALLBACK, SINGULAR, STRING,   id,                4) \
X(a, CALLBACK, SINGULAR, STRING,   mac,               5) \
X(a, STATIC,   SINGULAR, INT32,    rssi,              6) \
X(a, STATIC,   SINGULAR, UINT32,   free_heap,         7) \
X(a, STATIC,   SINGULAR, UINT32,   loop_duration,     8) \
X(a, STATIC,   SINGULAR, UINT32,   cpu_freq,          9) \
X(a, STATIC,   SINGULAR, UINT32,   flash_size,       10)
#define SlaveMessage_CALLBACK pb_default_field_callback
#define SlaveMessage_DEFAULT NULL

extern const pb_msgdesc_t SlaveMessage_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define SlaveMessage_fields &SlaveMessage_msg

/* Maximum encoded size of messages (where known) */
/* SlaveMessage_size depends on runtime parameters */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
