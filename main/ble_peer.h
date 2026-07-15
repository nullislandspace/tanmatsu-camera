#pragma once

#include <stdint.h>
#include <sys/queue.h>
#include "host/ble_gatt.h"

#define PEER_ADDR_VAL_SIZE 6

struct peer_dsc {
    SLIST_ENTRY(peer_dsc) next;
    struct ble_gatt_dsc dsc;
};
SLIST_HEAD(peer_dsc_list, peer_dsc);

struct peer_chr {
    SLIST_ENTRY(peer_chr) next;
    struct ble_gatt_chr chr;

    struct peer_dsc_list dscs;
};
SLIST_HEAD(peer_chr_list, peer_chr);
SLIST_HEAD(peer_svc_list, peer_svc);

#if MYNEWT_VAL(BLE_INCL_SVC_DISCOVERY) || MYNEWT_VAL(BLE_GATT_CACHING_INCLUDE_SERVICES)
struct peer_incl_svc {
    SLIST_ENTRY(peer_incl_svc) next;
    struct ble_gatt_incl_svc svc;
};
SLIST_HEAD(peer_incl_svc_list, peer_incl_svc);
#endif

struct peer_svc {
    SLIST_ENTRY(peer_svc) next;
    struct ble_gatt_svc svc;
#if MYNEWT_VAL(BLE_INCL_SVC_DISCOVERY) || MYNEWT_VAL(BLE_GATT_CACHING_INCLUDE_SERVICES)
    struct peer_incl_svc_list incl_svc;
#endif
    struct peer_chr_list chrs;
};

struct peer;
typedef void peer_disc_fn(const struct peer* peer, int status, void* arg);

/**
 * @brief The callback function for the devices traversal.
 *
 * @param peer
 * @param arg
 * @return int  0, continue; Others, stop the traversal.
 *
 */
typedef int peer_traverse_fn(const struct peer* peer, void* arg);

struct peer {
    SLIST_ENTRY(peer) next;
    uint16_t             conn_handle;
    uint8_t              peer_addr[PEER_ADDR_VAL_SIZE];
    struct peer_svc_list svcs;  // List of discovered GATT services

    // Keeps track of where we are in the service discovery process
    uint16_t         disc_prev_chr_val;
    struct peer_svc* cur_svc;

    // Callback that gets executed when service discovery completes
    peer_disc_fn* disc_cb;
    void*         disc_cb_arg;

    // Connection information
    uint16_t mtu;
};

void peer_traverse_all(peer_traverse_fn* trav_cb, void* arg);

int peer_disc_svc_by_uuid(uint16_t conn_handle, const ble_uuid_t* uuid, peer_disc_fn* disc_cb, void* disc_cb_arg);

int                    peer_disc_all(uint16_t conn_handle, peer_disc_fn* disc_cb, void* disc_cb_arg);
const struct peer_dsc* peer_dsc_find_uuid(const struct peer* peer, const ble_uuid_t* svc_uuid,
                                          const ble_uuid_t* chr_uuid, const ble_uuid_t* dsc_uuid);
const struct peer_chr* peer_chr_find_uuid(const struct peer* peer, const ble_uuid_t* svc_uuid,
                                          const ble_uuid_t* chr_uuid);
const struct peer_svc* peer_svc_find_uuid(const struct peer* peer, const ble_uuid_t* uuid);
int                    peer_set_mtu(uint16_t conn_handle, uint16_t mtu);
int                    peer_get_mtu(uint16_t conn_handle, uint16_t* mtu);
int                    peer_delete(uint16_t conn_handle);
int                    peer_add(uint16_t conn_handle);
#if MYNEWT_VAL(BLE_INCL_SVC_DISCOVERY) || MYNEWT_VAL(BLE_GATT_CACHING_INCLUDE_SERVICES)
int peer_init(int max_peers, int max_svcs, int max_incl_svcs, int max_chrs, int max_dscs);
#else
int peer_init(int max_peers, int max_svcs, int max_chrs, int max_dscs);
#endif
struct peer* peer_find(uint16_t conn_handle);
#if MYNEWT_VAL(ENC_ADV_DATA)
int peer_set_addr(uint16_t conn_handle, uint8_t* peer_addr);
#endif
void peer_list_all(const struct peer* peer);
