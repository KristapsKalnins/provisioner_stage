#ifndef BT_MESH_PROVISIONER_STAGE_H__
#define BT_MESH_PROVISIONER_STAGE_H__

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/mesh.h>
#include "prov_helper_srv.h"

void provisioner_unprovisioned_beacon_callback(uint8_t uuid[16],
				 bt_mesh_prov_oob_info_t oob_info,
				 uint32_t *uri_hash);

void provisioner_node_added_callback(uint16_t idx, uint8_t uuid[16], uint16_t addr, uint8_t num_elem);

void provisioner_stage_init(struct bt_mesh_model* model);
int provisioner_configure_cdb_with_app_key(struct bt_mesh_prov_helper_srv* srv, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf);
int provisioner_create_cdb_with_net_key(struct bt_mesh_prov_helper_srv* srv, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf);
int provisioner_process_provisioning_info(struct bt_mesh_prov_helper_srv* srv, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf);
int provisioner_send_node_to_base_provisioner(struct bt_mesh_model* model, uint16_t node_addr, uint16_t destination_node);

#endif /* BT_MESH_PROVISIONER_STAGE_H__ */