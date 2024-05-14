
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/bluetooth/mesh/access.h>
#include "prov_helper_cli.h"
#include "prov_helper_srv.h"

#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(provisioner_stage);

static uint8_t node_uuid[16];
static uint16_t node_addr;
static uint8_t primary_app_key[16] = {0};
static uint8_t primary_net_key[16] = {0};
static struct bt_mesh_model* prov_helper_cli_model;
static uint16_t initial_provisioner_address = 0x0;
static uint16_t provisioning_address_range_start = 0x0;
static uint16_t provisioning_address_range_end = 0x0;



K_SEM_DEFINE(sem_unprov_beacon, 0, 1);
K_SEM_DEFINE(sem_node_added, 0, 1);
K_SEM_DEFINE(sem_provisioner_app_key_received, 0, 1);
K_SEM_DEFINE(sem_provisioner_net_key_received, 0, 1);
K_SEM_DEFINE(sem_provisioner_addr_info_received, 0, 1);

void provisioner_unprovisioned_beacon_callback(uint8_t uuid[16],
				 bt_mesh_prov_oob_info_t oob_info,
				 uint32_t *uri_hash)
{
	memcpy(node_uuid, uuid, 16);
	k_sem_give(&sem_unprov_beacon);
}

void provisioner_node_added_callback(uint16_t idx, uint8_t uuid[16], uint16_t addr, uint8_t num_elem)
{
	node_addr = addr;
	k_sem_give(&sem_node_added);
}

void provisioner_create_cdb_with_net_key(struct bt_mesh_prov_helper_srv* srv, struct bt_mesh_msg_ctx *ctx,
		struct net_buf_simple *buf){

	memcpy(primary_net_key, buf->data, 16);

	int err = bt_mesh_cdb_create(buf->data);
	LOG_INF("CDB created with code %d",err);

	k_sem_give(&sem_provisioner_net_key_received);

	return;
}

void provisioner_configure_cdb_with_app_key(struct bt_mesh_prov_helper_srv* srv, struct bt_mesh_msg_ctx *ctx,
		struct net_buf_simple *buf){

	struct bt_mesh_cdb_app_key *key;
	int err;
	
	memcpy(primary_app_key, buf->data, 16);

	// Maybe the net and app id could also be transfered from the previous
	// provisioner
	key = bt_mesh_cdb_app_key_alloc(BT_MESH_NET_PRIMARY, 0);
	if (key == NULL) {
		printk("Failed to allocate app-key 0x%04x\n", 0);
		return;
	}

	err = bt_mesh_cdb_app_key_import(key, 0, buf->data);
	if (err) {
		printk("Failed to import appkey into cdb. Err:%d\n", err);
		return;
	}

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		bt_mesh_cdb_app_key_store(key);
	}

	k_sem_give(&sem_provisioner_app_key_received);

	return;
		
}

void provisioner_process_provisioning_address_range(struct bt_mesh_prov_helper_srv* srv, struct bt_mesh_msg_ctx *ctx,
		struct net_buf_simple *buf){
	
	provisioning_address_range_start = net_buf_simple_pull_le16(buf);
	provisioning_address_range_end = net_buf_simple_pull_le16(buf);
	initial_provisioner_address = net_buf_simple_pull_le16(buf);

	LOG_INF("Received range 0x%04X -> 0x%04X with 0x%04X origin",provisioning_address_range_start,
																 provisioning_address_range_end,
																 initial_provisioner_address);

	k_sem_give(&sem_provisioner_addr_info_received);

	return;
}

void provisioner_stage_init(struct bt_mesh_model* model){
	prov_helper_cli_model = model;
	return;
}

int provisioner_search_for_unprovisioned_devices(){
    // This semaphore is posted after this node
	// receives the app key and net key from the
	// previous node
	k_sem_take(&sem_provisioner_app_key_received, K_FOREVER);
	k_sem_take(&sem_provisioner_net_key_received, K_FOREVER);
	k_sem_take(&sem_provisioner_addr_info_received, K_FOREVER);
	printk("Starting provisioner stage");

	int addr_idx = 0;
	static uint16_t provisioned_node_addrs[16];

	// Fix this part - the address is wrong
	uint16_t current_node_address = provisioning_address_range_start;

	int64_t start_time_s = k_uptime_get()/1000;
	while(((k_uptime_get()/1000) - start_time_s) < 60){
		k_sem_reset(&sem_unprov_beacon);
		k_sem_reset(&sem_node_added);
		//bt_mesh_cdb_node_foreach(provisioner_check_unconfigured, NULL);
	
		printk("Waiting for unprovisioned beacon...\n");
		int err = k_sem_take(&sem_unprov_beacon, K_SECONDS(10));
		if (err == -EAGAIN) {
			continue;
		}

		char uuid_hex_str[33];

		bin2hex(node_uuid, 16, uuid_hex_str, sizeof(uuid_hex_str));

		printk("Provisioning %s\n", uuid_hex_str);
		// Next one has to be self + element count
		err = bt_mesh_provision_adv(node_uuid, 0, current_node_address, 0);
		if (err < 0) {
			printk("Provisioning failed (err %d)\n", err);
			continue;
		}

		printk("Waiting for node to be added...\n");
		err = k_sem_take(&sem_node_added, K_SECONDS(10));
		if (err == -EAGAIN) {
			printk("Timeout waiting for node to be added\n");
			continue;
		}

		printk("Added node 0x%04x\n", node_addr);
		
		provisioned_node_addrs[addr_idx] = node_addr;

		struct bt_mesh_cdb_node* newnode = bt_mesh_cdb_node_get(node_addr);

		printk("New node has %d elements\n", newnode->num_elem);

		current_node_address += newnode->num_elem;

		bt_mesh_prov_helper_cli_send_nodeinfo(prov_helper_cli_model, newnode, initial_provisioner_address);
		bt_mesh_prov_helper_cli_send_appkey(prov_helper_cli_model, primary_app_key, newnode->addr);
		bt_mesh_prov_helper_cli_send_netkey(prov_helper_cli_model, primary_net_key, newnode->addr);
		addr_idx++;
		if (current_node_address == provisioning_address_range_end){break;}
	}

	uint16_t remaining_address_count = provisioning_address_range_end - current_node_address;	

	if (!remaining_address_count){
		return 0;
	}
	
	uint16_t addresses_per_next_provisioner = remaining_address_count / addr_idx;
	
	if(addresses_per_next_provisioner == 0){
		addresses_per_next_provisioner = 1;
	} 

	for(int i = 0; i < addr_idx; i++){
		
		uint16_t start_addr = current_node_address + (addresses_per_next_provisioner * i);
		uint16_t end_addr = start_addr + addresses_per_next_provisioner;

		remaining_address_count -= addresses_per_next_provisioner;

		LOG_INF("Sending 0x%04X -> 0x%04X with 0x%04X origin to 0x%04X", start_addr, end_addr,
																		  initial_provisioner_address,
																		  provisioned_node_addrs[i]);
		
		bt_mesh_prov_helper_cli_send_addrinfo(prov_helper_cli_model, start_addr,end_addr, initial_provisioner_address, provisioned_node_addrs[i]);
	
		if(remaining_address_count <= 0){
			break;
		}
	} 
	return 0;

}