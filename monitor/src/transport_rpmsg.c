/*
 * rpmsg-tty endpoint: Linux turns the name service announcement into
 * /dev/ttyRPMSG0. Setup follows samples/subsys/ipc/openamp_rsc_table.
 */
#include <zephyr/device.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <addr_translation.h>
#include <metal/device.h>
#include <metal/io.h>
#include <metal/sys.h>
#include <openamp/open_amp.h>
#include <resource_table.h>

#include "monitor/monitor.h"
#include "transport_rpmsg.h"

LOG_MODULE_REGISTER(transport);

#define SHM_NODE DT_CHOSEN(zephyr_ipc_shm)
#define SHM_START_ADDR DT_REG_ADDR(SHM_NODE)
#define SHM_SIZE DT_REG_SIZE(SHM_NODE)

#define RPMSG_STACK_SIZE 1536
#define RPMSG_PRIORITY K_PRIO_PREEMPT(1)

static K_THREAD_STACK_DEFINE(rpmsg_stack, RPMSG_STACK_SIZE);
static struct k_thread rpmsg_thread;
static K_SEM_DEFINE(doorbell, 0, 1);

static const struct device *const ipm = DEVICE_DT_GET(DT_CHOSEN(zephyr_ipc));

static metal_phys_addr_t shm_physmap = SHM_START_ADDR;
static metal_phys_addr_t rsc_physmap;
static struct metal_io_region shm_io;
static struct metal_io_region rsc_io;
static struct rpmsg_virtio_device rvdev;
static struct rpmsg_endpoint tty_ept;
static void *rsc_table;

static void ipm_callback(const struct device *dev, void *context, uint32_t id,
			 volatile void *data)
{
	k_sem_give(&doorbell);
}

static int mailbox_notify(void *priv, uint32_t id)
{
	ipm_send(ipm, 0, id, &id, 4);
	return 0;
}

static int tty_rx(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv)
{
	monitor_rx_bytes(data, len);
	return RPMSG_SUCCESS;
}

static int platform_init(void)
{
	struct metal_init_params params = METAL_INIT_DEFAULTS;
	int rsc_size;

	if (metal_init(&params)) {
		return -EIO;
	}
	metal_io_init(&shm_io, (void *)SHM_START_ADDR, &shm_physmap, SHM_SIZE, -1, 0,
		      addr_translation_get_ops(shm_physmap));

	rsc_table_get(&rsc_table, &rsc_size);
	rsc_physmap = (uintptr_t)rsc_table;
	metal_io_init(&rsc_io, rsc_table, &rsc_physmap, rsc_size, -1, 0, NULL);

	if (!device_is_ready(ipm)) {
		return -ENODEV;
	}
	ipm_register_callback(ipm, ipm_callback, NULL);
	return ipm_set_enabled(ipm, 1);
}

static int create_vdev(void)
{
	struct virtio_device *vdev;
	struct fw_rsc_vdev_vring *vring;

	vdev = rproc_virtio_create_vdev(VIRTIO_DEV_DEVICE, VDEV_ID, rsc_table_to_vdev(rsc_table),
					&rsc_io, NULL, mailbox_notify, NULL);
	if (vdev == NULL) {
		return -ENOMEM;
	}
	rproc_virtio_wait_remote_ready(vdev);

	vring = rsc_table_get_vring0(rsc_table);
	if (rproc_virtio_init_vring(vdev, 0, vring->notifyid, (void *)vring->da, &rsc_io,
				    vring->num, vring->align)) {
		return -EIO;
	}
	vring = rsc_table_get_vring1(rsc_table);
	if (rproc_virtio_init_vring(vdev, 1, vring->notifyid, (void *)vring->da, &rsc_io,
				    vring->num, vring->align)) {
		return -EIO;
	}
	return rpmsg_init_vdev(&rvdev, vdev, NULL, &shm_io, NULL);
}

static void rpmsg_main(void *a, void *b, void *c)
{
	int ret = platform_init();

	ret = ret ? ret : create_vdev();
	ret = ret ? ret
		  : rpmsg_create_ept(&tty_ept, rpmsg_virtio_get_rpmsg_device(&rvdev), "rpmsg-tty",
				     RPMSG_ADDR_ANY, RPMSG_ADDR_ANY, tty_rx, NULL);
	if (ret) {
		/* The periodic task keeps running; with no input the monitor stays idle. */
		LOG_ERR("rpmsg setup failed: %d", ret);
		return;
	}
	LOG_INF("rpmsg-tty up");

	for (;;) {
		k_sem_take(&doorbell, K_FOREVER);
		rproc_virtio_notified(rvdev.vdev, VRING1_ID);
	}
}

int transport_rpmsg_start(void)
{
	k_thread_create(&rpmsg_thread, rpmsg_stack, RPMSG_STACK_SIZE, rpmsg_main, NULL, NULL, NULL,
			RPMSG_PRIORITY, 0, K_NO_WAIT);
	return 0;
}

void transport_rpmsg_send(const uint8_t *buf, size_t len)
{
	/*
	 * trysend, never send: the blocking call waits for a free vring buffer
	 * and would stall rule evaluation behind a Linux that stopped reading.
	 * Fails until Linux has written once, because only then is the
	 * destination address known. Telemetry does that well before takeoff.
	 */
	(void)rpmsg_trysend(&tty_ept, buf, (int)len);
}
