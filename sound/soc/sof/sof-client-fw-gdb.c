// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright(c) 2022 Intel Corporation. All rights reserved.
//
// Author: Noah Klayman <noah.klayman@intel.com>
//

#include <linux/auxiliary_bus.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/ktime.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <sound/sof/header.h>
#include <sound/sof/ipc4/header.h>

#include "sof-client.h"
#include "sof-priv.h"
#include "ops.h"

#define SOF_IPC_CLIENT_SUSPEND_DELAY_MS	3000

struct sof_fw_gdb_priv {
	struct dentry *dfs_file;
	size_t max_msg_size;
	enum sof_ipc_type ipc_type;

	struct sof_ipc_gdb_message *tx_buffer;
	void *rx_buffer;
};

struct sof_ipc_gdb_message {
	struct sof_ipc_cmd_hdr hdr;
	char cmd[];
} __packed;

#define RING_SIZE 128
struct ring {
	unsigned char head;
	unsigned char fill1[63];
	unsigned char tail;
	unsigned char fill2[63];
	unsigned char data[RING_SIZE];
} __packed;

static int sof_fw_gdb_send_message(struct sof_client_dev *cdev)
{
	struct sof_fw_gdb_priv *priv = cdev->data;
	struct device *dev = &cdev->auxdev.dev;
	int ret, err;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0 && ret != -EACCES) {
		dev_err_ratelimited(dev, "debugfs write failed to resume %d\n", ret);
		return ret;
	}

	/* send the message */
	ret = sof_client_ipc_tx_message(cdev, priv->tx_buffer, priv->rx_buffer,
					priv->max_msg_size);
	if (ret)
		dev_err(dev, "IPC message send failed: %d\n", ret);

	pm_runtime_mark_last_busy(dev);
	err = pm_runtime_put_autosuspend(dev);
	if (err < 0)
		dev_err_ratelimited(dev, "debugfs write failed to idle %d\n", err);

	return ret;
}

static bool gdb_has_init = false;

static int sof_fw_gdb_dfs_open(struct inode *inode, struct file *file)
{
	struct sof_client_dev *cdev = inode->i_private;
	struct sof_fw_gdb_priv *priv = cdev->data;
	int ret;

	if (sof_client_get_fw_state(cdev) == SOF_FW_CRASHED)
		return -ENODEV;

	ret = debugfs_file_get(file->f_path.dentry);
	if (unlikely(ret))
		return ret;

	ret = simple_open(inode, file);
	if (ret)
		debugfs_file_put(file->f_path.dentry);

	// When file is opened, send GDB init command to FW
	priv->tx_buffer->hdr.cmd = SOF_IPC_GLB_GDB_DEBUG;
	priv->tx_buffer->hdr.size = sizeof(struct sof_ipc_cmd_hdr);
	if (!gdb_has_init) {
		sof_fw_gdb_send_message(cdev);
		gdb_has_init = true;
	}

	return ret;
}

int sof_debug_read(struct snd_sof_dev *sdev,
		       char *buff, size_t bytes)
{
	int count = 0;
	int bar = snd_sof_dsp_get_bar_index(sdev, SOF_FW_BLK_TYPE_SRAM);
	void __iomem *rx_ring_buffer = sdev->bar[bar] + sdev->debug_box.offset + 256;
	char* msg_region = rx_ring_buffer + 128;


	uint8_t head = readb(rx_ring_buffer);
	uint8_t tail = readb(rx_ring_buffer + 64);

	for (int i = 0; i < bytes; i++) {
		if (tail == head) {
			// No Data
			break;
		}
		buff[i] = readb(msg_region + tail);
		tail = (tail + 1) % RING_SIZE;
		count++;
	}

	writeb(tail, rx_ring_buffer + 64);

	return count;
}

static ssize_t sof_fw_gdb_dfs_read(struct file *file, char __user *buffer,
				       size_t count, loff_t *ppos)
{
	struct sof_client_dev *cdev = file->private_data;
	struct snd_sof_dev *sdev = sof_client_dev_to_sof_dev(cdev);

	char *kbuff = kzalloc(count, GFP_KERNEL);
	int count_read = sof_debug_read(sdev, kbuff, count);
	if (copy_to_user(buffer, kbuff, count_read))
		return -EFAULT;
	kfree(kbuff);

	// Since the DSP runs asynchronously to the kernel, we have to wait a
	// while for a response. If we return no data, gdb will exit. This keeps
	// it listening for future data.
	if (count_read == 0) {
		if (copy_to_user(buffer, "\0", 1)) {
			return -EFAULT;
		} else {
			return 1;
		}
	}
	return count_read;
}

// TODO, should be same as ipc3
static ssize_t sof_fw_gdb_ipc4_dfs_read(struct file *file,
					    char __user *buffer,
					    size_t count, loff_t *ppos)
{
	return 0;
}

int sof_debug_write(struct snd_sof_dev *sdev,
		       char *message, size_t bytes)
{
	int count = 0;

	int bar = snd_sof_dsp_get_bar_index(sdev, SOF_FW_BLK_TYPE_SRAM);
	void __iomem *tx_ring_buffer = sdev->bar[bar] + sdev->debug_box.offset;
	char* msg_region = tx_ring_buffer + 128;

	uint8_t head = readb(tx_ring_buffer);
	uint8_t tail = readb(tx_ring_buffer + 64);

	for (int i = 0; i < bytes; i++) {
		if((head + 1) % RING_SIZE == tail) {
			break;
		}
		writeb(message[i], msg_region + head);
		head = (head + 1) % RING_SIZE;
		count++;
	}

	writeb(head, tx_ring_buffer);

	return count;
}

static ssize_t sof_fw_gdb_dfs_write(struct file *file, const char __user *buffer,
					size_t count, loff_t *ppos)
{
	int num_failed, num_written;
	struct sof_client_dev *cdev = file->private_data;

	struct snd_sof_dev *sdev = sof_client_dev_to_sof_dev(cdev);
	char *kbuff = kzalloc(count + 1, GFP_KERNEL);
	num_failed = copy_from_user(kbuff, buffer, count);
	if (num_failed == count) {
		kfree(kbuff);
		return -EFAULT;
	}
        num_written = sof_debug_write(sdev, kbuff, count - num_failed);
	kfree(kbuff);
	return num_written;
};

// TODO, should be same as ipc3
static ssize_t sof_fw_gdb_ipc4_dfs_write(struct file *file,
					     const char __user *buffer,
					     size_t count, loff_t *ppos)
{
	return 0;
};

static int sof_fw_gdb_dfs_release(struct inode *inode, struct file *file)
{
	debugfs_file_put(file->f_path.dentry);

	return 0;
}

static const struct file_operations sof_fw_gdb_fops = {
	.open = sof_fw_gdb_dfs_open,
	.read = sof_fw_gdb_dfs_read,
	.write = sof_fw_gdb_dfs_write,
	.llseek = default_llseek,
	.release = sof_fw_gdb_dfs_release,

	.owner = THIS_MODULE,
};

static const struct file_operations sof_fw_gdb_ipc4_fops = {
	.open = sof_fw_gdb_dfs_open,
	.read = sof_fw_gdb_ipc4_dfs_read,
	.write = sof_fw_gdb_ipc4_dfs_write,
	.llseek = default_llseek,
	.release = sof_fw_gdb_dfs_release,

	.owner = THIS_MODULE,
};

static int sof_fw_gdb_probe(struct auxiliary_device *auxdev,
				const struct auxiliary_device_id *id)
{
	struct sof_client_dev *cdev = auxiliary_dev_to_sof_client_dev(auxdev);
	struct dentry *debugfs_root = sof_client_get_debugfs_root(cdev);
	static const struct file_operations *fops;
	struct device *dev = &auxdev->dev;
	struct sof_fw_gdb_priv *priv;
	size_t alloc_size;

	/* allocate memory for client data */
	priv = devm_kzalloc(&auxdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->ipc_type = sof_client_get_ipc_type(cdev);
	priv->max_msg_size = sof_client_get_ipc_max_payload_size(cdev);
	alloc_size = priv->max_msg_size;

	if (priv->ipc_type == SOF_INTEL_IPC4)
		alloc_size += sizeof(struct sof_ipc4_msg);

	priv->tx_buffer = devm_kmalloc(dev, alloc_size, GFP_KERNEL);
	priv->rx_buffer = devm_kzalloc(dev, alloc_size, GFP_KERNEL);
	if (!priv->tx_buffer || !priv->rx_buffer)
		return -ENOMEM;

	if (priv->ipc_type == SOF_INTEL_IPC4) {
		struct sof_ipc4_msg *ipc4_msg;

		ipc4_msg = (void*) priv->tx_buffer;
		ipc4_msg->data_ptr = priv->tx_buffer + sizeof(struct sof_ipc4_msg);

		ipc4_msg = priv->rx_buffer;
		ipc4_msg->data_ptr = priv->rx_buffer + sizeof(struct sof_ipc4_msg);

		fops = &sof_fw_gdb_ipc4_fops;
	} else {
		fops = &sof_fw_gdb_fops;
	}

	cdev->data = priv;

	priv->dfs_file = debugfs_create_file("fw_gdb", 0644, debugfs_root,
					     cdev, fops);

	/* enable runtime PM */
	pm_runtime_set_autosuspend_delay(dev, SOF_IPC_CLIENT_SUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_idle(dev);

	return 0;
}

static void sof_fw_gdb_remove(struct auxiliary_device *auxdev)
{
	struct sof_client_dev *cdev = auxiliary_dev_to_sof_client_dev(auxdev);
	struct sof_fw_gdb_priv *priv = cdev->data;

	pm_runtime_disable(&auxdev->dev);

	debugfs_remove(priv->dfs_file);
}

static const struct auxiliary_device_id sof_fw_gdb_client_id_table[] = {
	{ .name = "snd_sof.fw_gdb" },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, sof_fw_gdb_client_id_table);

/*
 * No need for driver pm_ops as the generic pm callbacks in the auxiliary bus
 * type are enough to ensure that the parent SOF device resumes to bring the DSP
 * back to D0.
 * Driver name will be set based on KBUILD_MODNAME.
 */
static struct auxiliary_driver sof_gdb_fw_client_drv = {
	.probe = sof_fw_gdb_probe,
	.remove = sof_fw_gdb_remove,

	.id_table = sof_fw_gdb_client_id_table,
};

module_auxiliary_driver(sof_gdb_fw_client_drv);

MODULE_DESCRIPTION("SOF IPC FW GDB Client Driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(SND_SOC_SOF_CLIENT);
