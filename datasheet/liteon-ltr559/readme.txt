【error: negative width in bit-field '<anonymous>'】

分析：
在Linux-4.14.78内核中对权限管理很严，这里是DEVICE_ATTR宏的权限问题，将上述的0666改为0660，问题迎刃而解。

解决方法：
static DEVICE_ATTR(als_adc, 0666, als_adc_show, NULL);
改成：
static DEVICE_ATTR(als_adc, 0660, als_adc_show, NULL);
或者
static DEVICE_ATTR(als_adc, S_IWUSR | S_IRUGO, als_adc_show, NULL);





【WARNING: CPU: 0 PID: 1 at  kernel_imx/drivers/base/core.c:1301 device_create_file+0x88/0xa4】
/**
 * device_create_file - create sysfs attribute file for device.
 * @dev: device.
 * @attr: device attribute descriptor.
 */
int device_create_file(struct device *dev,
                       const struct device_attribute *attr)
{
        int error = 0;

        if (dev) {
                WARN(((attr->attr.mode & S_IWUGO) && !attr->store),
                        "Attribute %s: write permission without 'store'\n",
                        attr->attr.name);
                WARN(((attr->attr.mode & S_IRUGO) && !attr->show),
                        "Attribute %s: read permission without 'show'\n",
                        attr->attr.name);
                error = sysfs_create_file(&dev->kobj, &attr->attr);
        }

        return error;
}
EXPORT_SYMBOL_GPL(device_create_file);




【/sys/devices/platform/30a40000.i2c/i2c-1/1-0023】
