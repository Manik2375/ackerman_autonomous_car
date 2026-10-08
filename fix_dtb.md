# PWM pins not working and `jetson-io.py` can’t be accessed

The problem comes with the **DTB** file not being loaded into `/boot/extlinux/extlinux.conf`.

**DTB** stands for **Device Tree Blob**.

Think of it as a **hardware configuration file** that Linux reads during boot. Unlike a PC, an ARM board like the Jetson Nano can't automatically discover all of its hardware. The DTB tells the kernel things like:

- Which GPIO pins exist
- Which pins can be PWM, SPI, I²C, UART
- Which I²C buses exist
- Which cameras are connected
- Which PWM controllers are enabled

## Fixing the DTB / `jetson-io.py` issue

To check whether the DTB file exists, run:

```bash
ls /boot/dtb
```

If it exists, open `extlinux.conf`:

```bash
sudo vim /boot/extlinux/extlinux.conf
```

Add the following line:

```text
FDT /boot/dtb/kernel_tegra210-p3448-0000-p3449-0000-b00.dtb
```

> **Note:** Make sure the DTB filename is correct for your Jetson Nano.

In the end, your configuration should look something like this:

```text
LABEL primary
    MENU LABEL primary kernel
    LINUX /boot/Image
    FDT /boot/dtb/kernel_tegra210-p3448-0000-p3449-0000-b00.dtb
    INITRD /boot/initrd
    APPEND ${cbootargs} ...
```

That's it!

Save the file, reboot the Jetson Nano, and try accessing `jetson-io.py` again.