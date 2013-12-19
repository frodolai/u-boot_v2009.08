u-boot_v2009.08
===============

u-boot v2009.08 for AR6MX series

https://github.com/frodolai/-boot_v2009.08/tree/rel_imx_3.0.35_4.0.0-bcm

How to build:
Configurations
--------------
Linux:
make mx6q_ar6mx_config
make mx6solo_ar6mx_config

Android:
make mx6q_ar6mx_android_config
make mx6solo_ar6mx_android_config

Mfgtool:
make mx6q_ar6mx_android_config
make mx6solo_ar6mx_mfg_config

then
	make -j4
get u-boot.bin

Flash to SD card
	sudo dd if=u-boot.bin of=/dev/sdx bs=512 seek=2 skip=2
