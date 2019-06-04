#!/bin/bash
FILE="vmlinuz-initrd-7278b1"
TAR_IMAGE="vmlinuz-initrd_vuduo4k_$(date +%Y%m%d).tar"
source toolchain_helper.sh
cd rootfs
if [ ! -e images ]; then
	mkdir images
fi
make images -j8
cd images
if [ -e $FILE ]; then
	tar -cvf $TAR_IMAGE $FILE
	gzip $TAR_IMAGE
fi
