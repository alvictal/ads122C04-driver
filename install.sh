#!/bin/sh
# Script to install ads122c04 driver to linux system using a compiled kernel object

echo "Installing ads122c045 driver ..."

KVER=$(uname -r)
MODDESTDIR="/lib/modules/$KVER/kernel/drivers/iio/adc/"

grep ads122c04.ko /lib/modules/$KVER/modules.dep && rm -fr $MODDESTDIR/ti-ads122c04.ko || echo "No system ti-ads122c04.ko file found. Install the new ads122c04.ko driver into the system"
install -p -m 644 ti-ads122c04.ko $MODDESTDIR
# analyzes your kernel modules (in the directory /lib/modules/$KVER) and creates/updates a list of dependencies (named modules.dep)
depmod -v -A
modprobe ads122c04

echo "done."
exit 0
