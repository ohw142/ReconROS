#!/bin/bash
cat /opt/reconos/bitstream_full_static.bit | tee /dev/xdevcfg
insmod /opt/reconos/mreconos.ko
