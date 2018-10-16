#!/bin/sh

patch -R -p2 < patch-R_CoreSrcSai.diff
patch -R -p2 < patch-R_CoreIncStm32l4xx_hal_conf.diff
patch -R -p2 < patch-R_DriversSrcStmSrcHalRcc.diff
patch -R -p2 < patch-R_CoreSrcTim.diff

