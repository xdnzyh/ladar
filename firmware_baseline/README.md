# 当前设备固件基准

这两个文件是用户提供的已烧录版本原样副本，仅作为协议、版本和验证对象基准：

- `rotation_main_ROTATION_SYNC_MP_V2.py`：旋转端 MicroPython，版本 `ROTATION_SYNC_MP_V2`。
- `measurement_main_MEASUREMENT_SYNC_CAL_V3.py`：测距端 MicroPython，版本 `MEASUREMENT_SYNC_CAL_V3`，CCD 实际曝光固定为 3，中心坐标上限为 1500。

候选修订不覆盖本目录。当前 PC 同步入口以这两个版本的 `SYNC`、`START`、`ROT`、`PIX`、`TRIG`、`PING`、`STOP`、`LASER` 和 `OFF` 行为为依据。
