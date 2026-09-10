# 当前设备固件基准

本目录保存用户提供的已烧录程序副本，用于核对协议和测量条件。

- `rotation_main_ROTATION_SYNC_MP_V2.py`：旋转端 `ROTATION_SYNC_MP_V2`，提供 `SYNC`、`ROT` 和 `TRIG`。
- `measurement_main_CCD_PEAK_RAW_CAL_3.py`：当前测距端 `CCD-PEAK-RAW-CAL-3.0`，使用 `@c0071#@` 和曝光档位 5，只提供静态标定及 `MIN/SAMPLE` 原始坐标，不提供 `SYNC/START/PIX`。

`measurement_main_MEASUREMENT_SYNC_CAL_V3.py` 是上一版已烧录同步测距程序的留档，曝光档位为 3，不能直接套用当前曝光 5 的标定表。它不再作为当前设备状态。

`manifest.json` 中 `measurement.source_sha256` 是用户原文件的 SHA-256；仓库副本统一了换行符并补齐文件末尾换行，文件校验值记录在 `measurement.sha256`。

正式旋转建图需要使用 `firmware_candidates/` 中与曝光 5 一致的同步候选，并完成烧录与实机验证。候选修订不覆盖本目录中的已烧录基准。
