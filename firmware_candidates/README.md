# 候选固件修订

本目录不代表当前设备已经烧录的内容，也不作为默认烧录入口。

`measurement_main_MEASUREMENT_SYNC_CAL_V3_candidate.py` 针对 CCD 超时后的迟到帧和采集互斥做候选修订；`rotation_main_ROTATION_SYNC_MP_V2_candidate.py` 针对启动屏蔽状态做候选修订。它们需要在真实设备上单独编译、上传和回放验证后，才可以替换基准文件。
