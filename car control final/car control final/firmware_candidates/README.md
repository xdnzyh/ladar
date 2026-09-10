# 候选固件修订

本目录不代表当前设备已经烧录的内容，也不作为默认烧录入口。

`measurement_main_MEASUREMENT_SYNC_CAL_V3_candidate.py` 针对 CCD 超时后的迟到帧、采集互斥和启动期串口排空做候选修订，并已将固定曝光统一为当前标定使用的档位 5；无线 AUX 持续未就绪时会在 USB 控制台报告发送队列阻塞。`rotation_main_ROTATION_SYNC_MP_V2_candidate.py` 针对启动屏蔽状态做候选修订。它们需要在真实设备上单独编译、上传和回放验证后，才可以用于正式同步扫描。
