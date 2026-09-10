# CONFIG1参数表

95项执行参数。JSON填写实际数值；串口传输值为实际数值乘“倍率”后的无符号16位整数。参数编号固定，CONFIG版本1；当前默认整组CRC16为5809（十进制）。

速度目标单位为编码器计数/20毫秒，PWM为0～255编码中的受限区间，时间为毫秒，刹车目标与提前量为CNT。高级增益沿用原控制公式与单位。逐项范围之外还需满足速度档位、前馈/PWM、刹车表、降速区间等组合约束。

前44项是LONG/LAT/DIAG/ROT四类profile，每类11项；44～75为八方向刹车表；76～80为超时、保持与降速区间；81～94为高级控制增益。默认不调高级增益。

| 编号 | 参数 | 默认实际值 | 可设置范围 | 倍率 |
| ---: | --- | ---: | --- | ---: |
| 0 | `LONG_FAST` | 40 | 12～50 | 100 |
| 1 | `LONG_MID` | 34 | 12～50 | 100 |
| 2 | `LONG_SLOW` | 28 | 12～50 | 100 |
| 3 | `LONG_FF_FAST` | 100 | 60～180 | 1 |
| 4 | `LONG_FF_MID` | 98 | 60～180 | 1 |
| 5 | `LONG_FF_SLOW` | 96 | 60～180 | 1 |
| 6 | `LONG_START_SPEED` | 20 | 12～50 | 100 |
| 7 | `LONG_START_FF` | 92 | 60～180 | 1 |
| 8 | `LONG_RAMP_MS` | 600 | 100～2000 | 1 |
| 9 | `LONG_MIN_PWM` | 90 | 60～120 | 1 |
| 10 | `LONG_MAX_PWM` | 120 | 90～180 | 1 |
| 11 | `LAT_FAST` | 32 | 12～50 | 100 |
| 12 | `LAT_MID` | 28 | 12～50 | 100 |
| 13 | `LAT_SLOW` | 24 | 12～50 | 100 |
| 14 | `LAT_FF_FAST` | 135 | 60～180 | 1 |
| 15 | `LAT_FF_MID` | 130 | 60～180 | 1 |
| 16 | `LAT_FF_SLOW` | 125 | 60～180 | 1 |
| 17 | `LAT_START_SPEED` | 20 | 12～50 | 100 |
| 18 | `LAT_START_FF` | 150 | 60～180 | 1 |
| 19 | `LAT_RAMP_MS` | 500 | 100～2000 | 1 |
| 20 | `LAT_MIN_PWM` | 90 | 60～120 | 1 |
| 21 | `LAT_MAX_PWM` | 180 | 90～180 | 1 |
| 22 | `DIAG_FAST` | 32 | 12～50 | 100 |
| 23 | `DIAG_MID` | 28 | 12～50 | 100 |
| 24 | `DIAG_SLOW` | 24 | 12～50 | 100 |
| 25 | `DIAG_FF_FAST` | 130 | 60～180 | 1 |
| 26 | `DIAG_FF_MID` | 125 | 60～180 | 1 |
| 27 | `DIAG_FF_SLOW` | 120 | 60～180 | 1 |
| 28 | `DIAG_START_SPEED` | 20 | 12～50 | 100 |
| 29 | `DIAG_START_FF` | 145 | 60～180 | 1 |
| 30 | `DIAG_RAMP_MS` | 500 | 100～2000 | 1 |
| 31 | `DIAG_MIN_PWM` | 90 | 60～120 | 1 |
| 32 | `DIAG_MAX_PWM` | 180 | 90～180 | 1 |
| 33 | `ROT_FAST` | 28 | 12～50 | 100 |
| 34 | `ROT_MID` | 24 | 12～50 | 100 |
| 35 | `ROT_SLOW` | 20 | 12～50 | 100 |
| 36 | `ROT_FF_FAST` | 96 | 60～180 | 1 |
| 37 | `ROT_FF_MID` | 94 | 60～180 | 1 |
| 38 | `ROT_FF_SLOW` | 92 | 60～180 | 1 |
| 39 | `ROT_START_SPEED` | 16 | 12～50 | 100 |
| 40 | `ROT_START_FF` | 90 | 60～180 | 1 |
| 41 | `ROT_RAMP_MS` | 400 | 100～2000 | 1 |
| 42 | `ROT_MIN_PWM` | 90 | 60～120 | 1 |
| 43 | `ROT_MAX_PWM` | 120 | 90～180 | 1 |
| 44 | `BRAKE_W_SHORT_TARGET` | 700 | 16～16000 | 1 |
| 45 | `BRAKE_W_LONG_TARGET` | 1400 | 17～16000 | 1 |
| 46 | `BRAKE_W_SHORT_LEAD` | 100 | 0～2000 | 1 |
| 47 | `BRAKE_W_LONG_LEAD` | 125 | 0～2000 | 1 |
| 48 | `BRAKE_S_SHORT_TARGET` | 700 | 16～16000 | 1 |
| 49 | `BRAKE_S_LONG_TARGET` | 1400 | 17～16000 | 1 |
| 50 | `BRAKE_S_SHORT_LEAD` | 95 | 0～2000 | 1 |
| 51 | `BRAKE_S_LONG_LEAD` | 125 | 0～2000 | 1 |
| 52 | `BRAKE_A_SHORT_TARGET` | 700 | 16～16000 | 1 |
| 53 | `BRAKE_A_LONG_TARGET` | 1400 | 17～16000 | 1 |
| 54 | `BRAKE_A_SHORT_LEAD` | 90 | 0～2000 | 1 |
| 55 | `BRAKE_A_LONG_LEAD` | 75 | 0～2000 | 1 |
| 56 | `BRAKE_D_SHORT_TARGET` | 700 | 16～16000 | 1 |
| 57 | `BRAKE_D_LONG_TARGET` | 1400 | 17～16000 | 1 |
| 58 | `BRAKE_D_SHORT_LEAD` | 75 | 0～2000 | 1 |
| 59 | `BRAKE_D_LONG_LEAD` | 75 | 0～2000 | 1 |
| 60 | `BRAKE_Q_SHORT_TARGET` | 600 | 16～16000 | 1 |
| 61 | `BRAKE_Q_LONG_TARGET` | 1200 | 17～16000 | 1 |
| 62 | `BRAKE_Q_SHORT_LEAD` | 50 | 0～2000 | 1 |
| 63 | `BRAKE_Q_LONG_LEAD` | 55 | 0～2000 | 1 |
| 64 | `BRAKE_E_SHORT_TARGET` | 600 | 16～16000 | 1 |
| 65 | `BRAKE_E_LONG_TARGET` | 1200 | 17～16000 | 1 |
| 66 | `BRAKE_E_SHORT_LEAD` | 50 | 0～2000 | 1 |
| 67 | `BRAKE_E_LONG_LEAD` | 50 | 0～2000 | 1 |
| 68 | `BRAKE_Z_SHORT_TARGET` | 600 | 16～16000 | 1 |
| 69 | `BRAKE_Z_LONG_TARGET` | 1200 | 17～16000 | 1 |
| 70 | `BRAKE_Z_SHORT_LEAD` | 65 | 0～2000 | 1 |
| 71 | `BRAKE_Z_LONG_LEAD` | 70 | 0～2000 | 1 |
| 72 | `BRAKE_C_SHORT_TARGET` | 600 | 16～16000 | 1 |
| 73 | `BRAKE_C_LONG_TARGET` | 1200 | 17～16000 | 1 |
| 74 | `BRAKE_C_SHORT_LEAD` | 45 | 0～2000 | 1 |
| 75 | `BRAKE_C_LONG_LEAD` | 50 | 0～2000 | 1 |
| 76 | `RUN_TIMEOUT_MS` | 8000 | 1000～15000 | 1 |
| 77 | `BRAKE_HOLD_MS` | 300 | 100～600 | 1 |
| 78 | `IDLE_BRAKE_HOLD_MS` | 200 | 100～600 | 1 |
| 79 | `MID_ZONE_PERCENT` | 45 | 5～90 | 1 |
| 80 | `SLOW_ZONE_PERCENT` | 20 | 1～80 | 1 |
| 81 | `WHEEL_KP` | 0.8 | 0～3 | 10000 |
| 82 | `WHEEL_KI` | 0.3 | 0～1 | 10000 |
| 83 | `WHEEL_I_OUT_LIMIT` | 10 | 0～20 | 100 |
| 84 | `KR_D` | 1.25 | 0～3 | 10000 |
| 85 | `R_MEM_LEAK` | 0.985 | 0.9～0.9999 | 10000 |
| 86 | `K_RMEM` | 0.02 | 0～0.1 | 10000 |
| 87 | `R_MEM_LIMIT` | 75 | 1～150 | 100 |
| 88 | `MAX_RMEM_CORRECTION` | 1.5 | 0～5 | 100 |
| 89 | `MAX_R_CORRECTION` | 5 | 0～10 | 100 |
| 90 | `KY_P` | 0.001 | 0～0.01 | 10000 |
| 91 | `KY_D` | 0.15 | 0～0.5 | 10000 |
| 92 | `MAX_Y_CORRECTION` | 1.2 | 0～5 | 100 |
| 93 | `KS_D` | 0.12 | 0～0.5 | 10000 |
| 94 | `MAX_S_CORRECTION` | 0.8 | 0～5 | 100 |

距离系数独立存于JSON的counts_per_mm，共W/S/A/D/Q/E/Z/C八项，单位CNT/mm，最多4位小数，范围0.0001～1000。它们由新电脑工具换算后发送CNT，不写入此95项设备参数，也不修改固件内置MM系数。

