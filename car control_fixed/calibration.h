#ifndef CAR_DISTANCE_CALIBRATION_H
#define CAR_DISTANCE_CALIBRATION_H

// 可调物理标定配置。单位CNT/mm，直接修改小数值即可。
// 来源：2026-09-09现场长距离测量；安装雷达/改变载荷后可替换。
// 斜移毫米为沿45度指令方向的距离，不是单独X/Y分量。
// 修改后重新编译并上传。CNT指令不受这些系数影响。
#define CAL_W_COUNTS_PER_MM 7.1775f
#define CAL_S_COUNTS_PER_MM 7.1317f
#define CAL_A_COUNTS_PER_MM 7.2372f
#define CAL_D_COUNTS_PER_MM 7.3864f
#define CAL_Q_COUNTS_PER_MM 10.1756f
#define CAL_E_COUNTS_PER_MM 9.1232f
#define CAL_Z_COUNTS_PER_MM 9.6476f
#define CAL_C_COUNTS_PER_MM 10.1874f

// 将配置编译为万分之一精度的定点数。运行时不用浮点做距离换算。
#define CAL_FIXED(value) ((long)((value) * 10000.0f + 0.5f))
#define CAL_CHECK(value) static_assert((value) >= 0.0001f && (value) <= 1000.0f, "Invalid CNT/mm calibration")
CAL_CHECK(CAL_W_COUNTS_PER_MM);
CAL_CHECK(CAL_S_COUNTS_PER_MM);
CAL_CHECK(CAL_A_COUNTS_PER_MM);
CAL_CHECK(CAL_D_COUNTS_PER_MM);
CAL_CHECK(CAL_Q_COUNTS_PER_MM);
CAL_CHECK(CAL_E_COUNTS_PER_MM);
CAL_CHECK(CAL_Z_COUNTS_PER_MM);
CAL_CHECK(CAL_C_COUNTS_PER_MM);

#endif
