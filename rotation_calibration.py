"""Two-anchor rotation conversion shared by navigation request/result paths."""
import math


def validate_calibration(data):
    if not isinstance(data, dict) or data.get('schema') != 2 or data.get('enabled') is not True:
        raise ValueError('旋转停车补偿必须启用且为 schema 2')
    for mode in ('r', 'f'):
        values = [data.get(mode + suffix) for suffix in ('_500_deg', '_1000_deg', '_lead_cnt')]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
            raise ValueError('旋转标定值必须为有限数值')
        small, large, lead = values
        if not 0 < small < large <= 180 or not 0 <= lead < 500:
            raise ValueError('旋转标定锚点或提前量无效')
    return data


def goal_counts(data, mode, angle_rad):
    validate_calibration(data)
    degrees = math.degrees(abs(angle_rad))
    if not math.isfinite(degrees) or not 10 <= degrees <= 25.000001:
        raise ValueError('自动导航旋转只允许 10–25° 小角度')
    small, large = data[mode.lower() + '_500_deg'], data[mode.lower() + '_1000_deg']
    goal = 500 * degrees / small if degrees <= small else 500 + 500 * (degrees - small) / (large - small)
    goal = int(math.floor(goal + .5))
    wire = goal - int(data[mode.lower() + '_lead_cnt'])
    if wire <= 0:
        raise ValueError('旋转目标扣除停车提前量后非正')
    return goal, wire


def angle_from_encoder(data, mode, encoder):
    validate_calibration(data)
    if not math.isfinite(encoder) or encoder < 0:
        raise ValueError('旋转编码器值无效')
    small, large = data[mode.lower() + '_500_deg'], data[mode.lower() + '_1000_deg']
    degrees = encoder * small / 500 if encoder <= 500 else small + (encoder - 500) * (large - small) / 500
    return math.radians(degrees)
