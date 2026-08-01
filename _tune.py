import sys

with open('application/balance/balance.c', 'r', encoding='utf-8') as f:
    bc = f.read()
with open('application/balance/balance.h', 'r', encoding='utf-8') as f:
    bh = f.read()

changes = 0

# ==================== balance.h ====================
old = '''#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ========== API ========== */'''

new = '''#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ========== PID 在线调参 ========== */

typedef enum {
\tBALANCE_PARAM_KP = 0,       /**< °/mm */
\tBALANCE_PARAM_KI,           /**< °/(mm·s) */
\tBALANCE_PARAM_KD,           /**< °/(mm/s) */
\tBALANCE_PARAM_P_LIMIT,      /**< °, P 项限幅 */
\tBALANCE_PARAM_I_LIMIT,      /**< °, I 项限幅 */
\tBALANCE_PARAM_D_LIMIT,      /**< °, D 项限幅 */
\tBALANCE_PARAM_DEADBAND,     /**< mm, D 项死区 */
\tBALANCE_PARAM_POS_ALPHA,    /**< 位置低通系数 */
\tBALANCE_PARAM_VEL_ALPHA,    /**< 速度低通系数 */
\tBALANCE_PARAM_FF_GAIN,      /**< °/(m/s²), FF 增益 */
\tBALANCE_PARAM_FF_DEADZONE,  /**< m/s², FF 死区 */
\tBALANCE_PARAM_FF_FILTER,    /**< FF 低通系数 */
\tBALANCE_PARAM_MAX_ANGLE,    /**< °, 输出总限幅 */
\tBALANCE_PARAM_RESET,        /**< 清零积分（value 忽略） */
} Balance_ParamID;

void Balance_SetParam(Balance_ParamID id, float value);
float Balance_GetParam(Balance_ParamID id);

/* ========== API ========== */'''

assert old in bh, "bh enum not found"
bh = bh.replace(old, new, 1)
changes += 1
print(f'{changes} OK: balance.h')

with open('application/balance/balance.h', 'w', encoding='utf-8') as f:
    f.write(bh)

# ==================== balance.c ====================

# 1. Runtime param variables
old = '''static uint32_t s_last_update_ms;

/* ========== 内部控制 ========== */'''

new = '''static uint32_t s_last_update_ms;

/* ========== 可调参数（运行时副本，Pi 可在线修改） ========== */

static float s_kp          = BALANCE_KP;
static float s_ki          = BALANCE_KI;
static float s_kd          = BALANCE_KD;
static float s_p_limit     = BALANCE_P_LIMIT_DEG;
static float s_i_limit     = BALANCE_I_LIMIT_DEG;
static float s_d_limit     = BALANCE_D_LIMIT_DEG;
static float s_deadband    = BALANCE_DEADBAND_MM;
static float s_pos_alpha   = POS_FILTER_ALPHA;
static float s_vel_alpha   = VEL_FILTER_ALPHA;
static float s_ff_gain     = FF_ACCEL_GAIN;
static float s_ff_deadzone = FF_ACCEL_DEADZONE;
static float s_ff_filter   = FF_ACCEL_FILTER;
static float s_max_angle   = BALANCE_MAX_ANGLE_DEG;

/* ========== 内部控制 ========== */'''

assert old in bc, "runtime vars not found"
bc = bc.replace(old, new, 1)
changes += 1
print(f'{changes} OK: runtime vars')

# 2. Replace all macro refs with runtime vars in Balance_Update + Balance_ChassisFF
replacements = [
    ('\tangle_p = BALANCE_KP * pos_error;',               '\tangle_p = s_kp * pos_error;'),
    ('\tif (angle_p > BALANCE_P_LIMIT_DEG)\n\t\tangle_p = BALANCE_P_LIMIT_DEG;\n\telse if (angle_p < -BALANCE_P_LIMIT_DEG)\n\t\tangle_p = -BALANCE_P_LIMIT_DEG;',
     '\tif (angle_p > s_p_limit)\n\t\tangle_p = s_p_limit;\n\telse if (angle_p < -s_p_limit)\n\t\tangle_p = -s_p_limit;'),
    ('\tangle_d = BALANCE_KD * s_ball_vel;',               '\tangle_d = s_kd * s_ball_vel;'),
    ('\tif (abs_err < BALANCE_DEADBAND_MM)\n\t\tangle_d *= (abs_err / BALANCE_DEADBAND_MM);',
     '\tif (abs_err < s_deadband)\n\t\tangle_d *= (abs_err / s_deadband);'),
    ('\tif (angle_d > BALANCE_D_LIMIT_DEG)\n\t\tangle_d = BALANCE_D_LIMIT_DEG;\n\telse if (angle_d < -BALANCE_D_LIMIT_DEG)\n\t\tangle_d = -BALANCE_D_LIMIT_DEG;',
     '\tif (angle_d > s_d_limit)\n\t\tangle_d = s_d_limit;\n\telse if (angle_d < -s_d_limit)\n\t\tangle_d = -s_d_limit;'),
    ('\t\t\ts_i_accum += BALANCE_KI * pos_error * dt_s;',  '\t\t\ts_i_accum += s_ki * pos_error * dt_s;'),
    ('\t\tif (s_i_accum > BALANCE_I_LIMIT_DEG)\n\t\t\ts_i_accum = BALANCE_I_LIMIT_DEG;\n\t\telse if (s_i_accum < -BALANCE_I_LIMIT_DEG)\n\t\t\ts_i_accum = -BALANCE_I_LIMIT_DEG;',
     '\t\tif (s_i_accum > s_i_limit)\n\t\t\ts_i_accum = s_i_limit;\n\t\telse if (s_i_accum < -s_i_limit)\n\t\t\ts_i_accum = -s_i_limit;'),
    ('&& pd_sum > -BALANCE_MAX_ANGLE_DEG\n\t\t    && pd_sum < BALANCE_MAX_ANGLE_DEG)',
     '&& pd_sum > -s_max_angle\n\t\t    && pd_sum < s_max_angle)'),
    ('s_ball_pos = s_ball_pos * (1.0f - POS_FILTER_ALPHA)\n\t           + ball_pos_mm    * POS_FILTER_ALPHA;',
     's_ball_pos = s_ball_pos * (1.0f - s_pos_alpha)\n\t           + ball_pos_mm    * s_pos_alpha;'),
    ('s_ball_vel = s_ball_vel * (1.0f - VEL_FILTER_ALPHA)\n\t\t           + raw_vel    * VEL_FILTER_ALPHA;',
     's_ball_vel = s_ball_vel * (1.0f - s_vel_alpha)\n\t\t           + raw_vel    * s_vel_alpha;'),
    ('\ts_ff_accel += FF_ACCEL_GAIN * accel_m_s2;',        '\ts_ff_accel += s_ff_gain * accel_m_s2;'),
]

for i, (old_s, new_s) in enumerate(replacements):
    assert old_s in bc, f"replacement {i} not found: {repr(old_s[:40])}"
    bc = bc.replace(old_s, new_s, 1)

changes += 1
print(f'{changes} OK: {len(replacements)} macro→runtime replacements')

# 3. GetP/GetD
old = '''float Balance_GetP(void)
{
\tfloat error = s_target_pos - s_ball_pos;
\treturn BALANCE_KP * error;
}

float Balance_GetD(void)
{
\treturn BALANCE_KD * s_ball_vel;
}'''

new = '''float Balance_GetP(void)
{
\tfloat error = s_target_pos - s_ball_pos;
\treturn s_kp * error;
}

float Balance_GetD(void)
{
\treturn s_kd * s_ball_vel;
}'''

assert old in bc, "GetP/GetD not found"
bc = bc.replace(old, new, 1)
changes += 1
print(f'{changes} OK: GetP/GetD')

# 4. SetParam/GetParam
old = '''/* ========== PID 控制更新（Pi @50Hz 调用） ========== */'''

new = '''/* ========== 在线调参 ========== */

void Balance_SetParam(Balance_ParamID id, float value)
{
\tswitch (id)
\t{
\tcase BALANCE_PARAM_KP:          s_kp          = value; break;
\tcase BALANCE_PARAM_KI:          s_ki          = value; break;
\tcase BALANCE_PARAM_KD:          s_kd          = value; break;
\tcase BALANCE_PARAM_P_LIMIT:     s_p_limit     = value; break;
\tcase BALANCE_PARAM_I_LIMIT:     s_i_limit     = value; break;
\tcase BALANCE_PARAM_D_LIMIT:     s_d_limit     = value; break;
\tcase BALANCE_PARAM_DEADBAND:    s_deadband    = value; break;
\tcase BALANCE_PARAM_POS_ALPHA:   s_pos_alpha   = value; break;
\tcase BALANCE_PARAM_VEL_ALPHA:   s_vel_alpha   = value; break;
\tcase BALANCE_PARAM_FF_GAIN:     s_ff_gain     = value; break;
\tcase BALANCE_PARAM_FF_DEADZONE: s_ff_deadzone = value; break;
\tcase BALANCE_PARAM_FF_FILTER:   s_ff_filter   = value; break;
\tcase BALANCE_PARAM_MAX_ANGLE:   s_max_angle   = value; break;
\tcase BALANCE_PARAM_RESET:       s_i_accum     = 0.0f;  break;
\t}
}

float Balance_GetParam(Balance_ParamID id)
{
\tswitch (id)
\t{
\tcase BALANCE_PARAM_KP:          return s_kp;
\tcase BALANCE_PARAM_KI:          return s_ki;
\tcase BALANCE_PARAM_KD:          return s_kd;
\tcase BALANCE_PARAM_P_LIMIT:     return s_p_limit;
\tcase BALANCE_PARAM_I_LIMIT:     return s_i_limit;
\tcase BALANCE_PARAM_D_LIMIT:     return s_d_limit;
\tcase BALANCE_PARAM_DEADBAND:    return s_deadband;
\tcase BALANCE_PARAM_POS_ALPHA:   return s_pos_alpha;
\tcase BALANCE_PARAM_VEL_ALPHA:   return s_vel_alpha;
\tcase BALANCE_PARAM_FF_GAIN:     return s_ff_gain;
\tcase BALANCE_PARAM_FF_DEADZONE: return s_ff_deadzone;
\tcase BALANCE_PARAM_FF_FILTER:   return s_ff_filter;
\tcase BALANCE_PARAM_MAX_ANGLE:   return s_max_angle;
\tdefault:                        return 0.0f;
\t}
}

/* ========== PID 控制更新（Pi @50Hz 调用） ========== */'''

assert old in bc, "SetParam insert not found"
bc = bc.replace(old, new, 1)
changes += 1
print(f'{changes} OK: SetParam/GetParam')

with open('application/balance/balance.c', 'w', encoding='utf-8') as f:
    f.write(bc)

print(f'\nAll {changes} changes written.')
