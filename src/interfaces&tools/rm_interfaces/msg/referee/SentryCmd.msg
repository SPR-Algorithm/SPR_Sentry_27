# SentryCmd.msg

# Sentry posture
uint8 POSTURE_ATTACK=1
uint8 POSTURE_DEFENSE=2
uint8 POSTURE_MOBILE=3
uint8 POSTURE_ENHANCED_ATTACK=4
uint8 POSTURE_ENHANCED_DEFENSE=5
uint8 POSTURE_ENHANCED_MOBILE=6

bool resurrection_en  # 确认复活为1,不复活为0(即使已读条完成)
bool buy_resurrection_en  # 确认买活为1(等待直至满足条件)
uint16 buy_projectile_allowance  # 将购买发弹量,仅允许递增10倍数,开局为0
uint16 buy_projectile_times  # 远程买弹,每次仅能增加1(100发)
uint16 buy_hp_times  # 远程买血,每次仅能增加1(6s后增加血量上限60%)
uint8 desired_posture  # 请求切换到的姿态
bool confirm_activate_rune  # 确认激活能量机关为1,否则为0(默认)