# One Dex3-1 hand. Seven motors, in the order they appear on
# rt/dex3/<side>/state: thumb_0 thumb_1 thumb_2 middle_0 middle_1 index_0 index_1.
# MotorCmd.mode is ignored; the bridge packs the Dex3 mode bitfield itself.
float64 duration
bool hold_position
MotorCmd[] motor_cmd
