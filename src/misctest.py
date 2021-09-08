import numpy as np

roll_x = np.radians(45)
pitch_y = np.radians(90)

m_x = 1
m_y = 1
m_z = 1

yaw_y = m_x * np.sin(roll_x) * np.sin(pitch_y) + m_y * np.cos(roll_x) + m_z * np.sin(roll_x) * np.cos(pitch_y)
yaw_x = m_x * np.cos(pitch_y) + m_z * np.sin(pitch_y)

yaw_z = np.degrees(np.arctan2(yaw_y, yaw_x))
print(yaw_z)

yaw_y = m_z * np.sin(roll_x) - m_y * np.cos(roll_x)
yaw_x = m_x * np.cos(pitch_y) + m_y * np.sin(roll_x) * np.sin(pitch_y) + m_z * np.cos(roll_x) * np.sin(pitch_y)

yaw_z = np.degrees(np.arctan2(yaw_y, yaw_x))
print(yaw_z)
