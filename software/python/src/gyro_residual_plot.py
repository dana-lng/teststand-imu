# Plot if more imus tested.

import matplotlib.pyplot as plt

mou6050_gyro_res_before = 25.498744910962415
mou6050_gyro_res_after = 12.546341986819943

plt.bar(['Before Calibration', 'After Calibration'], [mou6050_gyro_res_before, mou6050_gyro_res_after], color=['red', 'green'])
plt.ylabel('Gyroscope Residual')
plt.title('Gyroscope Residual Before and After Calibration')
plt.show()