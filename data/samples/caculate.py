import numpy as np
from scipy.spatial.transform import Rotation as R

# 定义旋转平移矩阵
affine_list = [[0.2111, -0.9774, 0.0108, -0.1016],
          [0.0456, -0.0012, -0.9990, -0.4083],
          [0.9764, 0.2114, 0.0443, -0.3710],
          [0  , 0  , 0  , 1 ]]  # 4x4单位矩阵作为示例
# 填充实际的旋转平移矩阵 affine = [[r11, r12, r13, t1],
#                            [r21, r22, r23, t2],
#                            [r31, r32, r33, t3],
#                            [  0,   0,   0,  1]]
affine = np.array(affine_list)
translation = affine[:3, 3]  # 提取平移部分
rotation_matrix = affine[:3, :3]  # 提取旋转部分

# 将旋转矩阵转换为欧拉角
r = R.from_matrix(rotation_matrix)
euler = r.as_euler('zyx', degrees=True)  # 返回欧拉角，单位为度

print("Translation:", translation)
print("Euler Angles (degrees):", euler)

