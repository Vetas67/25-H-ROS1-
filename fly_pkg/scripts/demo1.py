# -*- coding: utf-8 -*-

import struct
import time
import rospy

# 浮点数数组
data = [1.23, 4.56, 7.89]

# 计算校验值（这里使用简单的异或校验）
checksum = 0
for num in data:
    checksum ^= struct.unpack('I', struct.pack('f', num))[0]

print("解析前的数据:", data)
print("解析前的校验值:", checksum)

# 将数据和校验值打包
packet = struct.pack('<{}fI'.format(len(data)), *(data + [checksum]))

received_data = struct.unpack('<3fI', packet)

# 提取校验值
received_checksum = received_data[-1]
received_data = received_data[:-1]

calculated_checksum = 0
for num in received_data:
    calculated_checksum ^= struct.unpack('I', struct.pack('f', num))[0]

if received_checksum == calculated_checksum:
    print("Received data:", received_data)
else:
    print("Checksum error!")

print("解析后的数据:", received_data)
print("解析后的校验值:", received_checksum)
