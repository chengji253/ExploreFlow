# area_info_list = [(1, 1.0), (10, 2), (25, 1.0), (35, 2.5), (50, 1.0), (62, 2.0), (77, 1.0)]
# area_info_list = [(1, 1.0), (10, 2), (25, 1.0)]

# 初始化结果列表，长度为 5
area_info_list = [(1, 1.0), (10, 2.414213562373095), (25, 1.0), (35, 2.414213562373095), (50, 1.0), (62, 2.0), (77, 1.0)]

pre_time = 5

# 初始化一个长度为 5 的列表，用于存储结果，初始值设为 None
result = [None] * pre_time

# 反向遍历元组列表，这样可以保证优先处理占据长度大的元素
for cell_value, length in reversed(area_info_list):
    start = 0
    # 累加前面元组的长度得到当前元组的起始位置
    for prev_cell, prev_length in area_info_list:
        if (prev_cell, prev_length) == (cell_value, length):
            break
        start += prev_length
    end = start + length
    # 遍历 1 到 5 的每个单位长度
    for i in range(1, pre_time + 1):
        if start < i <= end:
            result[i - 1] = cell_value

print(result)
