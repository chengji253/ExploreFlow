import json

# 打开并读取 JSON 文件
with open('flow500.json', 'r', encoding='utf-8') as file:
    data = json.load(file)

flow_three_section = data.get('flow_section_list')


flow_sum_list = []
for flow_data in flow_three_section:
    flow_data_sum = sum(flow_data)
    flow_sum_list.append(flow_data_sum)

print()