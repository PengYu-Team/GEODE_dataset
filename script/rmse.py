import os
import subprocess
import re
import glob
import sys

# 从命令行读取变量
if len(sys.argv) != 4:
    print("Usage: python script.py <method> <gt> <offset>")
    sys.exit(1)

method = sys.argv[1]
gt = sys.argv[2]
offset = sys.argv[3]

# 设置FOLDER_PATH为当前文件夹
FOLDER_PATH = os.getcwd()

# 搜索包含method字符串的所有txt文件
method_files = glob.glob(os.path.join(FOLDER_PATH, f"*{method}*.txt"))

# 初始化结果列表
results = []

# 对每个文件进行误差计算
for method_file in method_files:
    command = [
        'evo_ape', 'tum',
        method_file,
        os.path.join(FOLDER_PATH, gt),
        '-va', '--t_max_diff', '0.1', '--t_offset', offset
    ]
    process = subprocess.run(command, capture_output=True, text=True)
    output = process.stdout
    print(f"Command output for {method_file}:\n{output}")

    # 解析输出以提取APE的RMSE值
    match = re.search(r"rmse\s+(\d+\.\d+)", output)
    if match:
        ape_rmse = float(match.group(1))
        results.append(ape_rmse)
        print(f"Extracted RMSE: {ape_rmse}")
    else:
        results.append(None)
        print("RMSE value not found for", method_file)

# 计算平均RMSE值
average_rmse = sum(filter(None, results)) / len(results) if results else None
print(f"Average RMSE: {average_rmse}" if average_rmse is not None else "No RMSE values were found.")

# 找出 RMSE 最小值及其对应的 t_offset
min_rmse = min(results)  # 假设 results 中没有 None 值
min_rmse_index = results.index(min_rmse)
# min_rmse_offset = t_offset_values[min_rmse_index]
print(f" The minimum min_rmse is {min_rmse}")
