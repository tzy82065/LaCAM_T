import pandas as pd
import os
# import matplotlib.pyplot as plt
# import seaborn as sns

# 定义地图名称列表
map_names = [
    'empty-32-32',
    'random-32-32-20',
    'room-64-64-8',
    'warehouse-10-20-10-2-2',
    'den520d'
]

# 定义 agent_num 列表: 5, 10, ..., 50
agent_nums = list(range(5, 101, 5))

# 定义基础路径
base_dir = '../experiment'

# 数据行数设定 (根据之前的调试结果设为25)
n_cases = 25 

# 初始化绘图风格
# sns.set(style="whitegrid")

for map_name in map_names:
    # --- 第一步：数据处理与CSV生成 ---
    
    # 创建 DataFrame 存储原始差值数值 (用于绘图)
    raw_diff_df = pd.DataFrame(index=range(1, n_cases + 1))
    # 创建 DataFrame 存储格式化字符串 (用于存CSV)
    formatted_diff_df = pd.DataFrame(index=range(1, n_cases + 1))
    
    map_path = os.path.join(base_dir, map_name)
    print(f"正在处理地图: {map_name} ...")
    
    for num in agent_nums:
        # folder_base = str(num)
        folder_base = str(num)
        folder_ucb = f"{num}_ucb"
        
        file_base = os.path.join(map_path, folder_base, 'summary.csv')
        file_ucb = os.path.join(map_path, folder_ucb, 'summary.csv')
        
        if not os.path.exists(file_base) or not os.path.exists(file_ucb):
            print(f"  [警告] 文件不存在: {num}，跳过。")
            continue
            
        try:
            df_base = pd.read_csv(file_base)
            df_ucb = pd.read_csv(file_ucb)
            
            makespan_base = df_base['makespan'].iloc[:n_cases].values
            makespan_ucb = df_ucb['makespan'].iloc[:n_cases].values
            
            # 自动对齐长度
            min_len = min(len(makespan_base), len(makespan_ucb))
            makespan_base = makespan_base[:min_len]
            makespan_ucb = makespan_ucb[:min_len]

            # 计算差距数值
            diff_vals = []
            for b, u in zip(makespan_base, makespan_ucb):
                if b == 0:
                    diff_vals.append(0.0) # 避免除零，设为0
                else:
                    diff_vals.append((u - b) / b)
            
            # 存入原始数据表
            raw_diff_df[num] = pd.Series(diff_vals, index=range(1, min_len + 1))
            # 存入格式化表
            formatted_diff_df[num] = raw_diff_df[num].apply(lambda x: "{:.2%}".format(x))
            
        except Exception as e:
            print(f"  [错误] 处理 {num} 时发生错误: {e}")
    
    # 保存 CSV
    csv_filename = f"{map_name}.csv"
    formatted_diff_df.to_csv(csv_filename)
    print(f"  已生成表格: {csv_filename}")


print("\n所有处理完成！")