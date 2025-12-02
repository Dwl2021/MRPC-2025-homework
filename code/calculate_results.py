import numpy as np
import os
def calculate_rmse_and_more():
    des_pos_data = []
    pos_data = []
    time_stamps = []
    time_stamps2 = []
    length_increments = []

    with open("/home/stuwork/MRPC-2025-homework/code/src/quadrotor_simulator/so3_control/src/control_data.txt", "r") as file:
        lines = file.readlines()
        for i, line in enumerate(lines):
            data = line.strip().split()
            des_pos = np.array([float(data[1]), float(data[2]), float(data[3])])
            pos = np.array([float(data[4]), float(data[5]), float(data[6])])
            des_pos_data.append(des_pos)
            pos_data.append(pos)
            time_stamps.append(float(data[0]))

            if i > 0:
                # 计算当前时刻与上一时刻实际运行位置的差值作为轨迹长度增量
                prev_pos = np.array([float(lines[i - 1].strip().split()[4]),
                                    float(lines[i - 1].strip().split()[5]),
                                    float(lines[i - 1].strip().split()[6])])
                length_increment = np.linalg.norm(pos - prev_pos)
                length_increments.append(length_increment)

    des_pos_array = np.array(des_pos_data)
    pos_array = np.array(pos_data)
    time_stamps_array = np.array(time_stamps)
    length_increments_array = np.array(length_increments)

    if des_pos_array.shape!= pos_array.shape:
        raise ValueError("期望位置数据和实际位置数据的形状不一致。")
    
    with open("/home/stuwork/MRPC-2025-homework/code/src/quadrotor_simulator/so3_control/src/control_timedata.txt", "r") as file:
        lines = file.readlines()
        for i, line in enumerate(lines):
            datatime = line.strip().split()
            time_stamps2.append(float(datatime[0]))
    
    time_stamps2_array = np.array(time_stamps2)

    # 计算均方根误差（RMSE）
    diff_array = des_pos_array - pos_array
    squared_diff_array = diff_array ** 2
    mean_squared_error = np.mean(squared_diff_array)
    rmse = np.sqrt(mean_squared_error)

    # 计算轨迹运行总时间
    total_time = time_stamps2_array[-1] - time_stamps_array[0]

    # 计算总轨迹长度
    total_length = np.sum(length_increments_array)

    # 检查是否发生了碰撞
    additional_score = check_additional_file()

    overall_score = 200. * rmse + 1./5. * total_time + 1./5. * total_length + 10. * additional_score
    
    return rmse, total_time, total_length, additional_score, overall_score
def check_additional_file():
    file_path = "/home/stuwork/MRPC-2025-homework/code/src/quadrotor_simulator/so3_control/src/issafe.txt"  # 替换为实际的文件路径
    try:
        with open(file_path, "r") as file:
            content = file.read().strip()
            if content:
                return 1
            return 0
    except FileNotFoundError:
        return 0

if __name__ == "__main__":
    try:
        rmse, total_time, total_length, additional_score, overall_score = calculate_rmse_and_more()
        print(f"计算得到的均方根误差（RMSE）值为: {rmse}")
        print(f"轨迹运行总时间为: {total_time}")
        print(f"总轨迹长度为: {total_length}")
        print(f"是否发生了碰撞: {additional_score}")
        print(f"综合评价得分为(综合分数越低越好): {overall_score}")
        
        result_file_path = "/home/stuwork/MRPC-2025-homework/solutions/result.txt"
        with open(result_file_path, "w") as f:
            f.write(f"{rmse} {total_time} {total_length} {additional_score} {overall_score}")
    except ValueError as e:
        print(f"发生错误: {e}")
