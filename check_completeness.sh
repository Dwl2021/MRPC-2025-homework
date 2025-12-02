#!/bin/bash

echo "--------------------------------"
echo "正在检查最终提交格式..."
echo "--------------------------------"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查必需文件
missing_files=()
required_files=(
    "solutions/report.pdf"
    "solutions/video.mp4"
    "solutions/result.txt"
    "code/src/quadrotor_simulator/so3_control/src/issafe.txt"
    "code/src/quadrotor_simulator/so3_control/src/control_timedata.txt"
    "code/src/quadrotor_simulator/so3_control/src/control_data.txt"
)

echo "【必需文件检查】"
empty_files=()
for file in "${required_files[@]}"; do
    if [ -f "$file" ]; then
        echo -e "${GREEN}✓${NC} $file 存在"
        if [[ "$file" == *.txt ]] && [[ "$file" != *"issafe.txt" ]]; then
            if [ ! -s "$file" ]; then
                empty_files+=("$file")
            fi
        fi
    else
        echo -e "${RED}✗${NC} $file 缺失"
        missing_files+=("$file")
    fi
done
echo ""

# 如果有空文件，给出警告
if [ ${#empty_files[@]} -gt 0 ]; then
    echo -e "${YELLOW}警告：以下txt文件为空，请检查：${NC}"
    for file in "${empty_files[@]}"; do
        echo "  - $file"
    done
    echo ""
fi

# 检查可选文件（题目三）
echo "【题目三选做检查】"
optional_file1="solutions/df_quaternion.csv"
optional_dir="code/src/quadrotor_df"

if [ ! -f "$optional_file1" ] && [ ! -d "$optional_dir" ]; then
    echo -e "${YELLOW}你没有选做 homework.pdf 的题目三${NC}"
else
    echo -e "${GREEN}你选做了题目三${NC}"
    [ ! -f "$optional_file1" ] && echo -e "${YELLOW}  - 缺少 $optional_file1 (结果文件)${NC}"
    [ ! -d "$optional_dir" ] && echo -e "${YELLOW}  - 缺少 $optional_dir (ROS包)${NC}"
fi
echo ""

# 检查编译文件
echo "【编译文件检查】"
compile_dirs=(
    "code/devel"
    "code/build"
    "code/logs"
)

found_compile_dirs=()
for dir in "${compile_dirs[@]}"; do
    if [ -d "$dir" ]; then
        found_compile_dirs+=("$dir")
    fi
done

if [ ${#found_compile_dirs[@]} -gt 0 ]; then
    echo -e "${YELLOW}编译文件无需提交，请压缩之前先执行以下命令删除编译文件：${NC}"
    for dir in "${found_compile_dirs[@]}"; do
        echo "  rm -rf $dir"
    done
else
    echo -e "${GREEN}✓${NC} 未发现编译文件，很好！"
fi
echo ""


# 检查所有必须的文件（如未报错或警告则全部检查通过）
all_ok=true

if [ ${#missing_files[@]} -ne 0 ] || [ ${#empty_files[@]} -ne 0 ] || [ ${#found_compile_dirs[@]} -ne 0 ]; then
    all_ok=false
fi

if $all_ok; then
    echo -e "${GREEN}所有检查均通过，可以执行如下示例命令进行压缩并提交：${NC}"
    echo ""
    echo "zip -r 张三-23123456.zip ../MRPC-2025-homework"
    echo ""
    echo "（请将生成的 zip 文件提交到 SYSU_HILAB_Course@163.com ）"
else
    echo -e "${YELLOW}未全部通过检查，请修正上面的问题后再压缩提交。${NC}"
fi


