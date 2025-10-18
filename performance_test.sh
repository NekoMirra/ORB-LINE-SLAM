#!/bin/bash

# ORB-SLAM3-LINE 性能测试脚本
# 用于测试优化前后的性能对比

echo "=========================================="
echo "  ORB-SLAM3-LINE 性能测试"
echo "=========================================="
echo ""

# 检查必要的文件
if [ ! -f "./Examples/Stereo-Line-Inertial/stereo_line_inertial_euroc" ]; then
    echo "错误: 可执行文件不存在,请先编译项目"
    echo "运行: ./build.sh"
    exit 1
fi

# 配置参数
VOCAB_ORB="./Vocabulary/ORBvoc.txt"
VOCAB_LINE="./Vocabulary/LINEvoc.bin"
CONFIG="./Examples/Stereo-Line-Inertial/EuRoC.yaml"
DATASET_PATH="${1:-/path/to/EuRoC/MH_01_easy}"  # 默认数据集路径

# 检查数据集路径
if [ ! -d "$DATASET_PATH" ]; then
    echo "错误: 数据集路径不存在: $DATASET_PATH"
    echo "用法: $0 <数据集路径>"
    echo "示例: $0 /home/user/Datasets/EuRoC/MH_01_easy"
    exit 1
fi

echo "配置信息:"
echo "  - 词汇表(ORB): $VOCAB_ORB"
echo "  - 词汇表(Line): $VOCAB_LINE"
echo "  - 配置文件: $CONFIG"
echo "  - 数据集: $DATASET_PATH"
echo ""

# 创建输出目录
OUTPUT_DIR="./performance_test_results"
mkdir -p $OUTPUT_DIR
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

echo "开始性能测试..."
echo "结果将保存到: $OUTPUT_DIR"
echo ""

# 运行测试
echo "正在运行 ORB-SLAM3-LINE..."
echo "请等待测试完成(可能需要几分钟)..."
echo ""

# 使用time命令测量性能
/usr/bin/time -v ./Examples/Stereo-Line-Inertial/stereo_line_inertial_euroc \
    $VOCAB_ORB \
    $VOCAB_LINE \
    $CONFIG \
    $DATASET_PATH \
    $OUTPUT_DIR/result_$TIMESTAMP \
    2>&1 | tee $OUTPUT_DIR/performance_log_$TIMESTAMP.txt

echo ""
echo "=========================================="
echo "  测试完成!"
echo "=========================================="
echo ""

# 提取关键性能指标
if [ -f "$OUTPUT_DIR/performance_log_$TIMESTAMP.txt" ]; then
    echo "性能统计摘要:"
    echo ""
    
    # 提取执行时间
    ELAPSED_TIME=$(grep "Elapsed (wall clock) time" $OUTPUT_DIR/performance_log_$TIMESTAMP.txt | awk '{print $8}')
    echo "  总运行时间: $ELAPSED_TIME"
    
    # 提取CPU使用率
    CPU_USAGE=$(grep "Percent of CPU" $OUTPUT_DIR/performance_log_$TIMESTAMP.txt | awk '{print $7}')
    echo "  CPU使用率: $CPU_USAGE"
    
    # 提取最大内存使用
    MAX_MEM=$(grep "Maximum resident set size" $OUTPUT_DIR/performance_log_$TIMESTAMP.txt | awk '{print $6}')
    MAX_MEM_MB=$(echo "scale=2; $MAX_MEM/1024" | bc)
    echo "  最大内存使用: ${MAX_MEM_MB}MB"
    
    echo ""
    echo "详细日志保存在: $OUTPUT_DIR/performance_log_$TIMESTAMP.txt"
    
    # 如果存在轨迹文件,显示信息
    if [ -f "$OUTPUT_DIR/result_${TIMESTAMP}_FrameTrajectory_TUM_Format.txt" ]; then
        FRAME_COUNT=$(wc -l < "$OUTPUT_DIR/result_${TIMESTAMP}_FrameTrajectory_TUM_Format.txt")
        echo "  处理帧数: $FRAME_COUNT"
        
        # 计算平均帧率
        if [ ! -z "$ELAPSED_TIME" ]; then
            # 将时间转换为秒
            TIME_IN_SECONDS=$(echo $ELAPSED_TIME | awk -F: '{ if (NF == 3) print ($1 * 3600) + ($2 * 60) + $3; else if (NF == 2) print ($1 * 60) + $2; else print $1 }')
            if [ ! -z "$TIME_IN_SECONDS" ]; then
                AVG_FPS=$(echo "scale=2; $FRAME_COUNT / $TIME_IN_SECONDS" | bc)
                echo "  平均帧率: ${AVG_FPS} FPS"
            fi
        fi
    fi
fi

echo ""
echo "=========================================="
echo "  性能对比建议"
echo "=========================================="
echo ""
echo "如果性能不理想,可以尝试:"
echo ""
echo "1. 降低特征点数量:"
echo "   修改 $CONFIG 中的 ORBextractor.nFeatures"
echo "   建议值: 800-1000 (默认1200)"
echo ""
echo "2. 关闭可视化(如果开启了):"
echo "   运行时不使用 --viewer 参数"
echo ""
echo "3. 使用Release编译模式:"
echo "   检查 build/CMakeCache.txt 中 CMAKE_BUILD_TYPE=Release"
echo ""
echo "4. 进一步优化检查频率:"
echo "   修改 Tracking.cc 中 frameCounter % 5 改为 % 10"
echo ""

# 生成简单的性能报告
cat > $OUTPUT_DIR/performance_summary_$TIMESTAMP.md << EOF
# ORB-SLAM3-LINE 性能测试报告

## 测试信息
- 测试时间: $(date)
- 数据集: $DATASET_PATH
- 配置文件: $CONFIG

## 性能指标
- 总运行时间: $ELAPSED_TIME
- CPU使用率: $CPU_USAGE
- 最大内存: ${MAX_MEM_MB}MB
- 处理帧数: $FRAME_COUNT
- 平均帧率: ${AVG_FPS} FPS

## 优化效果评估

### 预期性能目标
- 平均帧率: > 25 FPS (优秀)
- CPU使用率: < 60% (良好)
- 内存使用: < 2GB (正常)

### 实际表现
EOF

# 添加性能评估
if [ ! -z "$AVG_FPS" ]; then
    FPS_INT=$(echo $AVG_FPS | cut -d. -f1)
    if [ $FPS_INT -ge 25 ]; then
        echo "- 帧率: ✅ 优秀 (${AVG_FPS} FPS >= 25 FPS)" >> $OUTPUT_DIR/performance_summary_$TIMESTAMP.md
    elif [ $FPS_INT -ge 15 ]; then
        echo "- 帧率: ⚠️ 良好 (${AVG_FPS} FPS >= 15 FPS)" >> $OUTPUT_DIR/performance_summary_$TIMESTAMP.md
    else
        echo "- 帧率: ❌ 需要优化 (${AVG_FPS} FPS < 15 FPS)" >> $OUTPUT_DIR/performance_summary_$TIMESTAMP.md
    fi
fi

echo ""
echo "性能报告已保存到: $OUTPUT_DIR/performance_summary_$TIMESTAMP.md"
echo ""
echo "测试完成! 🎉"
