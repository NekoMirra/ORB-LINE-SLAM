#!/bin/bash

# 性能优化验证脚本
# 用于验证优化是否正确应用

echo "========================================"
echo "  ORB-SLAM3-LINE 优化验证"
echo "========================================"
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

SUCCESS=0
WARNINGS=0
ERRORS=0

# 检查函数
check_file() {
    if [ -f "$1" ]; then
        echo -e "${GREEN}✓${NC} 找到文件: $1"
        return 0
    else
        echo -e "${RED}✗${NC} 缺失文件: $1"
        ERRORS=$((ERRORS + 1))
        return 1
    fi
}

check_string() {
    local file=$1
    local pattern=$2
    local description=$3
    
    if grep -q "$pattern" "$file" 2>/dev/null; then
        echo -e "${GREEN}✓${NC} $description"
        SUCCESS=$((SUCCESS + 1))
        return 0
    else
        echo -e "${RED}✗${NC} $description"
        ERRORS=$((ERRORS + 1))
        return 1
    fi
}

check_string_warn() {
    local file=$1
    local pattern=$2
    local description=$3
    
    if grep -q "$pattern" "$file" 2>/dev/null; then
        echo -e "${GREEN}✓${NC} $description"
        SUCCESS=$((SUCCESS + 1))
        return 0
    else
        echo -e "${YELLOW}⚠${NC} $description (建议修改)"
        WARNINGS=$((WARNINGS + 1))
        return 1
    fi
}

echo "1. 检查核心文件..."
echo "-------------------"
check_file "Tracking.cc"
check_file "System.cc"
check_file "ReprojectionErrorMonitor.cc"
echo ""

echo "2. 验证Tracking.cc优化..."
echo "-------------------------"
check_string "Tracking.cc" "frameCounter % 5" "重投影误差检查频率已优化(每5帧)"
check_string "Tracking.cc" "frameCounter % 10" "可选: 使用每10帧检查(更快)" || echo "  → 当前使用每5帧(平衡模式)"
echo ""

echo "3. 验证ReprojectionErrorMonitor.cc优化..."
echo "-----------------------------------------"
check_string "ReprojectionErrorMonitor.cc" "std::min(pFrame->N, 100)" "地图点数量限制已启用"
check_string "ReprojectionErrorMonitor.cc" "depth > 0.1f && depth < 100.0f" "深度合理性检查已添加"
check_string "ReprojectionErrorMonitor.cc" "dx\*dx + dy\*dy" "使用平方误差避免sqrt"
echo ""

echo "4. 验证System.cc优化..."
echo "-----------------------"
check_string "System.cc" "errorThreshold.*5.0" "误差阈值已提高到5.0"
check_string "System.cc" "changeRateThreshold.*1.0" "变化率阈值已提高到1.0"
check_string "System.cc" "durationThreshold.*2.0" "持续时间阈值已提高到2.0秒"
check_string "System.cc" "vpKeyFrames.resize(10)" "关键帧数量限制为10"
check_string "System.cc" "FastPoseOptimizationWithValidation.*3.*1e-5" "迭代次数减少到3,容差放宽到1e-5"
echo ""

echo "5. 检查编译配置..."
echo "------------------"
if [ -f "build/CMakeCache.txt" ]; then
    if grep -q "CMAKE_BUILD_TYPE:STRING=Release" "build/CMakeCache.txt"; then
        echo -e "${GREEN}✓${NC} 使用Release编译模式"
        SUCCESS=$((SUCCESS + 1))
    else
        echo -e "${RED}✗${NC} 未使用Release编译模式"
        echo "  → 运行: cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j"
        ERRORS=$((ERRORS + 1))
    fi
else
    echo -e "${YELLOW}⚠${NC} 未找到build目录,请先编译项目"
    WARNINGS=$((WARNINGS + 1))
fi
echo ""

echo "6. 检查可执行文件..."
echo "-------------------"
if [ -f "Examples/Stereo-Line-Inertial/stereo_line_inertial_euroc" ]; then
    echo -e "${GREEN}✓${NC} 可执行文件存在"
    SUCCESS=$((SUCCESS + 1))
    
    # 检查文件日期
    BUILD_TIME=$(stat -c %Y "Examples/Stereo-Line-Inertial/stereo_line_inertial_euroc" 2>/dev/null || stat -f %m "Examples/Stereo-Line-Inertial/stereo_line_inertial_euroc" 2>/dev/null)
    CURRENT_TIME=$(date +%s)
    AGE=$((CURRENT_TIME - BUILD_TIME))
    
    if [ $AGE -lt 3600 ]; then
        echo -e "${GREEN}✓${NC} 最近编译(${AGE}秒前)"
    else
        HOURS=$((AGE / 3600))
        echo -e "${YELLOW}⚠${NC} 编译于${HOURS}小时前,建议重新编译"
        WARNINGS=$((WARNINGS + 1))
    fi
else
    echo -e "${RED}✗${NC} 可执行文件不存在"
    echo "  → 运行: ./build.sh"
    ERRORS=$((ERRORS + 1))
fi
echo ""

echo "7. 检查配置文件建议..."
echo "---------------------"
CONFIG_FILE="Examples/Stereo-Line-Inertial/EuRoC.yaml"
if [ -f "$CONFIG_FILE" ]; then
    check_string_warn "$CONFIG_FILE" "nFeatures.*:[[:space:]]*[0-9]*0[0-9]*" "特征点数量在推荐范围(800-1500)"
    check_string_warn "$CONFIG_FILE" "nLevels.*:[[:space:]]*[67]" "金字塔层数在推荐范围(6-7)"
else
    echo -e "${YELLOW}⚠${NC} 未找到配置文件"
    WARNINGS=$((WARNINGS + 1))
fi
echo ""

echo "8. 检查文档..."
echo "--------------"
check_file "PERFORMANCE_OPTIMIZATION.md"
check_file "TUNING_GUIDE.md"
check_file "OPTIMIZATION_SUMMARY.md"
echo ""

# 总结
echo "========================================"
echo "  验证结果"
echo "========================================"
echo -e "${GREEN}成功: $SUCCESS${NC}"
echo -e "${YELLOW}警告: $WARNINGS${NC}"
echo -e "${RED}错误: $ERRORS${NC}"
echo ""

if [ $ERRORS -eq 0 ]; then
    echo -e "${GREEN}✓ 优化已正确应用!${NC}"
    echo ""
    echo "建议的下一步:"
    echo "1. 运行性能测试: ./performance_test.sh /path/to/dataset"
    echo "2. 查看详细文档: cat PERFORMANCE_OPTIMIZATION.md"
    echo "3. 调优参数: cat TUNING_GUIDE.md"
    echo ""
    exit 0
else
    echo -e "${RED}✗ 发现 $ERRORS 个错误,请修复后重试${NC}"
    echo ""
    echo "常见问题解决:"
    echo "1. 如果缺少文件,请确保在正确的目录中"
    echo "2. 如果编译配置有问题,运行: ./build.sh"
    echo "3. 如果优化未应用,请检查代码是否正确修改"
    echo ""
    exit 1
fi
