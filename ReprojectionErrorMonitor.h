/**
* This file is part of ORB-LINE-SLAM
*
* Copyright (C) 2020-2021 John Alamanos, National Technical University of Athens.
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-LINE-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-LINE-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or PARTICULAR PURPOSE. See the GNU General Public
* License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-LINE-SLAM.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef REPROJECTIONERRORMONITOR_H
#define REPROJECTIONERRORMONITOR_H

#include <vector>
#include <deque>
#include <mutex>
#include <chrono>
#include <memory>
#include <atomic>

#include "KeyFrame.h"
#include "Map.h"
#include "Frame.h"

namespace ORB_SLAM3
{

class Optimizer;

/**
 * @brief 重投影误差动态分析模型
 * 
 * 该类负责实时监控重投影误差的变化，通过分析误差的变化率和持续时间
 * 来判断是否需要进行局部重优化。当检测到重投影误差异常增大时，触发优化机制。
 */
class ReprojectionErrorMonitor
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     * @brief 构造函数
     * @param pMap 地图指针
     * @param windowSize 滑动窗口大小，用于计算误差变化率
     * @param errorThreshold 误差阈值，超过此值认为异常
     * @param changeRateThreshold 变化率阈值，超过此值认为异常
     * @param durationThreshold 持续时间阈值（秒），异常持续超过此时间触发优化
     */
    ReprojectionErrorMonitor(Map* pMap, 
                            size_t windowSize = 20,
                            double errorThreshold = 2.0,
                            double changeRateThreshold = 0.5,
                            double durationThreshold = 1.0);

    /**
     * @brief 析构函数
     */
    ~ReprojectionErrorMonitor();

    /**
     * @brief 更新重投影误差
     * @param pFrame 当前帧
     * @param reprojectionError 当前帧的重投影误差
     * @return 是否需要触发局部重优化
     */
    bool UpdateReprojectionError(Frame* pFrame, double reprojectionError);

    /**
     * @brief 计算重投影误差
     * @param pFrame 帧指针
     * @return 重投影误差值
     */
    double ComputeReprojectionError(Frame* pFrame);

    /**
     * @brief 检查是否需要触发局部重优化
     * @return 是否需要触发优化
     */
    bool ShouldTriggerOptimization();

    /**
     * @brief 获取当前误差统计信息
     * @return 包含误差统计信息的结构体
     */
    struct ErrorStats {
        double currentError;
        double averageError;
        double changeRate;
        double maxError;
        double minError;
        size_t errorCount;
        double lastUpdateTime;
    };
    
    ErrorStats GetErrorStats() const;

    /**
     * @brief 重置监控器状态
     */
    void Reset();

    /**
     * @brief 设置参数
     * @param windowSize 滑动窗口大小
     * @param errorThreshold 误差阈值
     * @param changeRateThreshold 变化率阈值
     * @param durationThreshold 持续时间阈值
     */
    void SetParameters(size_t windowSize, double errorThreshold, 
                      double changeRateThreshold, double durationThreshold);

    /**
     * @brief 启用/禁用自适应参数调整
     * @param enable 是否启用自适应调整
     */
    void EnableAdaptiveAdjustment(bool enable);

    /**
     * @brief 获取优化历史记录
     * @return 优化历史记录列表
     */
    std::vector<OptimizationRecord> GetOptimizationHistory() const;

    /**
     * @brief 获取自适应参数
     * @return 当前自适应参数
     */
    AdaptiveParams GetAdaptiveParams() const;

    /**
     * @brief 记录优化结果
     * @param record 优化记录
     */
    void RecordOptimizationResult(const OptimizationRecord& record);

private:
    /**
     * @brief 计算误差变化率
     * @return 误差变化率
     */
    double ComputeErrorChangeRate();

    /**
     * @brief 检查异常持续时间
     * @return 异常持续时间（秒）
     */
    double CheckAbnormalDuration();

    /**
     * @brief 更新滑动窗口
     * @param error 新的误差值
     */
    void UpdateSlidingWindow(double error);

    /**
     * @brief 计算滑动窗口内的统计信息
     */
    void ComputeWindowStatistics();

    /**
     * @brief 自适应调整参数
     */
    void AdjustParametersAdaptively();

    /**
     * @brief 验证优化结果
     * @param preError 优化前误差
     * @param postError 优化后误差
     * @param poseChanges 位姿变化
     * @return 验证结果
     */
    bool ValidateOptimizationResult(double preError, double postError, 
                                  const std::vector<cv::Mat>& poseChanges);

    // 地图指针
    Map* mpMap;

    // 滑动窗口存储误差历史
    std::deque<double> mvErrorHistory;
    
    // 滑动窗口大小
    size_t mWindowSize;
    
    // 误差阈值
    double mErrorThreshold;
    
    // 变化率阈值
    double mChangeRateThreshold;
    
    // 持续时间阈值（秒）
    double mDurationThreshold;
    
    // 异常开始时间
    std::chrono::steady_clock::time_point mAbnormalStartTime;
    
    // 是否处于异常状态
    std::atomic<bool> mbIsAbnormal;
    
    // 当前误差统计信息
    ErrorStats mErrorStats;
    
    // 互斥锁保护数据访问
    mutable std::mutex mMutex;
    
    // 优化器指针，用于触发局部重优化
    Optimizer* mpOptimizer;
    
    // 上次更新时间
    std::chrono::steady_clock::time_point mLastUpdateTime;

    // 优化历史记录
    struct OptimizationRecord {
        double timestamp;
        double preError;
        double postError;
        double improvement;
        bool converged;
        int iterations;
        double duration;
        std::vector<KeyFrame*> involvedKFs;
    };
    
    std::deque<OptimizationRecord> mvOptimizationHistory;
    size_t mMaxHistorySize;
    
    // 自适应参数调整
    struct AdaptiveParams {
        double errorThreshold;
        double changeRateThreshold;
        double durationThreshold;
        int maxIterations;
    };
    
    AdaptiveParams mAdaptiveParams;
    bool mbEnableAdaptiveAdjustment;
};

} // namespace ORB_SLAM3

#endif // REPROJECTIONERRORMONITOR_H
