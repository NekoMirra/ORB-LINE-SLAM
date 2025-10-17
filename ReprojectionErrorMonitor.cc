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

#include "ReprojectionErrorMonitor.h"
#include "Optimizer.h"
#include "MapPoint.h"
#include "MapLine.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "Converter.h"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <algorithm>
#include <numeric>
#include <cmath>

namespace ORB_SLAM3
{

ReprojectionErrorMonitor::ReprojectionErrorMonitor(Map* pMap, 
                                                 size_t windowSize,
                                                 double errorThreshold,
                                                 double changeRateThreshold,
                                                 double durationThreshold)
    : mpMap(pMap)
    , mWindowSize(windowSize)
    , mErrorThreshold(errorThreshold)
    , mChangeRateThreshold(changeRateThreshold)
    , mDurationThreshold(durationThreshold)
    , mbIsAbnormal(false)
    , mpOptimizer(nullptr)
    , mMaxHistorySize(100)
    , mbEnableAdaptiveAdjustment(true)
{
    // 初始化误差统计信息
    mErrorStats.currentError = 0.0;
    mErrorStats.averageError = 0.0;
    mErrorStats.changeRate = 0.0;
    mErrorStats.maxError = 0.0;
    mErrorStats.minError = std::numeric_limits<double>::max();
    mErrorStats.errorCount = 0;
    mErrorStats.lastUpdateTime = 0.0;
    
    // 初始化自适应参数
    mAdaptiveParams.errorThreshold = errorThreshold;
    mAdaptiveParams.changeRateThreshold = changeRateThreshold;
    mAdaptiveParams.durationThreshold = durationThreshold;
    mAdaptiveParams.maxIterations = 3;
    
    mLastUpdateTime = std::chrono::steady_clock::now();
    mAbnormalStartTime = mLastUpdateTime;
}

ReprojectionErrorMonitor::~ReprojectionErrorMonitor()
{
}

bool ReprojectionErrorMonitor::UpdateReprojectionError(Frame* pFrame, double reprojectionError)
{
    std::lock_guard<std::mutex> lock(mMutex);
    
    // 更新滑动窗口
    UpdateSlidingWindow(reprojectionError);
    
    // 计算统计信息
    ComputeWindowStatistics();
    
    // 检查是否需要触发优化
    bool shouldOptimize = ShouldTriggerOptimization();
    
    // 更新异常状态
    if (reprojectionError > mErrorThreshold) {
        if (!mbIsAbnormal) {
            mAbnormalStartTime = std::chrono::steady_clock::now();
            mbIsAbnormal = true;
        }
    } else {
        mbIsAbnormal = false;
    }
    
    // 更新时间
    mLastUpdateTime = std::chrono::steady_clock::now();
    
    return shouldOptimize;
}

double ReprojectionErrorMonitor::ComputeReprojectionError(Frame* pFrame)
{
    if (!pFrame || !mpMap) {
        return 0.0;
    }
    
    double totalError = 0.0;
    int validPoints = 0;
    
    // 计算点的重投影误差
    for (int i = 0; i < pFrame->N; i++) {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if (pMP) {
            if (pMP->Observations() > 0) {
                // 获取3D点坐标
                cv::Mat p3D = pMP->GetWorldPos();
                
                // 获取观测点
                cv::Point2f observed = pFrame->mvKeysUn[i].pt;
                
                // 计算重投影误差
                cv::Mat p3D_cam = pFrame->mTcw * p3D;
                if (p3D_cam.at<float>(2) > 0) {  // 点在相机前方
                    // 计算重投影点
                    float x = p3D_cam.at<float>(0) / p3D_cam.at<float>(2);
                    float y = p3D_cam.at<float>(1) / p3D_cam.at<float>(2);
                    
                    // 计算重投影误差
                    float dx = observed.x - x;
                    float dy = observed.y - y;
                    double error = sqrt(dx*dx + dy*dy);
                    totalError += error;
                    validPoints++;
                }
            }
        }
    }
    
    // 计算线的重投影误差（如果有的话）
    for (int i = 0; i < pFrame->Nlines; i++) {
        MapLine* pML = pFrame->mvpMapLines[i];
        if (pML) {
            if (pML->Observations() > 0) {
                // 获取3D线坐标
                cv::Mat p3D_start = pML->GetWorldPosStart();
                cv::Mat p3D_end = pML->GetWorldPosEnd();
                
                // 计算线的重投影误差（简化处理）
                double error = 0.0;
                // 这里可以添加更复杂的线重投影误差计算
                totalError += error;
                validPoints++;
            }
        }
    }
    
    if (validPoints > 0) {
        return totalError / validPoints;
    }
    
    return 0.0;
}

bool ReprojectionErrorMonitor::ShouldTriggerOptimization()
{
    // 检查误差是否超过阈值
    if (mErrorStats.currentError <= mErrorThreshold) {
        return false;
    }
    
    // 检查变化率是否超过阈值
    if (mErrorStats.changeRate <= mChangeRateThreshold) {
        return false;
    }
    
    // 检查异常持续时间
    double abnormalDuration = CheckAbnormalDuration();
    if (abnormalDuration < mDurationThreshold) {
        return false;
    }
    
    return true;
}

ReprojectionErrorMonitor::ErrorStats ReprojectionErrorMonitor::GetErrorStats() const
{
    std::lock_guard<std::mutex> lock(mMutex);
    return mErrorStats;
}

void ReprojectionErrorMonitor::Reset()
{
    std::lock_guard<std::mutex> lock(mMutex);
    
    mvErrorHistory.clear();
    mbIsAbnormal = false;
    
    mErrorStats.currentError = 0.0;
    mErrorStats.averageError = 0.0;
    mErrorStats.changeRate = 0.0;
    mErrorStats.maxError = 0.0;
    mErrorStats.minError = std::numeric_limits<double>::max();
    mErrorStats.errorCount = 0;
    mErrorStats.lastUpdateTime = 0.0;
    
    mLastUpdateTime = std::chrono::steady_clock::now();
    mAbnormalStartTime = mLastUpdateTime;
}

void ReprojectionErrorMonitor::SetParameters(size_t windowSize, double errorThreshold, 
                                           double changeRateThreshold, double durationThreshold)
{
    std::lock_guard<std::mutex> lock(mMutex);
    
    mWindowSize = windowSize;
    mErrorThreshold = errorThreshold;
    mChangeRateThreshold = changeRateThreshold;
    mDurationThreshold = durationThreshold;
    
    // 如果新的窗口大小小于当前历史记录大小，需要裁剪
    while (mvErrorHistory.size() > mWindowSize) {
        mvErrorHistory.pop_front();
    }
}

double ReprojectionErrorMonitor::ComputeErrorChangeRate()
{
    if (mvErrorHistory.size() < 2) {
        return 0.0;
    }
    
    // 计算最近几个误差值的变化率
    size_t numSamples = std::min(mvErrorHistory.size(), size_t(5));
    std::vector<double> recentErrors(mvErrorHistory.end() - numSamples, mvErrorHistory.end());
    
    double changeRate = 0.0;
    for (size_t i = 1; i < recentErrors.size(); i++) {
        changeRate += std::abs(recentErrors[i] - recentErrors[i-1]);
    }
    
    return changeRate / (recentErrors.size() - 1);
}

double ReprojectionErrorMonitor::CheckAbnormalDuration()
{
    if (!mbIsAbnormal) {
        return 0.0;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - mAbnormalStartTime);
    return duration.count() / 1000.0;  // 转换为秒
}

void ReprojectionErrorMonitor::UpdateSlidingWindow(double error)
{
    mvErrorHistory.push_back(error);
    
    // 保持滑动窗口大小
    if (mvErrorHistory.size() > mWindowSize) {
        mvErrorHistory.pop_front();
    }
}

void ReprojectionErrorMonitor::ComputeWindowStatistics()
{
    if (mvErrorHistory.empty()) {
        return;
    }
    
    // 当前误差
    mErrorStats.currentError = mvErrorHistory.back();
    
    // 平均误差
    double sum = std::accumulate(mvErrorHistory.begin(), mvErrorHistory.end(), 0.0);
    mErrorStats.averageError = sum / mvErrorHistory.size();
    
    // 最大和最小误差
    auto minmax = std::minmax_element(mvErrorHistory.begin(), mvErrorHistory.end());
    mErrorStats.minError = *minmax.first;
    mErrorStats.maxError = *minmax.second;
    
    // 误差计数
    mErrorStats.errorCount = mvErrorHistory.size();
    
    // 变化率
    mErrorStats.changeRate = ComputeErrorChangeRate();
    
    // 更新时间
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - mLastUpdateTime);
    mErrorStats.lastUpdateTime = duration.count() / 1000.0;
}

void ReprojectionErrorMonitor::EnableAdaptiveAdjustment(bool enable)
{
    std::lock_guard<std::mutex> lock(mMutex);
    mbEnableAdaptiveAdjustment = enable;
}

std::vector<ReprojectionErrorMonitor::OptimizationRecord> ReprojectionErrorMonitor::GetOptimizationHistory() const
{
    std::lock_guard<std::mutex> lock(mMutex);
    return std::vector<OptimizationRecord>(mvOptimizationHistory.begin(), mvOptimizationHistory.end());
}

ReprojectionErrorMonitor::AdaptiveParams ReprojectionErrorMonitor::GetAdaptiveParams() const
{
    std::lock_guard<std::mutex> lock(mMutex);
    return mAdaptiveParams;
}

void ReprojectionErrorMonitor::RecordOptimizationResult(const OptimizationRecord& record)
{
    std::lock_guard<std::mutex> lock(mMutex);
    
    mvOptimizationHistory.push_back(record);
    
    // 保持历史记录大小限制
    while (mvOptimizationHistory.size() > mMaxHistorySize) {
        mvOptimizationHistory.pop_front();
    }
    
    // 自适应调整参数
    if (mbEnableAdaptiveAdjustment) {
        AdjustParametersAdaptively();
    }
}

void ReprojectionErrorMonitor::AdjustParametersAdaptively()
{
    if (mvOptimizationHistory.size() < 5) {
        return; // 需要足够的历史数据
    }
    
    // 计算最近优化的成功率
    int recentOptimizations = std::min(10, (int)mvOptimizationHistory.size());
    int successfulOptimizations = 0;
    double totalImprovement = 0.0;
    
    for (int i = mvOptimizationHistory.size() - recentOptimizations; i < mvOptimizationHistory.size(); i++) {
        const auto& record = mvOptimizationHistory[i];
        if (record.converged && record.improvement > 0.1) {
            successfulOptimizations++;
            totalImprovement += record.improvement;
        }
    }
    
    double successRate = (double)successfulOptimizations / recentOptimizations;
    double avgImprovement = totalImprovement / std::max(1, successfulOptimizations);
    
    // 根据成功率调整参数
    if (successRate < 0.3) {
        // 成功率低，放宽触发条件
        mAdaptiveParams.errorThreshold *= 1.1;
        mAdaptiveParams.durationThreshold *= 1.2;
    } else if (successRate > 0.8 && avgImprovement > 0.5) {
        // 成功率高且改善明显，可以更积极地触发优化
        mAdaptiveParams.errorThreshold *= 0.95;
        mAdaptiveParams.durationThreshold *= 0.9;
    }
    
    // 限制参数范围
    mAdaptiveParams.errorThreshold = std::max(1.0, std::min(5.0, mAdaptiveParams.errorThreshold));
    mAdaptiveParams.durationThreshold = std::max(0.5, std::min(3.0, mAdaptiveParams.durationThreshold));
}

bool ReprojectionErrorMonitor::ValidateOptimizationResult(double preError, double postError, 
                                                         const std::vector<cv::Mat>& poseChanges)
{
    // 1. 检查误差是否有改善
    if (postError >= preError) {
        std::cout << "Warning: Optimization did not improve error. Pre: " << preError 
                  << ", Post: " << postError << std::endl;
        return false;
    }
    
    // 2. 检查改善是否足够显著
    double improvement = (preError - postError) / preError;
    if (improvement < 0.05) { // 至少5%的改善
        std::cout << "Warning: Improvement too small: " << improvement * 100 << "%" << std::endl;
        return false;
    }
    
    // 3. 检查位姿变化是否合理
    for (const auto& poseChange : poseChanges) {
        if (poseChange.empty()) continue;
        
        // 检查旋转变化（四元数或旋转矩阵）
        cv::Mat R = poseChange.rowRange(0, 3).colRange(0, 3);
        double rotChange = cv::norm(R - cv::Mat::eye(3, 3, CV_32F));
        if (rotChange > 0.5) { // 旋转变化过大
            std::cout << "Warning: Large rotation change detected: " << rotChange << std::endl;
            return false;
        }
        
        // 检查平移变化
        cv::Mat t = poseChange.rowRange(0, 3).col(3);
        double transChange = cv::norm(t);
        if (transChange > 2.0) { // 平移变化过大（单位：米）
            std::cout << "Warning: Large translation change detected: " << transChange << std::endl;
            return false;
        }
    }
    
    return true;
}

} // namespace ORB_SLAM3
