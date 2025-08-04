#include "LineGraphModel.h"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>

using namespace line_graph;

// 自适应特征提取示例
void demonstrateAdaptiveFeatureExtraction() {
    // 1. 加载测试图像 - 使用EuRoC数据集的真实图像
    cv::Mat image = cv::imread("/home/mirra/桌面/REFIX/mav0/cam0/data/1403636579763555584.png", cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        std::cerr << "无法加载EuRoC图像，尝试加载测试图像..." << std::endl;
        image = cv::imread("test_image.jpg", cv::IMREAD_GRAYSCALE);
        if (image.empty()) {
            std::cerr << "创建模拟图像进行测试..." << std::endl;
            // 创建一个带有线特征的模拟图像
            image = cv::Mat::zeros(480, 640, CV_8UC1);
            
            // 添加一些直线和结构
            cv::line(image, cv::Point(100, 100), cv::Point(500, 120), cv::Scalar(255), 2);
            cv::line(image, cv::Point(200, 200), cv::Point(400, 400), cv::Scalar(255), 2);
            cv::line(image, cv::Point(50, 300), cv::Point(600, 350), cv::Scalar(255), 2);
            cv::rectangle(image, cv::Rect(300, 50, 100, 80), cv::Scalar(255), 2);
            
            // 添加一些噪声和纹理
            cv::Mat noise(image.size(), CV_8UC1);
            cv::randu(noise, 0, 50);
            cv::add(image, noise, image);
        }
    }
    
    // 2. 提取ORB特征点
    cv::Ptr<cv::ORB> orb = cv::ORB::create(1000);
    std::vector<cv::KeyPoint> orb_keypoints;
    cv::Mat orb_descriptors;
    orb->detectAndCompute(image, cv::Mat(), orb_keypoints, orb_descriptors);
    
    std::cout << "检测到 " << orb_keypoints.size() << " 个ORB特征点" << std::endl;
    
    // 3. 创建概率图模型
    LineGraphModel line_model(0.01, 10.0);
    
    // 4. 执行自适应线特征提取（调整参数使其更敏感）
    std::vector<LineSegment> adaptive_lines = line_model.adaptiveLineExtraction(
        image,                    // 输入图像
        orb_keypoints,           // ORB特征点
        3,                       // 四叉树深度
        5,                       // 最小特征点阈值
        0.3                      // 线特征响应度阈值
    );
    
    std::cout << "在特征点稀疏区域提取到 " << adaptive_lines.size() << " 条高质量线特征" << std::endl;
    
    // 5. 可视化结果
    cv::Mat visualization;
    cv::cvtColor(image, visualization, cv::COLOR_GRAY2BGR);
    
    // 绘制ORB特征点（绿色）
    for (const auto& kp : orb_keypoints) {
        cv::circle(visualization, kp.pt, 3, cv::Scalar(0, 255, 0), 1);
    }
    
    // 绘制自适应线特征（红色）
    for (const auto& line : adaptive_lines) {
        cv::line(visualization, line.start, line.end, cv::Scalar(0, 0, 255), 2);
        
        // 显示线特征的置信度
        cv::Point2f center = line.getCenter();
        std::string conf_text = std::to_string(line.confidence).substr(0, 4);
        cv::putText(visualization, conf_text, center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
    }
    
    // 保存结果
    cv::imwrite("adaptive_features_result.jpg", visualization);
    std::cout << "结果已保存到 adaptive_features_result.jpg" << std::endl;
}

// 在SLAM系统中的集成示例
class AdaptiveSLAMFeatureExtractor {
private:
    LineGraphModel line_model;
    cv::Ptr<cv::ORB> orb_detector;
    
public:
    AdaptiveSLAMFeatureExtractor() : line_model(0.01, 10.0) {
        orb_detector = cv::ORB::create(1000);
    }
    
    // 提取混合特征（ORB + 自适应线特征）
    void extractHybridFeatures(const cv::Mat& image, 
                              std::vector<cv::KeyPoint>& keypoints,
                              std::vector<LineSegment>& line_features) {
        
        // 1. 提取ORB特征点
        cv::Mat descriptors;
        orb_detector->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
        
        // 2. 在特征点稀疏区域自适应提取线特征
        line_features = line_model.adaptiveLineExtraction(
            image, keypoints, 
            3,     // 四叉树深度
            8,     // 稀疏阈值（少于8个特征点认为稀疏）
            0.4    // 线特征质量阈值
        );
        
        std::cout << "提取到 " << keypoints.size() << " 个点特征和 " 
                  << line_features.size() << " 个线特征" << std::endl;
    }
    
    // 特征质量评估
    void evaluateFeatureQuality(const std::vector<LineSegment>& lines) {
        double avg_confidence = 0.0;
        double avg_length = 0.0;
        
        for (const auto& line : lines) {
            avg_confidence += line.confidence;
            avg_length += line.length;
        }
        
        if (!lines.empty()) {
            avg_confidence /= lines.size();
            avg_length /= lines.size();
            
            std::cout << "线特征质量统计:" << std::endl;
            std::cout << "  平均置信度: " << avg_confidence << std::endl;
            std::cout << "  平均长度: " << avg_length << " 像素" << std::endl;
        }
    }
};

int main() {
    std::cout << "=== 自适应特征提取演示 ===" << std::endl;
    
    // 基础演示
    demonstrateAdaptiveFeatureExtraction();
    
    std::cout << "\n=== SLAM集成示例 ===" << std::endl;
    
    // SLAM集成示例
    AdaptiveSLAMFeatureExtractor slam_extractor;
    
    // 模拟处理一帧图像
    cv::Mat test_image = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::randu(test_image, 0, 255); // 随机噪声图像用于测试
    
    std::vector<cv::KeyPoint> hybrid_keypoints;
    std::vector<LineSegment> hybrid_lines;
    
    slam_extractor.extractHybridFeatures(test_image, hybrid_keypoints, hybrid_lines);
    slam_extractor.evaluateFeatureQuality(hybrid_lines);
    
    return 0;
}
