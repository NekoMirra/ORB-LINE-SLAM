#include "LineGraphModel.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <random>

using namespace line_graph;

// 生成测试线段数据
std::vector<LineSegment> generateTestLines(int num_lines = 50) {
    std::vector<LineSegment> lines;
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // 图像尺寸
    const int width = 800;
    const int height = 600;
    
    // 生成随机线段
    std::uniform_real_distribution<> x_dist(0, width);
    std::uniform_real_distribution<> y_dist(0, height);
    std::uniform_real_distribution<> length_dist(20, 100);
    std::uniform_real_distribution<> angle_dist(0, M_PI);
    
    for (int i = 0; i < num_lines; ++i) {
        cv::Point2f start(x_dist(gen), y_dist(gen));
        double angle = angle_dist(gen);
        double length = length_dist(gen);
        
        cv::Point2f end(
            start.x + length * cos(angle),
            start.y + length * sin(angle)
        );
        
        // 确保线段在图像范围内
        if (end.x >= 0 && end.x < width && end.y >= 0 && end.y < height) {
            lines.emplace_back(start, end, i);
        }
    }
    
    return lines;
}

// 可视化概率图
void visualizeGraph(const LineGraphModel& model, const std::string& window_name = "Line Graph") {
    cv::Mat image(600, 800, CV_8UC3, cv::Scalar(255, 255, 255));
    
    const auto& nodes = model.getNodes();
    const auto& edges = model.getEdges();
    
    // 绘制边
    for (const auto& edge : edges) {
        // 找到对应的节点
        int node1_idx = -1, node2_idx = -1;
        for (int i = 0; i < nodes.size(); ++i) {
            if (nodes[i].line.id == edge.node1_id) node1_idx = i;
            if (nodes[i].line.id == edge.node2_id) node2_idx = i;
        }
        
        if (node1_idx >= 0 && node2_idx >= 0) {
            cv::Point2f center1 = nodes[node1_idx].line.getCenter();
            cv::Point2f center2 = nodes[node2_idx].line.getCenter();
            
            // 根据权重设置颜色
            int intensity = static_cast<int>(255 * edge.weight);
            cv::line(image, center1, center2, cv::Scalar(0, intensity, 255 - intensity), 1);
        }
    }
    
    // 绘制节点（线段）
    for (const auto& node : nodes) {
        const auto& line = node.line;
        
        // 根据概率设置颜色
        int intensity = static_cast<int>(255 * node.node_probability);
        cv::Scalar color(255 - intensity, intensity, 0);  // 红色=低概率，绿色=高概率
        
        cv::line(image, line.start, line.end, color, 2);
        
        // 绘制线段中心点
        cv::Point2f center = line.getCenter();
        cv::circle(image, center, 3, cv::Scalar(0, 0, 255), -1);
    }
    
    // 添加图例
    cv::putText(image, "Red: Low probability", cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
    cv::putText(image, "Green: High probability", cv::Point(10, 60), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    cv::putText(image, "Blue: Edge weight", cv::Point(10, 90), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
    
    cv::imshow(window_name, image);
    cv::waitKey(0);
}

// 分析图结构
void analyzeGraph(const LineGraphModel& model) {
    const auto& nodes = model.getNodes();
    const auto& edges = model.getEdges();
    
    std::cout << "=== 概率图分析 ===" << std::endl;
    std::cout << "节点数量: " << model.getNodeCount() << std::endl;
    std::cout << "边数量: " << model.getEdgeCount() << std::endl;
    
    // 计算平均节点概率
    double avg_prob = 0.0;
    for (const auto& node : nodes) {
        avg_prob += node.node_probability;
    }
    avg_prob /= nodes.size();
    std::cout << "平均节点概率: " << avg_prob << std::endl;
    
    // 计算平均边权重
    double avg_weight = 0.0;
    for (const auto& edge : edges) {
        avg_weight += edge.weight;
    }
    if (!edges.empty()) {
        avg_weight /= edges.size();
    }
    std::cout << "平均边权重: " << avg_weight << std::endl;
    
    // 找到高概率节点
    std::vector<std::pair<int, double>> high_prob_nodes;
    for (const auto& node : nodes) {
        if (node.node_probability > 0.7) {
            high_prob_nodes.emplace_back(node.line.id, node.node_probability);
        }
    }
    
    std::cout << "高概率节点数量 (>0.7): " << high_prob_nodes.size() << std::endl;
    for (const auto& pair : high_prob_nodes) {
        std::cout << "  节点 " << pair.first << ": 概率 = " << pair.second << std::endl;
    }
    
    // 找到强连接边
    std::vector<std::pair<std::pair<int, int>, double>> strong_edges;
    for (const auto& edge : edges) {
        if (edge.weight > 0.8) {
            strong_edges.emplace_back(std::make_pair(edge.node1_id, edge.node2_id), edge.weight);
        }
    }
    
    std::cout << "强连接边数量 (>0.8): " << strong_edges.size() << std::endl;
    for (const auto& pair : strong_edges) {
        std::cout << "  边 (" << pair.first.first << ", " << pair.first.second 
                  << "): 权重 = " << pair.second << std::endl;
    }
}

int main() {
    std::cout << "=== 线段概率图建模示例 ===" << std::endl;
    
    // 1. 生成测试数据
    std::cout << "生成测试线段..." << std::endl;
    auto test_lines = generateTestLines(30);
    std::cout << "生成了 " << test_lines.size() << " 条测试线段" << std::endl;
    
    // 2. 创建概率图模型
    std::cout << "构建概率图..." << std::endl;
    LineGraphModel model(0.01, 10.0);  // prob_threshold=0.01, logNT=10.0
    model.buildGraph(test_lines);
    
    // 3. 分析图结构
    analyzeGraph(model);
    
    // 4. 获取显著性线段
    std::cout << "\n获取显著性线段..." << std::endl;
    auto significant_lines = model.getSignificantLines(0.5);
    std::cout << "显著性线段数量 (阈值=0.5): " << significant_lines.size() << std::endl;
    
    // 5. 可视化结果
    std::cout << "可视化概率图..." << std::endl;
    visualizeGraph(model, "Line Probability Graph");
    
    // 6. 演示联合概率计算
    std::cout << "\n计算联合概率示例..." << std::endl;
    const auto& nodes = model.getNodes();
    if (nodes.size() >= 2) {
        double joint_prob = model.computeJointProbability(0, 1);
        std::cout << "节点 0 和节点 1 的联合概率: " << joint_prob << std::endl;
    }
    
    std::cout << "\n程序完成！按任意键退出..." << std::endl;
    cv::waitKey(0);
    
    return 0;
} 