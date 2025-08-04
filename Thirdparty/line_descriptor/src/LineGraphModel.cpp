#include "LineGraphModel.h"
#include <algorithm>
#include <iostream>
#include <queue>

using namespace line_graph;

// 构造函数
LineGraphModel::LineGraphModel(double prob_threshold, double _logNT) 
    : prob_threshold(prob_threshold), logNT(_logNT) {
    // 初始化NFA查找表
    nfa_lut = new NFALUT(1000, prob_threshold, logNT);
}

// 析构函数
LineGraphModel::~LineGraphModel() {
    if (nfa_lut) {
        delete nfa_lut;
    }
}

// 构建概率图
void LineGraphModel::buildGraph(const std::vector<LineSegment>& lines) {
    // 清空现有图
    nodes.clear();
    edges.clear();
    node_id_to_index.clear();
    
    // 添加所有线段作为节点
    for (const auto& line : lines) {
        addNode(line);
    }
    
    // 计算节点概率
    computeNodeProbabilities();
    
    // 构建边连接
    for (int i = 0; i < nodes.size(); ++i) {
        for (int j = i + 1; j < nodes.size(); ++j) {
            // 检查是否需要连接
            if (areLinesConnected(nodes[i].line, nodes[j].line)) {
                addEdge(nodes[i].line.id, nodes[j].line.id);
            }
        }
    }
    
    // 计算边权重
    computeEdgeWeights();
    
    // 移除弱边
    removeWeakEdges(0.1);
}

// 添加节点
void LineGraphModel::addNode(const LineSegment& line) {
    GraphNode node(line);
    node.line.confidence = computeLineConfidence(line);
    nodes.push_back(node);
    node_id_to_index[line.id] = nodes.size() - 1;
}

// 添加边
void LineGraphModel::addEdge(int node1_id, int node2_id) {
    // 检查边是否已存在
    for (const auto& edge : edges) {
        if ((edge.node1_id == node1_id && edge.node2_id == node2_id) ||
            (edge.node1_id == node2_id && edge.node2_id == node1_id)) {
            return;
        }
    }
    
    GraphEdge edge(node1_id, node2_id);
    edges.push_back(edge);
    
    // 更新邻居关系
    auto it1 = node_id_to_index.find(node1_id);
    auto it2 = node_id_to_index.find(node2_id);
    if (it1 != node_id_to_index.end() && it2 != node_id_to_index.end()) {
        nodes[it1->second].neighbors.push_back(node2_id);
        nodes[it2->second].neighbors.push_back(node1_id);
    }
}

// 计算节点概率
void LineGraphModel::computeNodeProbabilities() {
    for (auto& node : nodes) {
        // 基于NFA计算置信度
        double nfa_value = -log10(node.line.confidence) - logNT;
        node.node_probability = std::max(0.0, 1.0 - exp(-nfa_value));
    }
}

// 计算边权重
void LineGraphModel::computeEdgeWeights() {
    for (auto& edge : edges) {
        auto it1 = node_id_to_index.find(edge.node1_id);
        auto it2 = node_id_to_index.find(edge.node2_id);
        
        if (it1 != node_id_to_index.end() && it2 != node_id_to_index.end()) {
            const LineSegment& line1 = nodes[it1->second].line;
            const LineSegment& line2 = nodes[it2->second].line;
            
            // 计算空间权重
            edge.spatial_weight = computeSpatialWeight(line1, line2);
            
            // 计算几何权重
            edge.geometric_weight = computeGeometricWeight(line1, line2);
            
            // 综合权重
            edge.weight = 0.6 * edge.spatial_weight + 0.4 * edge.geometric_weight;
        }
    }
}

// 计算线段置信度
double LineGraphModel::computeLineConfidence(const LineSegment& line) {
    // 基于线段长度和NFA计算置信度
    int n = static_cast<int>(line.length);  // 使用长度作为样本数
    int k = static_cast<int>(line.length * 0.8);  // 假设80%的点是有效的
    
    if (nfa_lut->checkValidationByNFA(n, k)) {
        return 1.0 - prob_threshold;
    } else {
        return prob_threshold;
    }
}

// 计算空间权重
double LineGraphModel::computeSpatialWeight(const LineSegment& line1, const LineSegment& line2) {
    double proximity = computeProximity(line1, line2);
    double collinearity = computeCollinearity(line1, line2);
    
    // 归一化到[0,1]
    proximity = std::max(0.0, 1.0 - proximity / PROXIMITY_THRESHOLD);
    collinearity = std::max(0.0, 1.0 - collinearity / COLLINEARITY_THRESHOLD);
    
    return 0.7 * proximity + 0.3 * collinearity;
}

// 计算几何权重
double LineGraphModel::computeGeometricWeight(const LineSegment& line1, const LineSegment& line2) {
    double parallelism = computeParallelism(line1, line2);
    double perpendicularity = computePerpendicularity(line1, line2);
    
    // 选择更强的几何关系
    return std::max(parallelism, perpendicularity);
}

// 计算平行度
double LineGraphModel::computeParallelism(const LineSegment& line1, const LineSegment& line2) {
    double angle_diff = std::abs(line1.angle - line2.angle);
    angle_diff = std::min(angle_diff, M_PI - angle_diff);  // 考虑角度的周期性
    
    if (angle_diff < PARALLEL_THRESHOLD) {
        return 1.0 - angle_diff / PARALLEL_THRESHOLD;
    }
    return 0.0;
}

// 计算垂直度
double LineGraphModel::computePerpendicularity(const LineSegment& line1, const LineSegment& line2) {
    double angle_diff = std::abs(line1.angle - line2.angle);
    angle_diff = std::min(angle_diff, M_PI - angle_diff);
    
    double perp_diff = std::abs(angle_diff - M_PI / 2);
    if (perp_diff < PERPENDICULAR_THRESHOLD) {
        return 1.0 - perp_diff / PERPENDICULAR_THRESHOLD;
    }
    return 0.0;
}

// 计算接近度
double LineGraphModel::computeProximity(const LineSegment& line1, const LineSegment& line2) {
    cv::Point2f center1 = line1.getCenter();
    cv::Point2f center2 = line2.getCenter();
    
    double center_dist = cv::norm(center1 - center2);
    
    // 计算点到线段的距离
    double dist1 = pointToLineDistance(center1, line2);
    double dist2 = pointToLineDistance(center2, line1);
    
    return std::min(center_dist, std::min(dist1, dist2));
}

// 计算共线性
double LineGraphModel::computeCollinearity(const LineSegment& line1, const LineSegment& line2) {
    // 检查两条线段是否在同一直线上
    double dist1 = pointToLineDistance(line1.start, line2);
    double dist2 = pointToLineDistance(line1.end, line2);
    double dist3 = pointToLineDistance(line2.start, line1);
    double dist4 = pointToLineDistance(line2.end, line1);
    
    return std::min({dist1, dist2, dist3, dist4});
}

// 计算点到线段的距离
double LineGraphModel::pointToLineDistance(const cv::Point2f& point, const LineSegment& line) {
    cv::Point2f line_vec = line.end - line.start;
    cv::Point2f point_vec = point - line.start;
    
    double line_length_sq = line_vec.x * line_vec.x + line_vec.y * line_vec.y;
    if (line_length_sq == 0) return cv::norm(point_vec);
    
    double t = std::max(0.0, std::min(1.0, 
        (point_vec.x * line_vec.x + point_vec.y * line_vec.y) / line_length_sq));
    
    cv::Point2f projection = line.start + t * line_vec;
    return cv::norm(point - projection);
}

// 检查线段是否连接
bool LineGraphModel::areLinesConnected(const LineSegment& line1, const LineSegment& line2, double threshold) {
    // 检查端点是否接近
    double dist1 = cv::norm(line1.start - line2.start);
    double dist2 = cv::norm(line1.start - line2.end);
    double dist3 = cv::norm(line1.end - line2.start);
    double dist4 = cv::norm(line1.end - line2.end);
    
    double min_dist = std::min({dist1, dist2, dist3, dist4});
    
    // 检查几何关系
    double parallelism = computeParallelism(line1, line2);
    double perpendicularity = computePerpendicularity(line1, line2);
    
    return min_dist < threshold && (parallelism > 0.3 || perpendicularity > 0.3);
}

// 移除弱边
void LineGraphModel::removeWeakEdges(double threshold) {
    auto it = edges.begin();
    while (it != edges.end()) {
        if (it->weight < threshold) {
            // 从邻居列表中移除
            auto node1_it = node_id_to_index.find(it->node1_id);
            auto node2_it = node_id_to_index.find(it->node2_id);
            
            if (node1_it != node_id_to_index.end()) {
                auto& neighbors = nodes[node1_it->second].neighbors;
                neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), it->node2_id), neighbors.end());
            }
            
            if (node2_it != node_id_to_index.end()) {
                auto& neighbors = nodes[node2_it->second].neighbors;
                neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), it->node1_id), neighbors.end());
            }
            
            it = edges.erase(it);
        } else {
            ++it;
        }
    }
}

// 获取显著性线段
std::vector<LineSegment> LineGraphModel::getSignificantLines(double threshold) {
    std::vector<LineSegment> significant_lines;
    
    for (const auto& node : nodes) {
        if (node.node_probability > threshold) {
            significant_lines.push_back(node.line);
        }
    }
    
    // 按置信度排序
    std::sort(significant_lines.begin(), significant_lines.end(),
        [](const LineSegment& a, const LineSegment& b) {
            return a.confidence > b.confidence;
        });
    
    return significant_lines;
}

// 计算联合概率
double LineGraphModel::computeJointProbability(int node1_idx, int node2_idx) {
    if (node1_idx < 0 || node1_idx >= nodes.size() || 
        node2_idx < 0 || node2_idx >= nodes.size()) {
        return 0.0;
    }
    
    double p1 = nodes[node1_idx].node_probability;
    double p2 = nodes[node2_idx].node_probability;
    
    // 查找连接权重
    double edge_weight = 0.0;
    for (const auto& edge : edges) {
        if ((edge.node1_id == nodes[node1_idx].line.id && 
             edge.node2_id == nodes[node2_idx].line.id) ||
            (edge.node1_id == nodes[node2_idx].line.id && 
             edge.node2_id == nodes[node1_idx].line.id)) {
            edge_weight = edge.weight;
            break;
        }
    }
    
    // 使用边权重调整联合概率
    return p1 * p2 * (0.5 + 0.5 * edge_weight);
}

// 角度归一化
double LineGraphModel::normalizeAngle(double angle) {
    while (angle < 0) angle += M_PI;
    while (angle >= M_PI) angle -= M_PI;
    return angle;
}

// ==================== 自适应线特征提取实现 ====================

// 主要的自适应线特征提取函数
std::vector<LineSegment> LineGraphModel::adaptiveLineExtraction(const cv::Mat& image, 
                                                               const std::vector<cv::KeyPoint>& orb_keypoints,
                                                               int quadtree_depth,
                                                               int min_points_threshold,
                                                               double line_response_threshold) {
    std::vector<LineSegment> adaptive_lines;
    
    // 1. 构建四叉树分析特征点分布
    std::vector<QuadTreeNode> quad_nodes;
    buildQuadTree(image, orb_keypoints, quad_nodes, quadtree_depth, min_points_threshold);
    
    // 2. 对每个稀疏区域提取线特征
    for (const auto& node : quad_nodes) {
        if (node.is_sparse && node.region.area() > 100) { // 只处理足够大的稀疏区域
            // 在该区域提取线特征
            cv::Mat roi = image(node.region);
            
            // 使用EDLines在ROI中检测线段
            std::vector<LineSegment> region_lines;
            
            // 这里可以调用EDLines检测器
            // 为了演示，我们创建一个简化的线段检测
            // 在实际应用中，这里应该调用EDLines算法
            
            // 3. 过滤高质量线特征
            std::vector<LineSegment> filtered_lines = filterHighQualityLines(region_lines, node.region, line_response_threshold);
            
            // 添加到结果中
            adaptive_lines.insert(adaptive_lines.end(), filtered_lines.begin(), filtered_lines.end());
        }
    }
    
    return adaptive_lines;
}

// 过滤高质量线特征
std::vector<LineSegment> LineGraphModel::filterHighQualityLines(const std::vector<LineSegment>& candidate_lines,
                                                                const cv::Rect& region,
                                                                double quality_threshold) {
    std::vector<LineSegment> high_quality_lines;
    
    for (const auto& line : candidate_lines) {
        // 1. 检查线段是否在指定区域内
        if (!isLineInRegion(line, region)) {
            continue;
        }
        
        // 2. 计算线段置信度
        double confidence = computeLineConfidence(line);
        
        // 3. 基于置信度过滤
        if (confidence > quality_threshold) {
            // 4. 额外的几何约束检查
            if (line.length > 10.0) { // 最小长度约束
                high_quality_lines.push_back(line);
            }
        }
    }
    
    // 5. 按置信度排序，取前N个
    std::sort(high_quality_lines.begin(), high_quality_lines.end(), 
              [](const LineSegment& a, const LineSegment& b) {
                  return a.confidence > b.confidence;
              });
    
    // 限制每个区域最多提取的线特征数量
    int max_lines_per_region = 5;
    if (high_quality_lines.size() > max_lines_per_region) {
        high_quality_lines.resize(max_lines_per_region);
    }
    
    return high_quality_lines;
}

// 构建四叉树
void LineGraphModel::buildQuadTree(const cv::Mat& image, const std::vector<cv::KeyPoint>& keypoints,
                                  std::vector<QuadTreeNode>& quad_nodes, int depth, int min_threshold) {
    quad_nodes.clear();
    
    // 创建根节点
    QuadTreeNode root(cv::Rect(0, 0, image.cols, image.rows));
    
    // 计算根节点中的特征点数量
    for (const auto& kp : keypoints) {
        if (root.region.contains(kp.pt)) {
            root.keypoints.push_back(kp);
            root.point_count++;
        }
    }
    
    std::queue<std::pair<QuadTreeNode, int>> node_queue;
    node_queue.push({root, 0});
    
    while (!node_queue.empty()) {
        QuadTreeNode current_node = node_queue.front().first;
        int current_depth = node_queue.front().second;
        node_queue.pop();
        
        // 检查是否为稀疏区域
        current_node.is_sparse = (current_node.point_count < min_threshold);
        
        // 如果达到最大深度或区域太小，添加为叶节点
        if (current_depth >= depth || current_node.region.width < 32 || current_node.region.height < 32) {
            quad_nodes.push_back(current_node);
            continue;
        }
        
        // 如果不是稀疏区域且可以继续分割，则进行四分
        if (!current_node.is_sparse) {
            std::vector<QuadTreeNode> sub_nodes;
            subdivideQuadNode(current_node, sub_nodes, keypoints);
            
            for (const auto& sub_node : sub_nodes) {
                node_queue.push({sub_node, current_depth + 1});
            }
        } else {
            quad_nodes.push_back(current_node);
        }
    }
}

// 四叉树节点细分
void LineGraphModel::subdivideQuadNode(const QuadTreeNode& parent, std::vector<QuadTreeNode>& nodes,
                                      const std::vector<cv::KeyPoint>& all_keypoints) {
    int half_w = parent.region.width / 2;
    int half_h = parent.region.height / 2;
    int x = parent.region.x;
    int y = parent.region.y;
    
    // 创建四个子区域
    std::vector<cv::Rect> sub_regions = {
        cv::Rect(x, y, half_w, half_h),                           // 左上
        cv::Rect(x + half_w, y, half_w, half_h),                  // 右上
        cv::Rect(x, y + half_h, half_w, half_h),                  // 左下
        cv::Rect(x + half_w, y + half_h, half_w, half_h)          // 右下
    };
    
    for (const auto& region : sub_regions) {
        QuadTreeNode sub_node(region);
        
        // 统计该区域内的特征点
        for (const auto& kp : all_keypoints) {
            if (region.contains(kp.pt)) {
                sub_node.keypoints.push_back(kp);
                sub_node.point_count++;
            }
        }
        
        nodes.push_back(sub_node);
    }
}

// 计算线段响应度
double LineGraphModel::computeLineResponse(const LineSegment& line, const cv::Mat& image) {
    // 简化的线段响应度计算
    // 在实际应用中，这应该基于梯度强度等更复杂的特征
    
    // 沿线段采样点，计算梯度强度
    cv::Point2f direction = line.end - line.start;
    float length = cv::norm(direction);
    direction /= length;
    
    double total_response = 0.0;
    int sample_count = static_cast<int>(length / 2.0); // 每2像素采样一次
    sample_count = std::max(5, std::min(sample_count, 50)); // 限制采样点数量
    
    for (int i = 0; i < sample_count; ++i) {
        float t = static_cast<float>(i) / sample_count;
        cv::Point2f sample_pt = line.start + t * (line.end - line.start);
        
        // 确保采样点在图像范围内
        if (sample_pt.x >= 1 && sample_pt.x < image.cols-1 && 
            sample_pt.y >= 1 && sample_pt.y < image.rows-1) {
            
            // 计算该点的梯度强度（简化版本）
            if (image.type() == CV_8UC1) {
                const uchar* row = image.ptr<uchar>(static_cast<int>(sample_pt.y));
                int grad_x = abs(row[static_cast<int>(sample_pt.x+1)] - row[static_cast<int>(sample_pt.x-1)]);
                
                const uchar* next_row = image.ptr<uchar>(static_cast<int>(sample_pt.y+1));
                const uchar* prev_row = image.ptr<uchar>(static_cast<int>(sample_pt.y-1));
                int grad_y = abs(next_row[static_cast<int>(sample_pt.x)] - prev_row[static_cast<int>(sample_pt.x)]);
                
                total_response += sqrt(grad_x * grad_x + grad_y * grad_y);
            }
        }
    }
    
    return total_response / sample_count;
}

// 检查线段是否在区域内
bool LineGraphModel::isLineInRegion(const LineSegment& line, const cv::Rect& region) {
    // 检查线段的起点和终点是否都在区域内
    return region.contains(line.start) && region.contains(line.end);
} 