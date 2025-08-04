#ifndef _LINE_GRAPH_MODEL_
#define _LINE_GRAPH_MODEL_

#include <vector>
#include <map>
#include <set>
#include <cmath>
#include <opencv2/core.hpp>
#include "NFA.h"

// 线段结构体
struct LineSegment {
    cv::Point2f start;      // 起点
    cv::Point2f end;        // 终点
    double length;          // 长度
    double angle;           // 角度 (弧度)
    double confidence;      // 置信度 (基于NFA)
    int id;                 // 唯一标识符
    
    LineSegment() : length(0), angle(0), confidence(0), id(-1) {}
    LineSegment(const cv::Point2f& s, const cv::Point2f& e, int _id = -1) 
        : start(s), end(e), id(_id) {
        computeProperties();
    }
    
    void computeProperties() {
        cv::Point2f diff = end - start;
        length = cv::norm(diff);
        angle = atan2(diff.y, diff.x);
        // 标准化角度到 [0, π)
        if (angle < 0) angle += M_PI;
    }
    
    cv::Point2f getCenter() const {
        return (start + end) * 0.5f;
    }
};

// 图节点
struct GraphNode {
    LineSegment line;
    std::vector<int> neighbors;  // 邻居节点ID
    double node_probability;     // 节点概率
    
    GraphNode() : node_probability(0.0) {}
    GraphNode(const LineSegment& l) : line(l), node_probability(0.0) {}
};

// 图边
struct GraphEdge {
    int node1_id;
    int node2_id;
    double weight;              // 边权重
    double spatial_weight;      // 空间关系权重
    double geometric_weight;    // 几何特征权重
    
    GraphEdge() : node1_id(-1), node2_id(-1), weight(0.0), 
                  spatial_weight(0.0), geometric_weight(0.0) {}
    GraphEdge(int id1, int id2) : node1_id(id1), node2_id(id2), 
                                  weight(0.0), spatial_weight(0.0), geometric_weight(0.0) {}
};

// 概率图建模类
class LineGraphModel {
public:
    LineGraphModel(double prob_threshold = 0.01, double logNT = 10.0);
    ~LineGraphModel();
    
    // 主要接口
    void buildGraph(const std::vector<LineSegment>& lines);
    void computeNodeProbabilities();
    void computeEdgeWeights();
    std::vector<LineSegment> getSignificantLines(double threshold = 0.5);
    
    // 图操作
    void addNode(const LineSegment& line);
    void addEdge(int node1_id, int node2_id);
    void removeWeakEdges(double threshold = 0.1);
    
    // 获取图信息
    const std::vector<GraphNode>& getNodes() const { return nodes; }
    const std::vector<GraphEdge>& getEdges() const { return edges; }
    int getNodeCount() const { return nodes.size(); }
    int getEdgeCount() const { return edges.size(); }
    
    // 概率计算 (公有方法)
    double computeJointProbability(int node1_idx, int node2_idx);
    
private:
    // 核心成员
    std::vector<GraphNode> nodes;
    std::vector<GraphEdge> edges;
    std::map<int, int> node_id_to_index;  // ID到索引的映射
    
    // NFA相关
    NFALUT* nfa_lut;
    double prob_threshold;
    double logNT;
    
    // 几何计算函数
    double computeSpatialWeight(const LineSegment& line1, const LineSegment& line2);
    double computeGeometricWeight(const LineSegment& line1, const LineSegment& line2);
    double computeParallelism(const LineSegment& line1, const LineSegment& line2);
    double computePerpendicularity(const LineSegment& line1, const LineSegment& line2);
    double computeProximity(const LineSegment& line1, const LineSegment& line2);
    double computeCollinearity(const LineSegment& line1, const LineSegment& line2);
    
    // 概率计算
    double computeLineConfidence(const LineSegment& line);
    
    // 辅助函数
    double normalizeAngle(double angle);
    double pointToLineDistance(const cv::Point2f& point, const LineSegment& line);
    bool areLinesConnected(const LineSegment& line1, const LineSegment& line2, double threshold = 5.0);
    
    // 参数
    static constexpr double PARALLEL_THRESHOLD = 0.1;      // 平行阈值 (弧度)
    static constexpr double PERPENDICULAR_THRESHOLD = 0.1; // 垂直阈值 (弧度)
    static constexpr double PROXIMITY_THRESHOLD = 20.0;    // 接近度阈值 (像素)
    static constexpr double COLLINEARITY_THRESHOLD = 5.0;  // 共线性阈值 (像素)
};

#endif 