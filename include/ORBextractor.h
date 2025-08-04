/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>
#include "EDLines.h"  // EDLines头文件

// Forward declaration for LineGraphModel
namespace line_graph {
    class LineGraphModel;
}

namespace ORB_SLAM3
{

// Line structure for storing line features
struct Line {
    cv::Point2f start, end;
    float length;
    float angle;
    float response;
    int octave;
    bool isValid;
    
    Line() : length(0), angle(0), response(0), octave(0), isValid(false) {}
    Line(cv::Point2f s, cv::Point2f e, float resp = 0, int oct = 0) 
        : start(s), end(e), response(resp), octave(oct), isValid(true) {
        length = cv::norm(end - start);
        angle = atan2(end.y - start.y, end.x - start.x);
    }
};

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false), bHasLineFeatures(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);
    
    // Line feature processing methods
    bool hasLineFeatures() const;
    void processLineFeatures(const cv::Mat& image, line_graph::LineGraphModel& lineModel);
    void extractAdaptiveLineFeatures(const cv::Mat& image, line_graph::LineGraphModel& lineModel, 
                                   float sparsityThreshold = 5.0f);

    std::vector<cv::KeyPoint> vKeys;
    std::vector<Line> vLines;  // Added line features storage
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
    bool bHasLineFeatures;  // Added flag for line features
};

class ORBextractor
{
public:

    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor();

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    int operator()( cv::InputArray _image, cv::InputArray _mask,
                    std::vector<cv::KeyPoint>& _keypoints,
                    cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

    // Enhanced operator that also extracts line features
    int operator()( cv::InputArray _image, cv::InputArray _mask,
                    std::vector<cv::KeyPoint>& _keypoints,
                    cv::OutputArray _descriptors, std::vector<int> &vLappingArea,
                    std::vector<Line>& _lines);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    // Line feature accessor methods
    std::vector<Line> inline GetExtractedLines(){
        return mvExtractedLines;
    }

    void SetAdaptiveLineExtraction(bool enable) {
        mbAdaptiveLineExtraction = enable;
    }

    std::vector<cv::Mat> mvImagePyramid;

protected:

    void ComputePyramid(cv::Mat image);
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    // Enhanced DistributeOctTree with line feature extraction
    std::vector<cv::KeyPoint> DistributeOctTreeWithLines(const std::vector<cv::KeyPoint>& vToDistributeKeys, 
                                           const int &minX, const int &maxX, const int &minY, const int &maxY, 
                                           const int &nFeatures, const int &level, const cv::Mat& image);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    
    // Line feature extraction methods
    void ExtractLineFeatures(const cv::Mat& image, std::vector<Line>& lines, int level = 0);
    void FilterLineFeatures(std::vector<Line>& lines, float minLength = 30.0f, float maxLineError = 2.0f);
    bool IsSparseSampling(const std::vector<cv::KeyPoint>& keypoints, const cv::Rect& region, float threshold = 5.0f);
    
    std::vector<cv::Point> pattern;

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
    
    // Line feature related members
    std::vector<Line> mvExtractedLines;
    std::vector<std::vector<Line>> lineFeatures; // 存储每个层级的线特征
    line_graph::LineGraphModel* mLineGraphModel; // 改为指针
    EDLines* edLines; // EDLines对象指针
    bool mbAdaptiveLineExtraction;
    float mfLineMinLength;
    float mfLineMaxError;
    float mfSparsityThreshold;
    float mfMinAngle;      // 线特征的最小角度
    float mfMaxAngle;      // 线特征的最大角度
    float mfMinResponse;   // 线特征的最小响应强度
    
    // 线特征检查和处理方法
    bool hasLineFeature(const cv::Point2f& pt, int level = 0);
    void processLineFeatures(const std::vector<Line>& lines);
    void matchLineFeatures(const std::vector<Line>& lines);
    bool filterLineCondition(const Line& line);
};

} //namespace ORB_SLAM

#endif

