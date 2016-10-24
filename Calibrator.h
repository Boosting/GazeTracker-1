#pragma once

#include <vector>

#include <opencv2\core\core.hpp>

// TODO: v2.0
//class Calibrator
//{
//  public:
//    Calibrator() :
//      m_calibPhase(9),
//      m_calibStep(true),
//      m_calibTime(0),
//      m_calibResult(std::vector<cv::Point2f>(0)),
//      m_calibTemp(std::vector<cv::Point2f>(0))
//    { /* empty */ }
//
//    virtual ~Calibrator() = default;
//
//    void calibrate();
//
//    void drawCalibPoint();
//
//  private:
//    char m_calibPhase; // 9 phase for 9 different points on the screen
//    bool m_calibStep;  // refresh the calibTime at the first iteration of every phase
//    time_t m_calibTime;
//    std::vector<cv::Point2f> m_calibResult;
//    std::vector<cv::Point2f> m_calibTemp;
//};
