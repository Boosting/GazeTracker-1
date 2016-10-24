#include "Calibrator.h"

#include <time.h>

#include <opencv2\core\core_c.h>
#include <opencv2\highgui\highgui.hpp>

//void Calibrator::calibrate()
//{
//  time_t now;
//  time(&now);
//
//  if (difftime(now, m_calibTime) < 4) // Look at least this many seconds in one direction
//  {
//    m_calibTemp.push_back(cv::Point2f(avgTheta, avgPhi));
//  }
//  else
//  {
//    float tempTheta = 0.0, tempPhi = 0.0;
//
//    for (std::vector<cv::Point2f>::iterator it = m_calibTemp.begin(); it != m_calibTemp.end(); ++it)
//    {
//      tempTheta += it->x;
//      tempPhi += it->y;
//    }
//
//    if (m_calibTemp.size() != 0)
//      m_calibResult.push_back(cv::Point2f(tempTheta / m_calibTemp.size(), tempPhi / m_calibTemp.size()));
//
//    // Write the data into the logfile
//    std::ostringstream calibStr;
//    calibStr << "m_calibResult " << m_calibPhase << ";" << tempTheta / m_calibTemp.size() << ";" << tempPhi / m_calibTemp.size() << std::endl;
//    calibStr << "m_calibTemp.Size;" << m_calibTemp.size() << std::endl;
//    FaceTrackerResources.m_gazeDirectionLogFile.writeFile(calibStr.str());
//
//    m_calibTemp.clear();
//
//    m_calibPhase++;
//    m_calibStep = true;
//  }
//}
//
//
//void Calibrator::drawCalibPoint()
//{
//  cv::Mat gazeDir(windowSize.y, windowSize.x, CV_8UC3);
//  int r = 20;
//
//  if (m_calibPhase == 0)
//    circle(gazeDir, cv::Point(r, r), r, cv::Scalar(0, 0, 255), -1, CV_AA);
//  else if (m_calibPhase == 1)
//    circle(gazeDir, cv::Point(windowSize.x / 2, r), r, cv::Scalar(0, 0, 255), -1, CV_AA);
//  else if (m_calibPhase == 2)
//    circle(gazeDir, cv::Point(windowSize.x - r, r), r, cv::Scalar(0, 0, 255), -1, CV_AA);
//  else if (m_calibPhase == 5)
//    circle(gazeDir, cv::Point(r, windowSize.y / 2), r, cv::Scalar(0, 0, 255), -1, CV_AA);
//  else if (m_calibPhase == 4)
//    circle(gazeDir, cv::Point(windowSize.x / 2, windowSize.y / 2), r, cv::Scalar(0, 0, 255), -1, CV_AA);
//  else if (m_calibPhase == 3)
//    circle(gazeDir, cv::Point(windowSize.x - r, windowSize.y / 2), r, cv::Scalar(0, 0, 255), -1, CV_AA);
//  else if (m_calibPhase == 6)
//    circle(gazeDir, cv::Point(r, windowSize.y - r), r, cv::Scalar(0, 0, 255), -1, CV_AA);
//  else if (m_calibPhase == 7)
//    circle(gazeDir, cv::Point(windowSize.x / 2, windowSize.y - r), r, cv::Scalar(0, 0, 255), -1, CV_AA);
//  else if (m_calibPhase == 8)
//    circle(gazeDir, cv::Point(windowSize.x - r, windowSize.y - r), r, cv::Scalar(0, 0, 255), -1, CV_AA);
//
//  imshow("gaze", gazeDir);
//
//  gazeDir.release();
//  gazeDir.deallocate();
//}
