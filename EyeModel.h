#pragma once

#include <stdint.h>
#include <vector>

#include <opencv2\core\types_c.h> // for CvPoint3D32f

#include "aam_lib\AAM_Shape.h"

class EyeModel
{
  public:
    EyeModel() :
      m_eyeBallRadius(2.0),
      m_offsetLen(0.),
      m_gaze3D{ 0., 0. },
      m_eyeMid{ 0., 0., 0. }
    { /* empty */ }

    void calcEyeMidAndOffsetLen(std::vector<CvPoint3D32f> const & points3D, cv::Point eyeCornersIndex);

    void calc2DEyeMidAndCenter(CvPoint3D32f const & headTranslation, CvVect32f const & m_translationVector, CvPoint2D32f & point2DC, CvPoint2D32f & point2DM);

    void calc3DGazeDirection(cv::Point2d leftEyeCorner, cv::Point2d rightEyeCorner, CvPoint2D32f iris, CvPoint2D32f EC);

    double m_eyeBallRadius;

    double m_offsetLen;

    struct Gaze3D // TODO: there already is such a class somewhere in the m_gazer class, which is accidently does the same as this, oops
    {
      double theta;
      double phi;
    };

    void drawGazeDirection(cv::Mat & img, CvPoint2D32f * eyeCenter, CvPoint2D32f * eyeMid, CvPoint2D32f * iris);

  private:
    std::array<float, 16> calcModelView(CvPoint3D32f const & headTranslation, CvVect32f const & m_translationVector);

    Gaze3D m_gaze3D;

    CvPoint3D32f m_eyeMid;
};

class Gazer
{
  public:
    Gazer() :
      m_relative3DGazeDegrees{ 0.0, 0.0 },
      m_savedRelative3DGazeDegrees{ 500.0, 500.0 },
      m_gazeCoordinateIndex{ 0, 0 }
    { /* empty */ }

    virtual ~Gazer() = default;

    // Detect the left or right iris
    void detectEyeBlob(AAM_Shape const & Shape, std::vector<CvPoint2D32f> & kp, std::string const & eye = "left");

    // TODO: v2.0
    //void whereDoYouLook(float theta, float phi);

  private:
    struct Relative3DGazeDegrees
    {
      double theta;
      double phi;
    };

    Relative3DGazeDegrees m_relative3DGazeDegrees;
    Relative3DGazeDegrees m_savedRelative3DGazeDegrees;

    struct GazeCoordinateIndex
    {
      uint8_t horizontal;
      uint8_t vertical;
    };

    GazeCoordinateIndex m_gazeCoordinateIndex;
};