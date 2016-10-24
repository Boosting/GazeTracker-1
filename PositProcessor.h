#pragma once

#include <functional>
#include <memory>
#include <vector>

#include <opencv2\calib3d\calib3d.hpp>
#include "aam_lib\AAM_Shape.h"

class AAMWrapper;

namespace PositNS
{
  extern std::function<void(CvPOSITObject *)> positObjectDeleterFunction;
}

class PositProcessor
{
  public:
    PositProcessor();
    ~PositProcessor();

    struct FaceOrientation
    {
      double yaw;
      double pitch;
      double roll;
    };

    struct TranslationCoordinates
    {
      double x;
      double y;
      double z;
    };

    /**
     * Interface methods for OpenGl
     */
    void glDoAamPositFromAvi(AAM_Shape const & shape, cv::Mat const & image);

    // TODO: v2.0
    //void glDoAamPosit();
    // TODO: v2.0
    //void glDoAamPositWithCamera();

    FaceOrientation getFaceOrientation() { return m_faceOrientation; }

    std::vector<CvPoint3D32f> const & getPoints3D() { return m_points3D; } // TODO: make it safe

    CvVect32f getTranslationVector() { return m_translationVector; }

  private:
    // TODO: it may should be a functionality of the AAMWrapper, to let that object do the work on itself
    void mapShapeToModel(AAM_Shape const & shape, std::vector<CvPoint2D32f> & points2D, float const & width, float const & height);

    /**
     * Calculates the pose of the head and eyes
     */
    void doPositProcessing(AAM_Shape const & shape, cv::Mat const & image);

    void calcAngles(CvMatr32f m_rotationMatrix);

    //std::unique_ptr<CvPOSITObject> m_positObject; // it cannot be used with a smart pointer, because its definition is buried into the posit.cpp file
    std::unique_ptr<CvPOSITObject, decltype(PositNS::positObjectDeleterFunction)> m_positObject{ nullptr, PositNS::positObjectDeleterFunction };

    std::vector<CvPoint3D32f> m_points3D; // This the reduced 3D model with only the important points in it for OpenGL, calculated without translation vector

    FaceOrientation m_faceOrientation;      // Difference in degrees between the latest and the next-to-last headpose
    FaceOrientation m_savedFaceOrientation; // Stores the latest headpose for headmovement calculations

    TranslationCoordinates m_headCoordinates;       // Translation coordinates of the head, gives the direction and distance of the head from the camera
    TranslationCoordinates m_savedHeadCoordinates;  // Stores the latest translation coordinates

    std::unique_ptr<IplImage> m_image; // Image loaded from camera or hdd in IplImage format

    CvMatr32f m_rotationMatrix;
    CvVect32f m_translationVector;
};