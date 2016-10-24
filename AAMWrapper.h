#pragma once

#include <memory>

#include <opencv2\core\types_c.h>

#include "aam_lib\AAM_Util.h"  // for AAM_Pyramid
#include "aam_lib\AAM_Shape.h" // for AAM_Shape
#include "aam_lib\VJfacedetect.h"

#include "EyeModel.h"

class AAMWrapper
{
  public:
    AAMWrapper() :
      m_blobDetector(nullptr)
    {
      create3DModel();
    }

    virtual ~AAMWrapper() = default;

    struct Eyes
    {
      EyeModel left;
      EyeModel right;
    };

    /**
     * Builds an AAM from images and landmarks
     */
    void buildAAM(); // TODO: refactor it to be able to see from the signature what is used inside it

    void initVJAndAAM();

    bool fitAAMToImage(IplImage & image);

    void detectEyeBlob(cv::Mat const & image, std::vector<CvPoint2D32f> & kp, std::string const & eye);

    Eyes & getEyes() { return m_eyes; }

    AAM_Shape const & getShape() const { return m_aamShape; }

    static std::vector<CvPoint3D32f> m_modelPoints; // This is a reduced 3D model with only the important points in it

  private:
    /**
     * Loads the 3D points into m_modelPoints
     * Classification: Internal
     */
    void create3DModel();

    void fitAAM(AAM_Pyramid * model, AAM_Shape & shape, IplImage * image);

    VJfacedetect m_violaJonesFaceDetector;

    AAM_Pyramid m_aamModel;
    AAM_Shape m_aamShape;

    Eyes m_eyes;

    std::unique_ptr<cv::SimpleBlobDetector> m_blobDetector;

    // TODO: v3.0 - but hopefully not in this class
    //void createTrackBars(std::string wndName);
    // TODO: v3.0
    //static void on_trackbar(int, void *);
};
