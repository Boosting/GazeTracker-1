#include "AAMWrapper.h"
#include "Resources.h"

#include <iostream>

#include <opencv2\calib3d\calib3d.hpp> // for CvPOSITObject
#include <opencv2\core\types_c.h>

std::vector<CvPoint3D32f> AAMWrapper::m_modelPoints(std::vector<CvPoint3D32f>(0));

namespace
{
  int iy_14 = 2;
  int iy_18 = 2;
  int iy_22 = 6;
  int iy_26 = 6;

  /* Blob detection */
  int blobColor = 255;
  int minArea = 5;
  int maxArea = 50;
  int minThreshold = 225;
  int maxThreshold = 255;
  int thresholdStep = 1;
  /* End of blob detection */

  // This is an offset along the axis Z for the original model
  double const offsetOfZ = -6.763430;

  // This is the original model, it can not be modified, but it is copied into a mutable vector, which should be used instead of this array
  double Model3D[58][3] =
  {
    { -7.308957 + 0.5,        0.913869,              0.000000 + offsetOfZ },    //1
    { -6.775290,             -0.730814,             -0.012799 + offsetOfZ },
    { -5.665918 - 0.5,       -3.286078 + 0.6,        1.022951 + offsetOfZ },
    { -5.011779,             -4.876396,              1.047961 + offsetOfZ },
    { -4.056931,             -5.947019,              1.636229 + offsetOfZ },    //5
    { -1.833492,             -7.056977,              4.061275 + offsetOfZ },
    { 0.000000,              -7.415691,              4.070434 + offsetOfZ },
    { 1.833492,              -7.056977,              4.061275 + offsetOfZ },
    { 4.056931,              -5.947019,              1.636229 + offsetOfZ },
    { 5.011779,              -4.876396,              1.047961 + offsetOfZ },    //10
    { 5.665918 + 0.5,        -3.286078 + 0.6,        1.022951 + offsetOfZ },
    { 6.775290,              -0.730814,             -0.012799 + offsetOfZ },
    { 7.308957 - 0.5,         0.913869,              0.000000 + offsetOfZ },
    { 5.311432 - 0.5,         5.485328 - 0.5,        3.987654 + offsetOfZ },
    { 4.461908,               6.189018,              5.594410 + offsetOfZ },    //15
    { 3.550622,               6.185143,              5.712299 + offsetOfZ },
    { 2.542231,               5.862829,              4.687939 + offsetOfZ },
    { 1.789930,               5.393625 - 0.5,        4.413414 + offsetOfZ },
    { 2.693583,               5.018237,              5.072837 + offsetOfZ },
    { 3.530191,               4.981603,              4.937805 + offsetOfZ },    //20
    { 4.490323,               5.186498,              4.694397 + offsetOfZ },
    { -5.311432 + 0.5,        5.485328 - 0.5,        3.987654 + offsetOfZ },
    { -4.461908,              6.189018,              5.594410 + offsetOfZ },
    { -3.550622,              6.185143,              5.712299 + offsetOfZ },
    { -2.542231,              5.862829,              4.687939 + offsetOfZ },    //25
    { -1.789930,              5.393625 - 0.5 ,       4.413414 + offsetOfZ },
    { -2.693583,              5.018237,              5.072837 + offsetOfZ },
    { -3.530191,              4.981603,              4.937805 + offsetOfZ },
    { -4.490323,              5.186498,              4.694397 + offsetOfZ },
    { 1.330353,               7.122144,              6.903745 + offsetOfZ },    //30
    { 2.533424,               7.878085,              7.451034 + offsetOfZ },
    { 4.861131,               7.878672,              6.601275 + offsetOfZ },
    { 6.137002,               7.271266,              5.200823 + offsetOfZ },
    { 6.825897,               6.760612,              4.402142 + offsetOfZ },
    { -1.330353,              7.122144,              6.903745 + offsetOfZ },    //35
    { -2.533424,              7.878085,              7.451034 + offsetOfZ },
    { -4.861131,              7.878672,              6.601275 + offsetOfZ },
    { -6.137002,              7.271266,              5.200823 + offsetOfZ },
    { -6.825897,              6.760612,              4.402142 + offsetOfZ },
    { -2.774015,             -2.080775,              5.048531 + offsetOfZ },    //40
    { -0.509714,             -1.571179,              6.566167 + offsetOfZ },
    { 0.000000,              -1.646444,              6.704956 + offsetOfZ },
    { 0.509714,              -1.571179,              6.566167 + offsetOfZ },
    { 2.774015,              -2.080775,              5.048531 + offsetOfZ },
    { 0.589441,              -2.958597,              6.109526 + offsetOfZ },    //45
    { 0.000000,              -3.116408,              6.097667 + offsetOfZ },
    { -0.589441,             -2.958597,              6.109526 + offsetOfZ },
    { -0.981972,              4.554081,              6.301271 + offsetOfZ },
    { -0.973987,              1.916389,              7.654050 + offsetOfZ },
    { -2.005628,              1.409845,              6.165652 + offsetOfZ },    //50
    { -1.930245,              0.424351,              5.914376 + offsetOfZ },
    { -0.746313,              0.348381,              6.263227 + offsetOfZ },
    { 0.000000,               0.000000,              6.763430 + offsetOfZ },
    { 0.746313,               0.348381,              6.263227 + offsetOfZ },
    { 1.930245,               0.424351,              5.914376 + offsetOfZ },    //55
    { 2.005628,               1.409845,              6.165652 + offsetOfZ },
    { 0.973987,               1.916389,              7.654050 + offsetOfZ },
    { 0.981972,               4.554081,              6.301271 + offsetOfZ },
  };
}


void AAMWrapper::buildAAM()
{
  if (FaceTrackerResources::getPathToPoinst().length() == 0)
  {
    std::cerr << "The path of images and landmarks is not available. Exiting..." << std::endl;
    return;
  }

  int type = TYPE_AAM_IC;
  int level = 2;

  file_lists imgFiles = AAM_Common::ScanNSortDirectory(FaceTrackerResources::getPathToPoinst(), "jpg");
  file_lists ptsFiles = AAM_Common::ScanNSortDirectory(FaceTrackerResources::getPathToPoinst(), "pts");

  if (ptsFiles.size() != imgFiles.size())
  {
    fprintf(stderr, "ERROR(%s, %d): #Shapes != #Images\n", __FILE__, __LINE__);
    exit(0);
  }

  VJfacedetect vjFaceDetector;
  std::stringstream sstr;
  sstr << FaceTrackerResources::getAppPath() << FaceTrackerResources::getCascade();
  vjFaceDetector.LoadCascade(sstr.str().c_str());

  AAM_Pyramid model;
  model.Build(ptsFiles, imgFiles, type, level);
  model.BuildDetectMapping(ptsFiles, imgFiles, vjFaceDetector);
  model.WriteModel(FaceTrackerResources::getAAMName());
}


void AAMWrapper::create3DModel()
{
  if (m_modelPoints.size() == 0)
  {
    // Reference-point
    m_modelPoints.push_back(cvPoint3D32f(0.0, 0.0, 0.0));

    //tip of the nose
    //m_modelPoints.push_back(cvPoint3D32f(Model3D[52][0], Model3D[52][1], Model3D[52][2]));

    //for (int i = 0; i < 58; ++i)
    for (int i = 0; i < 13; ++i)
      m_modelPoints.push_back(cvPoint3D32f(Model3D[i][0], Model3D[i][1], Model3D[i][2]));

    //right eye corners
    double fy_21 = Model3D[21][1] - (double)iy_22 / 10.0;
    double fy_25 = Model3D[25][1] - (double)iy_26 / 10.0;

    m_modelPoints.push_back(cvPoint3D32f(Model3D[21][0], fy_21, Model3D[21][2]));
    m_modelPoints.push_back(cvPoint3D32f(Model3D[25][0], fy_25, Model3D[25][2]));

    //left eye corners
    double fy_13 = Model3D[13][1] - (double)iy_14 / 10.0;
    double fy_17 = Model3D[17][1] - (double)iy_18 / 10.0;

    m_modelPoints.push_back(cvPoint3D32f(Model3D[13][0], fy_13, Model3D[13][2]));
    m_modelPoints.push_back(cvPoint3D32f(Model3D[17][0], fy_17, Model3D[17][2]));

    //mouth corners
    m_modelPoints.push_back(cvPoint3D32f(Model3D[39][0], Model3D[39][1], Model3D[39][2]));
    m_modelPoints.push_back(cvPoint3D32f(Model3D[43][0], Model3D[43][1], Model3D[43][2]));

    //nose edges
    m_modelPoints.push_back(cvPoint3D32f(Model3D[49][0], Model3D[49][1], Model3D[49][2]));
    m_modelPoints.push_back(cvPoint3D32f(Model3D[55][0], Model3D[55][1], Model3D[55][2]));
  }

  std::cout << "\n-.- SOURCE MODEL POINTS -.-\n";
  for (size_t p = 0; p < m_modelPoints.size(); ++p)
    std::cout << m_modelPoints[p].x << ", " << m_modelPoints[p].y << ", " << m_modelPoints[p].z << "\n";
}


void AAMWrapper::initVJAndAAM()
{
  std::stringstream sstr;
  sstr << FaceTrackerResources::getAppPath() << FaceTrackerResources::getAAMFile();
  m_aamModel.ReadModel(sstr.str().c_str());

  std::stringstream sstr1;
  sstr1 << FaceTrackerResources::getAppPath() << FaceTrackerResources::getCascade();
  m_violaJonesFaceDetector.LoadCascade(sstr1.str().c_str());
}


void AAMWrapper::fitAAM(AAM_Pyramid * model, AAM_Shape & shape, IplImage * image)
{
  model->Fit(image, shape, 30, false);
  //model->Draw(image, shape, 1);
}


bool AAMWrapper::fitAAMToImage(IplImage & image)
{
  if (m_aamModel.InitShapeFromDetBox(m_aamShape, m_violaJonesFaceDetector, &image) == false)
  {
    fprintf(stderr, "The image doesn't contain any faces\n");
    FaceTrackerResources::getHeadPoseLogFile().writeFile("The image doesn't contain any faces\n");
    return false;
  }
  else
  {
    fitAAM(&m_aamModel, m_aamShape, &image);
  }

  return true;
}


void AAMWrapper::detectEyeBlob(cv::Mat const & image, std::vector<CvPoint2D32f> & kp, std::string const & eye)
{
  cv::Mat out;
  std::vector<cv::KeyPoint> keyPoints;
  keyPoints.clear();

  // m_imageMat holds the captured m_image
  cv::Mat src;
  resize(image, src, cv::Size(320, 240));

  // ROI - creating mask for the parallelogram
  cv::Mat mask = cv::Mat(240, 320, CV_8UC1);
  // Create black m_image with the same size as the original
  for (int i = 0; i < mask.cols; ++i)
    for (int j = 0; j < mask.rows; ++j)
      mask.at<uchar>(cv::Point(i, j)) = 0;

  // Create Polygon from vertices
  std::vector<cv::Point> roiEyeBoundingPolygon;
  std::vector<cv::Point> roiEyeBoundingVertices;

  int offset = 1; // TODO: what is this? I think it was a hack, because these points where not on the right places

  // TODO: don't do this right-left thing here, rather do it outside of this method and pass only the vertices here
  if (eye == "right")
  {
    roiEyeBoundingVertices.push_back(cv::Point(m_aamShape[26].x, m_aamShape[26].y - offset));
    roiEyeBoundingVertices.push_back(cv::Point(m_aamShape[27].x - offset, m_aamShape[27].y));
    roiEyeBoundingVertices.push_back(cv::Point(m_aamShape[30].x, m_aamShape[30].y + offset));
    roiEyeBoundingVertices.push_back(cv::Point(m_aamShape[29].x + offset, m_aamShape[29].y));
    roiEyeBoundingVertices.push_back(cv::Point(m_aamShape[26].x, m_aamShape[26].y - offset));
  }

  if (eye == "left")
  {
    roiEyeBoundingVertices.push_back(cv::Point(m_aamShape[33].x, m_aamShape[33].y - offset));
    roiEyeBoundingVertices.push_back(cv::Point(m_aamShape[34].x - offset, m_aamShape[34].y));
    roiEyeBoundingVertices.push_back(cv::Point(m_aamShape[35].x, m_aamShape[35].y + offset));
    roiEyeBoundingVertices.push_back(cv::Point(m_aamShape[32].x + offset, m_aamShape[32].y));
    roiEyeBoundingVertices.push_back(cv::Point(m_aamShape[33].x, m_aamShape[33].y - offset));
  }

  // Creates a parallelogram-like polygon from the vertices
  approxPolyDP(roiEyeBoundingVertices, roiEyeBoundingPolygon, 1.0, true);

  // Fill the polygon with white colour
  fillConvexPoly(mask, &roiEyeBoundingPolygon[0], roiEyeBoundingPolygon.size(), cv::Scalar(255), 8, 0);

  // Create new m_image for result storage
  cv::Mat tempImageROI = cv::Mat(240, 320, CV_8UC3);
  for (int i = 0; i < tempImageROI.cols; ++i)
    for (int j = 0; j < tempImageROI.rows; ++j)
      tempImageROI.at<uchar>(cv::Point(i, j)) = 0;

  // Cut out ROI and store it in imageROI
  src.copyTo(tempImageROI, mask);

  cv::Rect bRect = boundingRect(roiEyeBoundingVertices);
  // Check bRect, otherwise it can cause assertion error, and thus an appcrash
  if (!((bRect.x >= 0) && (bRect.y >= 0) && (bRect.width > 0) && (bRect.height > 0)))
    return;

  cv::Mat imageROI = tempImageROI(bRect);

  try
  {
    cv::cvtColor(~imageROI, imageROI, CV_BGR2GRAY);

    cv::threshold(imageROI, imageROI, minThreshold, 255, cv::THRESH_BINARY);

    cv::Mat transformedMask = cv::Mat(bRect.size(), CV_8UC1);

    std::vector<cv::Point> roiPolyTransformed;
    std::vector<cv::Point> roiVerticesTransformed;

    int transformationOffsetX = bRect.x;
    int transformationOffsetY = bRect.y;

    for (auto const & vertice : roiEyeBoundingVertices)
    {
      roiVerticesTransformed.push_back(cv::Point(vertice.x - transformationOffsetX, vertice.y - transformationOffsetY));
    }

    // Creates a parallelogram-like polygon from the vertices
    approxPolyDP(roiVerticesTransformed, roiPolyTransformed, 1.0, true);

    for (int i = 0; i < transformedMask.cols; ++i)
      for (int j = 0; j < transformedMask.rows; ++j)
        transformedMask.at<uchar>(cv::Point(i, j)) = 0;

    // Fill the polygon with white colour
    fillConvexPoly(transformedMask, &roiPolyTransformed[0], roiPolyTransformed.size(), cv::Scalar(255), 8, 0);

    for (int i = 0; i < imageROI.cols; ++i)
      for (int j = 0; j < imageROI.rows; ++j)
        if (transformedMask.at<uchar>(cv::Point(i, j)) == 0)
          imageROI.at<uchar>(cv::Point(i, j)) = 0;
  }
  catch (cv::Exception e)
  {
    std::cout << "CvException thrown in detectEyeBlob: " << e.msg << std::endl;
  }

  static cv::SimpleBlobDetector::Params params;

  if (!m_blobDetector)
  {
    //m_blobDetector->params.minDistBetweenBlobs = 10.0;
    params.blobColor = (float)blobColor; // 0 for darker, 255 for brighter colors
    params.filterByColor = true;

    params.minThreshold = (float)minThreshold;
    params.maxThreshold = (float)maxThreshold;
    params.thresholdStep = (float)thresholdStep;

    params.minArea = (float)minArea;
    params.maxArea = (float)maxArea;
    params.filterByArea = true;

    params.minRepeatability = 2;
    params.minDistBetweenBlobs = 10;

    params.filterByCircularity = false;
    params.minCircularity = 0.8f;
    params.maxCircularity = std::numeric_limits<float>::max();

    params.filterByInertia = false;
    //minInertiaRatio = 0.6;
    params.minInertiaRatio = 0.1f;
    params.maxInertiaRatio = std::numeric_limits<float>::max();

    params.filterByConvexity = false;
    //minConvexity = 0.8;
    params.minConvexity = 0.95f;
    params.maxConvexity = std::numeric_limits<float>::max();

    m_blobDetector.reset(new cv::SimpleBlobDetector(params));

    m_blobDetector->create("SimpleBlobDetector");
  }

  params.blobColor = ((float)blobColor == 0) ? 1 : (float)blobColor;
  params.minThreshold = ((float)minThreshold == 0) ? 1 : (float)minThreshold;
  params.maxThreshold = ((float)maxThreshold == 0) ? 1 : (float)maxThreshold;
  params.thresholdStep = ((float)thresholdStep == 0) ? 1 : (float)thresholdStep;
  params.minArea = ((float)minArea == 0) ? 1 : (float)minArea;
  params.maxArea = ((float)maxArea == 0) ? 1 : (float)maxArea;

  // Detect blobs
  m_blobDetector->detect(imageROI, keyPoints);

  // Draw the found blobs onto the m_image
  drawKeypoints(imageROI, keyPoints, out, CV_RGB(0, 150, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

  if (eye == "left")
  {
    cv::Mat resizedOut;
    resize(out, resizedOut, cv::Size(160, 120));

    //imshow("BlobLeft", resizedOut);
    //cvMoveWindow("BlobLeft", 530, 0);

    resizedOut.release();
    resizedOut.deallocate();
  }

  if (eye == "right")
  {
    cv::Mat resizedOut;
    resize(out, resizedOut, cv::Size(160, 120));

    //imshow("BlobRight", resizedOut);
    //cvMoveWindow("BlobRight", 350, 0);

    resizedOut.release();
    resizedOut.deallocate();
  }

  imageROI.release();
  imageROI.deallocate();


  // TODO: move it to the KalmanFilter class (smoothing)
  static bool isKalmanFilterInitialized = false;
  static cv::KalmanFilter kalmanLeftEye(4, 2, 0);
  static cv::KalmanFilter kalmanRightEye(4, 2, 0);
  static cv::Mat_<float> measurementLeft(2, 1);
  static cv::Mat_<float> measurementRight(2, 1);
  if (!isKalmanFilterInitialized)
  {
    // one for the left eye
    kalmanLeftEye.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
    measurementLeft.setTo(cv::Scalar(0));

    // init...
    kalmanLeftEye.statePre.at<float>(0) = 0; // .x;
    kalmanLeftEye.statePre.at<float>(1) = 0; // .y;
    kalmanLeftEye.statePre.at<float>(2) = 0;
    kalmanLeftEye.statePre.at<float>(3) = 0;
    setIdentity(kalmanLeftEye.measurementMatrix);
    setIdentity(kalmanLeftEye.processNoiseCov, cv::Scalar::all(1e-4));
    setIdentity(kalmanLeftEye.measurementNoiseCov, cv::Scalar::all(1e-1));
    setIdentity(kalmanLeftEye.errorCovPost, cv::Scalar::all(.1));

    // and one for the right
    kalmanRightEye.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
    measurementRight.setTo(cv::Scalar(0));

    kalmanRightEye.statePre.at<float>(0) = 0; // .x;
    kalmanRightEye.statePre.at<float>(1) = 0; // .y;
    kalmanRightEye.statePre.at<float>(2) = 0;
    kalmanRightEye.statePre.at<float>(3) = 0;
    setIdentity(kalmanRightEye.measurementMatrix);
    setIdentity(kalmanRightEye.processNoiseCov, cv::Scalar::all(1e-4));
    setIdentity(kalmanRightEye.measurementNoiseCov, cv::Scalar::all(1e-1));
    setIdentity(kalmanRightEye.errorCovPost, cv::Scalar::all(.1));

    isKalmanFilterInitialized = true;
  }
  // End of Kalman-filtering

  if (keyPoints.size() > 0)
  {
    cv::KeyPoint tempKP = keyPoints[0];

    for (size_t i = 1; i < keyPoints.size(); ++i)
    {
      if (keyPoints[i].size > tempKP.size)  // Find the biggest blob
        tempKP = keyPoints[i];
    }

    kp.clear();
    kp.push_back(cvPoint2D32f(tempKP.pt.x + (float)bRect.x, tempKP.pt.y + (float)bRect.y));

    // Kalman-filtering

    if (eye == "left")
    {
      // First predict, to update the internal statePre variable
      cv::Mat prediction = kalmanLeftEye.predict();
      cv::Point predictPt(prediction.at<float>(0), prediction.at<float>(1));

      // Get mouse point
      measurementLeft(0) = kp.at(0).x;
      measurementLeft(1) = kp.at(0).y;

      // The "correct" phase that is going to use the predicted value and our measurement
      cv::Mat estimated = kalmanLeftEye.correct(measurementLeft);
      cv::Point statePt(estimated.at<float>(0), estimated.at<float>(1));
      kp.clear();
      kp.push_back(cvPoint2D32f(estimated.at<float>(0), estimated.at<float>(1)));
    }
    else
    {
      // First predict, to update the internal statePre variable
      cv::Mat prediction = kalmanRightEye.predict();
      cv::Point predictPt(prediction.at<float>(0), prediction.at<float>(1));

      // Get mouse point
      measurementRight(0) = kp.at(0).x;
      measurementRight(1) = kp.at(0).y;

      // The "correct" phase that is going to use the predicted value and our measurement
      cv::Mat estimated = kalmanRightEye.correct(measurementRight);
      cv::Point statePt(estimated.at<float>(0), estimated.at<float>(1));
      kp.clear();
      kp.push_back(cvPoint2D32f(estimated.at<float>(0), estimated.at<float>(1)));
    }
    // End of Kalman-filtering
  }
  else
  {
    kp.clear();
  }

  src.release();
  src.deallocate();
  out.release();
  out.deallocate();
  mask.release();
  mask.deallocate();
  tempImageROI.release();
  tempImageROI.deallocate();
  keyPoints.clear();
}


#pragma region TODO: v3.0 movable modelpoints for model alignment to individual face at runtime
//// This is a callback function for every trackbar change
//void FaceTracker::on_trackbar(int, void*)
//{
//  m_modelPoints.clear();
//
//  m_positObject = nullptr;
//
//  create3DModel();
//}
//
//
//void FaceTracker::createTrackBars(std::string wndName)
//{
//  // Create Trackbars
//  char TrackbarName[50];
//  sprintf(TrackbarName, "blobColor");
//  cv::createTrackbar(TrackbarName, wndName, &blobColor, 255);
//
//  sprintf(TrackbarName, "minArea");
//  cv::createTrackbar(TrackbarName, wndName, &minArea, 100);
//
//  sprintf(TrackbarName, "maxArea");
//  cv::createTrackbar(TrackbarName, wndName, &maxArea, 200);
//
//  sprintf(TrackbarName, "minThreshold");
//  cv::createTrackbar(TrackbarName, wndName, &minThreshold, 255);
//
//  sprintf(TrackbarName, "maxThreshold");
//  cv::createTrackbar(TrackbarName, wndName, &maxThreshold, 255);
//
//  sprintf(TrackbarName, "thresholdStep");
//  cv::createTrackbar(TrackbarName, wndName, &thresholdStep, 100);
//
//  // Adjust model parameters
//  sprintf(TrackbarName, "y_14");
//  cv::createTrackbar(TrackbarName, wndName, &iy_14, 100, on_trackbar);
//
//  sprintf(TrackbarName, "y_18");
//  cv::createTrackbar(TrackbarName, wndName, &iy_18, 100, on_trackbar);
//
//  sprintf(TrackbarName, "y_22");
//  cv::createTrackbar(TrackbarName, wndName, &iy_22, 100, on_trackbar);
//
//  sprintf(TrackbarName, "y_26");
//  cv::createTrackbar(TrackbarName, wndName, &iy_26, 100, on_trackbar);
//}
#pragma endregion
