#include "PositProcessor.h"

#include "aam_lib\AAM_Shape.h"
#include "aam_lib\AAM_Util.h"

#include "AAMWrapper.h"
#include "Resources.h"


namespace PositNS
{
  std::function<void(CvPOSITObject *)> positObjectDeleterFunction = [](CvPOSITObject * p) { cvReleasePOSITObject(&p); };
}


namespace
{
  int blobColor = 255;
  int minArea = 5;
  int maxArea = 50;
  int minThreshold = 225;
  int maxThreshold = 255;
  int thresholdStep = 1;

  std::vector<uint8_t> importantShapeCoordinates =
  {
    41,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, // face contour
    27, 29, // right eye corners
    34, 32, // left eye corners
    48, 54, // mouth corners
    39, 43  // nose edges
  };
}


PositProcessor::PositProcessor() :
  m_points3D(std::vector<CvPoint3D32f>(0)),
  m_faceOrientation{ (0.0), (0.0), (0.0) },
  m_savedFaceOrientation{ (0.0), (0.0), (0.0) },
  m_headCoordinates{ (0.0), (0.0), (0.0) },
  m_savedHeadCoordinates{ (0.0), (0.0), (0.0) },
  m_rotationMatrix(nullptr),
  m_translationVector(nullptr)
{ /* empty */ }


PositProcessor::~PositProcessor()
{ /* empty */ }


void PositProcessor::doPositProcessing(AAM_Shape const & shape, cv::Mat const & image)
{
  std::vector<CvPoint2D32f> points;

  // Map the Shape points to modelPoints
  mapShapeToModel(shape, points, FaceTrackerResources::getHalfPictureWidth(), FaceTrackerResources::getHalfPictureHeight());

  cv::Mat img;
  resize(image, img, cv::Size(FaceTrackerResources::getPictureWidth(), FaceTrackerResources::getPictureHeight()));

  // Estimate the pose in 3D space
  m_rotationMatrix = new float[9];
  m_translationVector = new float[3];
  CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 1.0e-4f);

  // TODO: v4.0 do it during runtime as well, when the user changes the model in CAMERA mode
  // TODO: v1.0 init positobject somewhere else
  if (!m_positObject)
    m_positObject.reset(cvCreatePOSITObject(&AAMWrapper::m_modelPoints[0], (int)AAMWrapper::m_modelPoints.size()));

  cvPOSIT(m_positObject.get(), &points[0], Resources::cameraFocalLength, criteria, m_rotationMatrix, m_translationVector);

  // Project the model points with the estimated pose
  // TODO: it isn't used anywhere, do we really need it?
  std::vector<CvPoint2D32f> projectedPoints;
  m_points3D.clear();

  for (auto modelPoint : AAMWrapper::m_modelPoints)
  {
    CvPoint3D32f point3D;

    point3D.x = m_rotationMatrix[0] * modelPoint.x + m_rotationMatrix[1] * modelPoint.y + m_rotationMatrix[2] * modelPoint.z + m_translationVector[0];
    point3D.y = m_rotationMatrix[3] * modelPoint.x + m_rotationMatrix[4] * modelPoint.y + m_rotationMatrix[5] * modelPoint.z + m_translationVector[1];
    point3D.z = m_rotationMatrix[6] * modelPoint.x + m_rotationMatrix[7] * modelPoint.y + m_rotationMatrix[8] * modelPoint.z + m_translationVector[2];

    CvPoint2D32f point2D = cvPoint2D32f(0.0, 0.0);
    if (point3D.z != 0)
    {
      point2D.x = Resources::cameraFocalLength * point3D.x / point3D.z;
      point2D.y = Resources::cameraFocalLength * point3D.y / point3D.z;
    }
    projectedPoints.push_back(point2D);


    // Calculate the points without translation vector for OpenGL
    CvPoint3D32f point3DOGL;
    point3DOGL.x = m_rotationMatrix[0] * modelPoint.x + m_rotationMatrix[1] * modelPoint.y + m_rotationMatrix[2] * modelPoint.z;
    point3DOGL.y = m_rotationMatrix[3] * modelPoint.x + m_rotationMatrix[4] * modelPoint.y + m_rotationMatrix[5] * modelPoint.z;
    point3DOGL.z = m_rotationMatrix[6] * modelPoint.x + m_rotationMatrix[7] * modelPoint.y + m_rotationMatrix[8] * modelPoint.z;

    m_points3D.push_back(point3DOGL);
  }

  //cout << "\n-.- PROJECTED POINTS -.-\n";
  //for (size_t p = 0; p < projectedPoints.size(); ++p)
  //cout << projectedPoints[p].x << ", " << projectedPoints[p].y << " \n";

  //// Draw projected points onto the m_image
  //for (unsigned int i = 0; i < projectedPoints.size(); ++i)
  //{
  //  CvPoint2D32f p = cvPoint2D32f(projectedPoints[i].x + 320, 240 - projectedPoints[i].y);
  //  circle(img, p, 2, cv::Scalar(0, 255, 0), CV_FILLED);
  //}

  // Calculate the angles of the head (yaw, roll, pitch)
  calcAngles(m_rotationMatrix);

  // Write angles onto the image
  CvPoint2D32f py = cvPoint2D32f(10, 15);
  CvPoint2D32f pp = cvPoint2D32f(10, 30);
  CvPoint2D32f pr = cvPoint2D32f(10, 45);
  //CvPoint2D32f ptx = cvPoint2D32f(10, 60);

  std::stringstream sstry;
  std::stringstream sstrp;
  std::stringstream sstrr;
  std::stringstream sstrtx;
  sstry << "Yaw: " << m_savedFaceOrientation.yaw;
  sstrp << "Pitch: " << m_savedFaceOrientation.pitch;
  sstrr << "Roll: " << m_savedFaceOrientation.roll;
  //sstrtx << "Distance: " << m_translationVector[2];
  std::string sty = sstry.str();
  std::string stp = sstrp.str();
  std::string str = sstrr.str();
  //string sttx = sstrtx.str();

  CvFont font;
  double hScale = .5;
  double vScale = .5;
  int    lineWidth = 1;
  cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, hScale, vScale, 0, lineWidth);
  putText(img, sty, py, CV_FONT_HERSHEY_COMPLEX, hScale, cv::Scalar(0, 0, 255));
  putText(img, stp, pp, CV_FONT_HERSHEY_COMPLEX, hScale, cv::Scalar(0, 0, 255));
  putText(img, str, pr, CV_FONT_HERSHEY_COMPLEX, hScale, cv::Scalar(0, 0, 255));
  //putText(img, sttx, ptx, CV_FONT_HERSHEY_COMPLEX, hScale, Scalar(0, 0, 255));

  //FaceTx1 = m_translationVector[0] - txSave;
  //FaceTy1 = m_translationVector[1] - tySave;
  //FaceTz1 = m_translationVector[2] - tzSave;

  //txSave = m_translationVector[0];
  //tySave = m_translationVector[1];
  //tzSave = m_translationVector[2];

  //if (calibPhase > 8 && calibResult.size())
  //  whereDoYouLook(avgTheta, avgPhi);

  //delete[] m_rotationMatrix; // TODO: check every delete, whether they have to be array delete
  //delete[] m_translationVector; // TODO: deleting them is a bad idea, because they are accessible from outside

  //cv::resize(img, img, cv::Size(320, 240));
  //imshow("posit", img);
  //cvMoveWindow("posit", 900, 0);

  img.release();
  img.deallocate();
}


void PositProcessor::glDoAamPositFromAvi(AAM_Shape const & shape, cv::Mat const & image)
{
  //if (FaceTrackerResources::getVideoCapture().read(m_imageMat))
  //{
  m_faceOrientation.yaw = m_faceOrientation.pitch = m_faceOrientation.roll = 0.0;

  doPositProcessing(shape, image);

  // TODO: close the logfiles in case of exiting
  //}
  //else
  //{
  //  if (FaceTrackerResources::getLogFile() != "")
  //  {
  //    FaceTrackerResources::getHeadPoseLogFile().closeFile();
  //  }

  //  if (FaceTrackerResources::getGazeLogFile() != "")
  //  {
  //    FaceTrackerResources::getGazeDirectionLogFile().closeFile();
  //  }
  //}
}


void PositProcessor::mapShapeToModel(AAM_Shape const & shape, std::vector<CvPoint2D32f> & points, float const & width, float const & height)
{
  CvPoint pt {0, 0};
  for (auto const & coordinate : importantShapeCoordinates)
  {
    pt = cvPointFrom32f(shape[coordinate]);
    points.push_back(cvPoint2D32f(-width + pt.x, height - pt.y));
  }
}


void PositProcessor::calcAngles(CvMatr32f m_rotationMatrix)
{
  // Calculate axes (angles)
  float yaw = -asinf(m_rotationMatrix[6]);
  float pitch = atan2(m_rotationMatrix[7] / cos(yaw), m_rotationMatrix[8] / cos(yaw));
  float roll = atan2(m_rotationMatrix[3] / cos(yaw), m_rotationMatrix[0] / cos(yaw));

  //second form of calculation
  //float yaw = (float) atan2f(m_rotationMatrix[2], m_rotationMatrix[8]);
  //float pitch = (float) asin(-m_rotationMatrix[5]);
  //float roll = (float) atan2f(m_rotationMatrix[3], m_rotationMatrix[4]);
  //Calculate the angle in degree with radian * (float) (180.0 / CV_PI);
  //cout << endl << "yaw: " << yaw * (float) (180.0 / CV_PI) << " pitch: " << pitch * (float) (180.0 / CV_PI) << " roll: " << roll * (float) (180.0 / CV_PI) << endl;

  //pmartins
  //float pitch = (float) (atan(m_rotationMatrix[3]/m_rotationMatrix[0]));
  //float yaw = (float) (atan(-m_rotationMatrix[6]/m_rotationMatrix[0]) * cos(yaw) + m_rotationMatrix[3]*sin(yaw));
  //float roll = (float) (atan(m_rotationMatrix[7] / m_rotationMatrix[8]));
  //cout << "yaw: " << pitch * (float) (180.0 / CV_PI) << " pitch: " << roll * (float) (180.0 / CV_PI) << " roll: " << yaw * (float) (180.0 / CV_PI) << endl;

  //calculate degrees from rad
  yaw *= (float)(180.0 / CV_PI);
  pitch *= (float)(180.0 / CV_PI);
  roll *= (float)(180.0 / CV_PI);

  m_faceOrientation.yaw = yaw - m_savedFaceOrientation.yaw;
  m_faceOrientation.pitch = pitch - m_savedFaceOrientation.pitch;
  m_faceOrientation.roll = roll - m_savedFaceOrientation.roll;

  m_savedFaceOrientation.yaw = yaw;
  m_savedFaceOrientation.pitch = pitch;
  m_savedFaceOrientation.roll = roll;

  // Write data into file
  std::ostringstream data;
  data << m_savedFaceOrientation.yaw << ";" << m_savedFaceOrientation.pitch << ";" << m_savedFaceOrientation.roll << ";" << m_savedHeadCoordinates.z << std::endl;
  FaceTrackerResources::getHeadPoseLogFile().writeFile(data.str());
}


#pragma region POSIT with camera and stored images // TODO v2.0
//void PositProcessor::glDoAamPositWithCamera()
//{
//  if (!areFilesLoaded) // Load the important files just the first time
//  {
//    std::stringstream sstr;
//    sstr << FaceTrackerResources.appPath << FaceTrackerResources.aamFile;
//    m_aamModel.ReadModel(sstr.str().c_str());
//
//    std::stringstream sstr1;
//    sstr1 << FaceTrackerResources.appPath << FaceTrackerResources.cascade;
//    m_violaJonesFaceDetector.LoadCascade(sstr1.str().c_str());
//
//    // Start the clock
//    time(&start);
//
//    // Initializing the window for the output m_image
//    cvNamedWindow("img", CV_WINDOW_AUTOSIZE);
//    cvMoveWindow("img", 0, 0);
//
//    cvNamedWindow("trackbars", CV_WINDOW_NORMAL);
//    createTrackBars("trackbars");
//
//    cvNamedWindow("posit", CV_WINDOW_AUTOSIZE);
//
//    cvNamedWindow("gaze", CV_WINDOW_AUTOSIZE);
//    //cvSetWindowProperty("gaze", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
//    cvMoveWindow("gaze", 0, 0);
//    cvResizeWindow("gaze", windowSize.x, windowSize.y);
//
//    capture = cvCaptureFromCAM(CV_CAP_ANY);
//
//    // Opening a file, for saving statistic data
//    FaceTrackerResources.headPoseLogFile.openFile(FaceTrackerResources.logFile);
//    FaceTrackerResources.headPoseLogFile.writeFile("Yaw;Pitch;Roll;Distance\n");
//
//    FaceTrackerResources.gazeDirectionLogFile.openFile(FaceTrackerResources.gazeLogFile);
//
//    areFilesLoaded = true;
//  }
//
//  if (!capture)
//  {
//    fprintf(stderr, "ERROR: capture is NULL \n");
//    getchar();
//    return;
//  }
//
//  // Get a frame
//  // Comment from highgui_c.h:
//  // !!!DO NOT RELEASE or MODIFY the retrieved frame!!!
//  IplImage * sourceImage = cvQueryFrame(capture);
//  if (!sourceImage)
//  {
//    fprintf(stderr, "ERROR: frame is null...\n");
//    getchar();
//    return;
//  }
//
//  std::cout << "Image # " << imgCount << std::endl;
//
//  image = cvCreateImage(cvSize(320, 240), sourceImage->depth, sourceImage->nChannels);
//  cvResize(sourceImage, image);
//
//  m_imageMat = cv::cvarrToMat(image, true);
//
//  faceYaw = facePitch = faceRoll = 0.0;
//
//  bool flag = m_aamModel.InitShapeFromDetBox(m_aamShape, m_violaJonesFaceDetector, image);
//  if (flag == false)
//  {
//    fprintf(stderr, "The image doesn't contain any faces\n");
//  }
//  else
//  {
//    fitAAM(&m_aamModel, m_aamShape, image);           // AAM fitting and POSIT processing
//    imgCount++;
//  }
//
//  cvReleaseImage(&image);
//
//  time(&end);                               //
//  ++counter;                                //
//  double sec = difftime(end, start);        // Calculate FPS
//  double fps = counter / sec;               //
//  printf("\nfps: %.2lf\n", fps);            //
//
//                                            //if it is still the calibration phase, save the actual results
//  if (calibPhase < 9)
//  {
//    if (calibStep)
//    {
//      // Write the start of the calibration phase into file
//      std::ostringstream gazeData;
//      gazeData << "calibPhase;" << calibPhase << std::endl;
//      FaceTrackerResources.gazeDirectionLogFile.writeFile(gazeData.str());
//
//      drawCalibPoint();
//      Sleep(500); // Wait a 500ms, until the users eye catches the next point
//      time(&calibTime);
//      calibStep = false;
//    }
//
//    calibrate();
//  }
//  else
//  { // TODO: put this into the Calibrator class as a helper method:
//    //Print information about the calibration if needed
//    //cout << "calibResult: " << endl;
//    //for (char i = 0; i < calibResult.size(); i+=3)
//    //  cout << calibResult[i] << calibResult[i+1] << calibResult[i+2] << endl;
//  }
//
//  if (cvWaitKey(1) == 27)                // Esc pressed
//  {
//    cvWaitKey(0);
//  }
//
//  if (cvWaitKey(1) == 99)                // c - calibrate
//  {
//    calibPhase = 0;
//    calibStep = true;
//    calibTemp.clear();
//    calibResult.clear();
//  }
//
//  //releasing the POSIT-object
//  //cvReleasePOSITObject(&positObject);
//  //cvDestroyWindow("img");
//  //delete m_blobDetector
//  //delete m_capture;
//}
//
//
//void PositProcessor::glDoAamPosit()
//{
//  if (!areFilesLoaded) // Load the important files just the first time
//  {
//    std::stringstream sstr;
//    sstr << FaceTrackerResources.appPath << FaceTrackerResources.aamFile;
//    m_aamModel.ReadModel(sstr.str().c_str());
//
//    std::stringstream sstr1;
//    sstr1 << FaceTrackerResources.appPath << FaceTrackerResources.cascade;
//    m_violaJonesFaceDetector.LoadCascade(sstr1.str().c_str());
//
//    imgFiles = AAM_Common::ScanNSortDirectory(FaceTrackerResources.imagePath, "jpg");
//
//    // Start the clock
//    time(&start);
//
//    // Initializing the window for the output m_image
//    cvNamedWindow("img", CV_WINDOW_AUTOSIZE);
//    cvMoveWindow("img", 0, 0);
//
//    createTrackBars("img");
//
//    // Opening a file, for saving statistic data
//    FaceTrackerResources.headPoseLogFile.openFile(FaceTrackerResources.logFile); // "headPoseTestData.csv"
//    FaceTrackerResources.headPoseLogFile.writeFile("Yaw;Pitch;Roll;Distance\n");
//
//    FaceTrackerResources.gazeDirectionLogFile.openFile(FaceTrackerResources.gazeLogFile);
//
//    areFilesLoaded = true;
//  }
//
//  if (imgCount < imgFiles.size())
//  {
//    std::cout << "Image # " << imgCount << std::endl;
//
//    IplImage * sourceImage = cvLoadImage(imgFiles[imgCount].c_str(), -1); // Reading the m_image from file into IplImage, for AAM processing
//
//    m_imageMat = cv::imread(imgFiles[imgCount]);                  // Reading the same m_image into Mat, for POSIT
//
//    image = cvCreateImage(cvSize(320, 240), sourceImage->depth, sourceImage->nChannels);  //
//    cvResize(sourceImage, image);                                // Resizing the m_image
//    cvReleaseImage(&sourceImage);                                //
//
//    faceYaw = facePitch = faceRoll = 0.0;
//
//    bool flag = m_aamModel.InitShapeFromDetBox(m_aamShape, m_violaJonesFaceDetector, image);
//    if (flag == false)
//    {
//      fprintf(stderr, "The image doesn't contain any faces\n");
//    }
//    else
//    {
//      fitAAM(&m_aamModel, m_aamShape, image);            // AAM fitting and POSIT processing
//      imgCount++;
//    }
//
//    //cvShowImage("img", m_image);              // Draw the processed m_image
//    cvReleaseImage(&image);
//
//    time(&end);                      //
//    ++counter;                        //
//    double sec = difftime(end, start);          // Calculate FPS
//    double fps = counter / sec;                //
//    printf("\nfps: %.2lf\n", fps);            //
//
//    if (cvWaitKey(1) == 27)                // Esc pressed
//      cvWaitKey(0);
//  }
//  else
//  {
//    if (FaceTrackerResources.logFile != "")
//    {
//      FaceTrackerResources.headPoseLogFile.closeFile();
//    }
//
//    if (FaceTrackerResources.gazeLogFile != "")
//    {
//      FaceTrackerResources.gazeDirectionLogFile.closeFile();
//    }
//
//    //imgcnt = 1;
//
//    //releasing the POSIT-object
//    //cvReleasePOSITObject(&positObject);
//    //cvDestroyWindow("img");
//    //delete m_blobDetector
//  }
//}
#pragma endregion
