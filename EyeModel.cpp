#include "EyeModel.h"

#include <array>

#include <GL\glut.h>

#include "Resources.h"


void EyeModel::calcEyeMidAndOffsetLen(std::vector<CvPoint3D32f> const & points3D, cv::Point eyeCornersIndex)
{
  // left eye: 16th and 17th points in the vector
  // right eye: 14th and 15th points in the vector

  // Calculate the 3D position of the middle of the eye
  m_eyeMid.x = (points3D.at(eyeCornersIndex.x).x + points3D.at(eyeCornersIndex.y).x) / 2.0;
  m_eyeMid.y = (points3D.at(eyeCornersIndex.x).y + points3D.at(eyeCornersIndex.y).y) / 2.0;
  m_eyeMid.z = (points3D.at(eyeCornersIndex.x).z + points3D.at(eyeCornersIndex.y).z) / 2.0;

  // Calculate distance between EyeMid and one corner of the eye
  float distLeft = sqrtf(pow(m_eyeMid.x - points3D.at(eyeCornersIndex.x).x, 2) +
                         pow(m_eyeMid.y - points3D.at(eyeCornersIndex.x).y, 2) +
                         pow(m_eyeMid.z - points3D.at(eyeCornersIndex.x).z, 2));

  // Calculate the center of eye
  float alpha = acosf(distLeft / m_eyeBallRadius);
  m_offsetLen = m_eyeBallRadius * sin(alpha);
}


void EyeModel::calc2DEyeMidAndCenter(CvPoint3D32f const & headTranslation, CvVect32f const & m_translationVector, CvPoint2D32f & point2DC, CvPoint2D32f & point2DM)
{
  if (m_eyeMid.z == 0)
    return;

  std::array<float, 16> modelView = calcModelView(headTranslation, m_translationVector);

  if (modelView[14] != 0)
  {
    point2DC.x = Resources::cameraFocalLength * modelView[12] / modelView[14];
    point2DC.y = Resources::cameraFocalLength * modelView[13] / modelView[14];
  }

  if ((m_eyeMid.z + m_translationVector[2]) != 0)
  {
    point2DM.x = Resources::cameraFocalLength * (m_eyeMid.x + m_translationVector[0]) / (m_eyeMid.z + m_translationVector[2]);
    point2DM.y = Resources::cameraFocalLength * (m_eyeMid.y + m_translationVector[1]) / (m_eyeMid.z + m_translationVector[2]);
  }
}


std::array<float, 16> EyeModel::calcModelView(CvPoint3D32f const & headTranslation, CvVect32f const & m_translationVector)
{
  std::array<float, 16> modelView;

  glPushMatrix();
    glTranslatef(0, 0, 25);
    glTranslatef(m_eyeMid.x, m_eyeMid.y, m_eyeMid.z);
    glTranslatef(m_translationVector[0], m_translationVector[1], m_translationVector[2]);

    glRotatef(headTranslation.x, 0, 0, 1); // saved roll
    glRotatef(headTranslation.y, 1, 0, 0); // saved pitch
    glRotatef(headTranslation.z, 0, 1, 0); // saved yaw

    glTranslatef(0, 0, m_offsetLen);

    glGetFloatv(GL_MODELVIEW_MATRIX, modelView.data());

    //cout << "modelView matrix:" << endl;
    //for (int z = 0; z < 4; ++z)
    //  cout << setw(10) << modelView[z*4] << " " << setw(10) << modelView[z*4+1] << " " << setw(10) << modelView[z*4+2] << " " << setw(10) << modelView[z*4+3] << endl;
  glPopMatrix();

  return modelView;
}


void EyeModel::calc3DGazeDirection(cv::Point2d leftEyeCorner, cv::Point2d rightEyeCorner, CvPoint2D32f iris, CvPoint2D32f EC)
{
  // Coordinates in the AAM_Shape
  // corners of the left eye: 32 and 34
  // corners of the right eye: 27 and 29
  float e2 = iris.x - EC.x;

  // Distance between EyeMid and one corner of the right eye in 2D.
  // If the head is not looking straight forwards to the camera, then this approximation will give even worse results.
  float r2 = 0.0;
  r2 = sqrtf(powf((leftEyeCorner.x - rightEyeCorner.x), 2.0) + powf((leftEyeCorner.y - rightEyeCorner.y), 2.0)) / 2.0;

  // Calculate the horizontal angle of gazedirection
  float deg = (abs(e2) < r2) ? acosf(abs(e2 / r2)) * (float)(180.0 / CV_PI) : 0.0; // 0 means hear 90 degrees rotation

  int quadrant = (e2 > 0) ? 0 : 1;

  switch (quadrant)
  {
  case 0:
    deg = 90.0 - deg;
    break;
  case 1:
    deg = -90.0 + deg;
    break;
  default:
    break;
  }

  m_gaze3D.theta = deg;

  std::ostringstream gaze3DTheta;
  gaze3DTheta << "gaze3DTheta;" << m_gaze3D.theta << std::endl;
  FaceTrackerResources::getGazeDirectionLogFile().writeFile(gaze3DTheta.str());

  // Calculate the vertical angle of gaze direction
  // float gazeVectLen = sqrtf(powf(e2, 2.0) + powf(iris.y - EC.y, 2.0));
  // float degree = acosf(abs(e2 / gazeVectLen)) * (float) (180.0 / CV_PI);
  // or this way:
  float degree = atanf(abs((iris.y - EC.y) / e2)) * (float)(180.0 / CV_PI);

  if (iris.y - EC.y < 0)
    degree = 0.0 - degree;

  m_gaze3D.phi = degree;

  std::ostringstream gaze3DPhi;
  gaze3DPhi << "gaze3DPhiLeft;" << m_gaze3D.phi << std::endl;
  FaceTrackerResources::getGazeDirectionLogFile().writeFile(gaze3DPhi.str());
}


void EyeModel::drawGazeDirection(cv::Mat & img, CvPoint2D32f * eyeCenter, CvPoint2D32f * eyeMid, CvPoint2D32f * iris)
{
  //line from eyecenter to eyemid
  line(img, *eyeCenter, *eyeMid, cv::Scalar(255, 0, 0));

  //line from eyecenter to iris
  line(img, *eyeCenter, *iris, cv::Scalar(0, 0, 255));

  //draw gaze direction from iris to a third point on the augmentation of the line between eyecenter and iris
  float dx = iris->x - eyeCenter->x;
  float dy = iris->y - eyeCenter->y;
  float dxy = sqrtf(powf(dx, 2) + powf(dy, 2));

  float k = sqrtf(powf(dxy * 4, 2) / (powf(dx, 2) + powf(dy, 2)));

  float x3 = eyeCenter->x + dx * k;
  float y3 = eyeCenter->y + dy * k;
  line(img, *iris, cv::Point(x3, y3), cv::Scalar(0, 0, 255), 2);
}

#pragma region TODO: v2.0 Onto which segment does the user look on the screen
//void Gazer::whereDoYouLook(float theta, float phi)
//{
//  int hor = 0;
//  int vert = 0;
//
//  int thetaMin = ((m_calibResult.at(0).x / 2 + m_calibResult.at(1).x / 2) + (m_calibResult.at(5).x / 2 + m_calibResult.at(4).x / 2) + (m_calibResult.at(6).x / 2 + m_calibResult.at(7).x / 2)) / 3;
//  int thetaMax = ((m_calibResult.at(2).x / 2 + m_calibResult.at(1).x / 2) + (m_calibResult.at(3).x / 2 + m_calibResult.at(4).x / 2) + (m_calibResult.at(8).x / 2 + m_calibResult.at(7).x / 2)) / 3;
//
//  if ((theta < thetaMax) && (theta > thetaMin))
//  {
//    vert = 2;
//  }
//  else if (theta > thetaMax)
//  {
//    vert = 3;
//  }
//  else if (theta < thetaMin)
//  {
//    vert = 1;
//  }
//
//  int phiMin = 0.0; // TODO: what is this, shouldn't it be a double or float?
//  int phiMax = 0.0;
//  if (vert != 2)
//  {
//    phiMin = ((m_calibResult.at(0).y / 2 + m_calibResult.at(5).y / 2) + (m_calibResult.at(2).y / 2 + m_calibResult.at(3).y / 2)) / 2;
//    phiMax = ((m_calibResult.at(6).y / 2 + m_calibResult.at(5).y / 2) + (m_calibResult.at(8).y / 2 + m_calibResult.at(3).y / 2)) / 2;
//  }
//  else
//  {
//    phiMin = (m_calibResult.at(1).y / 2 + m_calibResult.at(4).y / 2) / 2;
//    phiMax = (m_calibResult.at(7).y / 2 + m_calibResult.at(4).y / 2) / 2;
//  }
//
//  if ((phi < phiMax) && (phi > phiMin))
//    hor = 2;
//  else if (phi > phiMax)
//    hor = 3;
//  else if (phi < phiMin)
//    hor = 1;
//
//  cv::Mat gazeDir(windowSize.y, windowSize.x, CV_8UC3);
//  int r = 20;
//
//  if ((hor == 1) && (vert == 1))
//    circle(gazeDir, cv::Point(r, r), r, cv::Scalar(0, 0, 255), -1, CV_AA);
//  else if ((hor == 1) && (vert == 2))
//    circle(gazeDir, cv::Point(windowSize.x / 2, r), r, cv::Scalar(0, 0, 255), -1, CV_AA);
//  else if ((hor == 1) && (vert == 3))
//    circle(gazeDir, cv::Point(windowSize.x - r, r), r, cv::Scalar(0, 0, 255), -1, CV_AA);
//  else if ((hor == 2) && (vert == 1))
//    circle(gazeDir, cv::Point(r, windowSize.y / 2), r, cv::Scalar(0, 0, 255), -1, CV_AA);
//  else if ((hor == 2) && (vert == 2))
//    circle(gazeDir, cv::Point(windowSize.x / 2, windowSize.y / 2), r, cv::Scalar(0, 0, 255), -1, CV_AA);
//  else if ((hor == 2) && (vert == 3))
//    circle(gazeDir, cv::Point(windowSize.x - r, windowSize.y / 2), r, cv::Scalar(0, 0, 255), -1, CV_AA);
//  else if ((hor == 3) && (vert == 1))
//    circle(gazeDir, cv::Point(r, windowSize.y - r), r, cv::Scalar(0, 0, 255), -1, CV_AA);
//  else if ((hor == 3) && (vert == 2))
//    circle(gazeDir, cv::Point(windowSize.x / 2, windowSize.y - r), r, cv::Scalar(0, 0, 255), -1, CV_AA);
//  else if ((hor == 3) && (vert == 3))
//    circle(gazeDir, cv::Point(windowSize.x - r, windowSize.y - r), r, cv::Scalar(0, 0, 255), -1, CV_AA);
//
//  // Set the variables to send keystrokes to other programs if needed
//  horizontalIdx = hor;
//  verticalIdx = vert;
//
//  imshow("gaze", gazeDir);
//
//  gazeDir.release();
//  gazeDir.deallocate();
//}
#pragma endregion
