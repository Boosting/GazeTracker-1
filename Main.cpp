// TODO: const correctness of the methods and members
// TODO: consider threading or task parallelism
// TODO: consider tracing
// TODO: consider a separate thread for the gui
// TODO: define interfaces for resources, aam fitting, posit, other stuff, don't let the FaceTracker be the God class
// TODO: refactor passing by pointer to pass by reference
// TODO: a class for printing onto the m_image
// TODO: a common resource pool, like a shared memory for the images
// TODO: consider the Blackboard pattern
// TODO: consider a COM interface for the whole face and gaze tracking stuff, to be able to consume it from various clients (GUIs)
// TODO: group and assign names to the significant points of the model - e.g. instead of Shape[34] the user could say LeftEye::RightCorner


/***************************************************************************
 *   Copyright (C) 2007 by pedromartins   *
 *   pedromartins@isr.uc.pt   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/


// This define is needed for INPUT in windows.h
#define _WIN32_WINNT 0x0500
#include <windows.h>

#ifndef GL_BGR
  #define GL_BGR 0x80E0
#endif

#include <stdint.h>
#include <stdlib.h>

#include <GL\glut.h>
#include <stdlib.h>
#include <stdio.h>

#include <future>
#include <iostream>
#include <memory>
#include <string>

#include "AAMWrapper.h"
#include "EyeModel.h"
#include "ImageLoader.h"
#include "PositProcessor.h"
#include "Resources.h"

namespace
{
  Resources::ResourcesBase resources;

  VideoImageLoader videoLoader;

  std::unique_ptr<AAMWrapper> aamWrapper;

  std::unique_ptr<PositProcessor> positProcessor;

  std::unique_ptr<Gazer> gazer;

  enum class ImageSource : uint8_t
  {
    IMG_SRC_VIDEO = 0,
    IMG_SRC_IMAGEFILES = 1,
    IMG_SRC_CAMERA = 2
  };

  ImageSource imageSource = ImageSource::IMG_SRC_VIDEO;
}

// TODO: omg, this are prebuilt Delauney-triangles for this specific model. If you wanna do it, do it live with calculations instead this solution.
// This is faster, but this array thing is a mess.
int MeanDelaunayTriangles[95][3] = // TODO: put them out into a file, xml, json, whatever...
{
  {0, 1, 50}, {0, 21, 38}, {0, 28, 21},
  {0, 49, 28}, {0, 50, 49}, {1, 2, 39},
  {1, 39, 50}, {2, 3, 39}, {3, 4, 46},
  {3, 46, 39}, {4, 5, 46}, {5, 6, 45},
  {5, 45, 46}, {6, 7, 44}, {6, 44, 45},
  {7, 8, 44}, {8, 9, 43}, {8, 43, 44},
  {9, 10, 43}, {10, 11, 43}, {11, 12, 54},
  {11, 54, 43}, {12, 13, 20}, {12, 20, 55},
  {12, 33, 13}, {12, 55, 54}, {13, 14, 20},
  {13, 33, 14}, {14, 15, 19}, {14, 19, 20},
  {14, 31, 15}, {14, 32, 31}, {14, 33, 32},
  {15, 16, 19}, {15, 30, 16}, {15, 31, 30},
  {16, 17, 18}, {16, 18, 19}, {16, 29, 17},
  {16, 30, 29}, {17, 29, 57}, {17, 57, 18},
  {18, 56, 19}, {18, 57, 56}, {19, 55, 20},
  {19, 56, 55}, {21, 22, 38}, {21, 28, 22},
  {22, 23, 36}, {22, 27, 23}, {22, 28, 27},
  {22, 36, 37}, {22, 37, 38}, {23, 24, 35},
  {23, 26, 24}, {23, 27, 26}, {23, 35, 36},
  {24, 25, 34}, {24, 26, 25}, {24, 34, 35},
  {25, 26, 48}, {25, 47, 34}, {25, 48, 47},
  {26, 27, 49}, {26, 49, 48}, {27, 28, 49},
  {29, 30, 35}, {29, 34, 57}, {29, 35, 34},
  {30, 31, 36}, {30, 36, 35}, {34, 47, 57},
  {39, 40, 50}, {39, 46, 40}, {40, 41, 52},
  {40, 46, 41}, {40, 51, 50}, {40, 52, 51},
  {41, 42, 52}, {41, 44, 42}, {41, 45, 44},
  {41, 46, 45}, {42, 43, 53}, {42, 44, 43},
  {42, 53, 52}, {43, 54, 53}, {47, 48, 57},
  {48, 49, 51}, {48, 51, 52}, {48, 52, 56},
  {48, 56, 57}, {49, 50, 51}, {52, 53, 56},
  {53, 54, 55}, {53, 55, 56}
};


// This method draws the model onto the screen
// This need not to be supported in v1.0
void Anthropometric3DModel(int LineWidth)
{
  //glColor3f(0.0, 0.0, 0.0);
  //glLineWidth(LineWidth);

  //glPushMatrix();
  //  glBegin(GL_LINES);
  //    for(uint8_t k = 0; k < 95; ++k)
  //    {
  //      glVertex3f(g_faceTracker->Model3D[MeanDelaunayTriangles[k][0]][0], g_faceTracker->Model3D[MeanDelaunayTriangles[k][0]][1], g_faceTracker->Model3D[MeanDelaunayTriangles[k][0]][2]);
  //      glVertex3f(g_faceTracker->Model3D[MeanDelaunayTriangles[k][1]][0], g_faceTracker->Model3D[MeanDelaunayTriangles[k][1]][1], g_faceTracker->Model3D[MeanDelaunayTriangles[k][1]][2]);

  //      glVertex3f(g_faceTracker->Model3D[MeanDelaunayTriangles[k][1]][0], g_faceTracker->Model3D[MeanDelaunayTriangles[k][1]][1], g_faceTracker->Model3D[MeanDelaunayTriangles[k][1]][2]);
  //      glVertex3f(g_faceTracker->Model3D[MeanDelaunayTriangles[k][2]][0], g_faceTracker->Model3D[MeanDelaunayTriangles[k][2]][1], g_faceTracker->Model3D[MeanDelaunayTriangles[k][2]][2]);

  //      glVertex3f(g_faceTracker->Model3D[MeanDelaunayTriangles[k][2]][0], g_faceTracker->Model3D[MeanDelaunayTriangles[k][2]][1], g_faceTracker->Model3D[MeanDelaunayTriangles[k][2]][2]);
  //      glVertex3f(g_faceTracker->Model3D[MeanDelaunayTriangles[k][0]][0], g_faceTracker->Model3D[MeanDelaunayTriangles[k][0]][1], g_faceTracker->Model3D[MeanDelaunayTriangles[k][0]][2]);
  //    }
  //  glEnd();

  //  glColor3f(1.0, 0.0, 0.0);
  //  glPointSize(LineWidth+2);

  //  glBegin(GL_POINTS);
  //    for (uint8_t k = 0; k < 58; ++k)
  //    {
  //      glVertex3f(g_faceTracker->Model3D[k][0], g_faceTracker->Model3D[k][1], g_faceTracker->Model3D[k][2]);
  //    }
  //  glEnd();

  //glPopMatrix();
}


struct Coordinates
{
  Coordinates() :
    x(0), y(0), z(0), orientation{ 0, 0, 0 }
  { /* empty */ }

  GLfloat x;
  GLfloat y;
  GLfloat z;

  struct Orientation
  {
    GLfloat roll;
    GLfloat pitch;
    GLfloat yaw;
  } orientation;
}
WorldCoords,
FaceCoords;


// This method draws the calculated 3D points of the model onto the screen
// This need not to be supported in v1.0
void draw3DModel(int LineWidth)
{
  //glColor3f(0.0, 0.0, 0.0);
  //glLineWidth(LineWidth);

  //glPushMatrix();
  //  glBegin(GL_LINES);
  //      glVertex3f(g_faceTracker->getPoints3D()[16].x, g_faceTracker->getPoints3D()[16].y, g_faceTracker->getPoints3D()[16].z);
  //      glVertex3f(g_faceTracker->getPoints3D()[17].x, g_faceTracker->getPoints3D()[17].y, g_faceTracker->getPoints3D()[17].z);
  //      glVertex3f(g_faceTracker->getPoints3D()[14].x, g_faceTracker->getPoints3D()[14].y, g_faceTracker->getPoints3D()[14].z);
  //      glVertex3f(g_faceTracker->getPoints3D()[15].x, g_faceTracker->getPoints3D()[15].y, g_faceTracker->getPoints3D()[15].z);
  //  glEnd();
  //glPopMatrix();

  //glPushMatrix();
  //  glColor3f(0.0, 1.0, 0.0);
  //  glPointSize(LineWidth+2);

  //  glBegin(GL_POINTS);

  //    for (size_t k = 0; k < g_faceTracker->getPoints3D().size(); ++k)
  //      glVertex3f(g_faceTracker->getPoints3D()[k].x, g_faceTracker->getPoints3D()[k].y, g_faceTracker->getPoints3D()[k].z);

  //  glEnd();
  //glPopMatrix();
}


GLUquadricObj * Quadric=gluNewQuadric();

// This draws the axes of the eyes
void DrawAxes(float Size, float Radius)
{
  glPushMatrix();

    //Draw z axis
    glColor3f(0.0, 0.0, 1.0);
    glPushMatrix();
    //gluCylinder(Quadric, Radius, Radius, Size, 40, 40);
    glTranslatef(0, 0, Size);
    //gluCylinder(Quadric, Radius*2, 0, 1, 40, 40);
    glPopMatrix();

    //Draw x axis
    glColor3f(1.0, 0.0, 0.0);
    glPushMatrix();
    glRotatef(90, 0, 1, 0);  //rotate over y 90º
    gluCylinder(Quadric, Radius, Radius, Size, 40, 40);
    glTranslatef(0, 0, Size);
    gluCylinder(Quadric, Radius*2, 0, 1, 40, 40);
    glPopMatrix();

    //Draw y axis
    glColor3f(0.0, 1.0, 0.0);
    glPushMatrix();
    glRotatef(-90, 1, 0, 0);  //rotate over x -90º
    gluCylinder(Quadric, Radius, Radius, Size, 40, 40);
    glTranslatef(0, 0, Size);
    gluCylinder(Quadric, Radius*2, 0, 1, 40, 40);
    glPopMatrix();

  glPopMatrix();
}


void Draw_Z_Axis(float Size, float Radius)
{
//  // draw z axis from the middle of the right eye
//  glPushMatrix();
//    glColor3f(0.0, 0.0, 1.0);
//
//    glTranslatef(g_faceTracker->getRightEyeMid().x, g_faceTracker->getRightEyeMid().y, g_faceTracker->getRightEyeMid().z);
//    glTranslatef(0, 0, -g_faceTracker->getRightOffsetLen());
//
//    gluCylinder(Quadric, Radius, Radius, Size, 40, 40);
//    glTranslatef(0, 0, Size);
//    gluCylinder(Quadric, Radius*2, 0, 1, 40, 40);
//  glPopMatrix();
//
//    //Draw x axis
//    glPushMatrix();
//    glColor3f(1.0, 0.0, 0.0);
//    glTranslatef(g_faceTracker->getRightEyeMid().x, g_faceTracker->getRightEyeMid().y, g_faceTracker->getRightEyeMid().z);
//    glTranslatef(0, 0, -g_faceTracker->getRightOffsetLen());
//
//    glRotatef(90, 0, 1, 0);  //rotate over y 90º
//    gluCylinder(Quadric, Radius, Radius, Size-5, 40, 40);
//    glTranslatef(0, 0, Size-5);
//    gluCylinder(Quadric, Radius*2, 0, 1, 40, 40);
//    glPopMatrix();
//
//    //Draw y axis
//    glPushMatrix();
//    glColor3f(0.0, 1.0, 0.0);
//    glTranslatef(g_faceTracker->getRightEyeMid().x, g_faceTracker->getRightEyeMid().y, g_faceTracker->getRightEyeMid().z);
//    glTranslatef(0, 0, -g_faceTracker->getRightOffsetLen());
//
//    glRotatef(-90, 1, 0, 0);  //rotate over x -90º
//
//    gluCylinder(Quadric, Radius, Radius, Size-5, 40, 40);
//    glTranslatef(0, 0, Size-5);
//    gluCylinder(Quadric, Radius*2, 0, 1, 40, 40);
//    glPopMatrix();
//
//
//  //draw right eyeball center
//  glPushMatrix();
//    glColor3f(0.0, 1.0, 0.0);
//    glPointSize(5);
//
//    glTranslatef(g_faceTracker->getRightEyeMid().x, g_faceTracker->getRightEyeMid().y, g_faceTracker->getRightEyeMid().z);
//    glTranslatef(0, 0, -g_faceTracker->getRightOffsetLen());
//
//
//    glBegin(GL_POINTS);
//      glVertex3f(0, 0, 0);
//    glEnd();
//  glPopMatrix();
//
//  //draw middle of right eye
//  glPushMatrix();
//    glColor3f(1.0, 0.0, 0.0);
//    glPointSize(5);
//
//    glBegin(GL_POINTS);
//      glVertex3f(g_faceTracker->getRightEyeMid().x, g_faceTracker->getRightEyeMid().y, g_faceTracker->getRightEyeMid().z);
//    glEnd();
//  glPopMatrix();
//
////-------------------------------------
//
//  // draw z axis from the middle of the left eye
//  glPushMatrix();
//    glColor3f(0.0, 0.0, 1.0);
//
//    glTranslatef(g_faceTracker->getLeftEyeMid().x, g_faceTracker->getLeftEyeMid().y, g_faceTracker->getLeftEyeMid().z);
//    glTranslatef(0, 0, - g_faceTracker->getLeftOffsetLen());
//
//    gluCylinder(Quadric, Radius, Radius, Size, 40, 40);
//    glTranslatef(0, 0, Size);
//    gluCylinder(Quadric, Radius*2, 0, 1, 40, 40);
//  glPopMatrix();
//
//  //draw left eyeball center
//  glPushMatrix();
//    glColor3f(0.0, 1.0, 0.0);
//    glPointSize(5);
//
//    glTranslatef(g_faceTracker->getLeftEyeMid().x, g_faceTracker->getLeftEyeMid().y, g_faceTracker->getLeftEyeMid().z);
//    glTranslatef(0, 0, - g_faceTracker->getLeftOffsetLen());
//
//    glBegin(GL_POINTS);
//      glVertex3f(0, 0, 0);
//    glEnd();
//  glPopMatrix();
//
//  //draw middle of the left eye
//  glPushMatrix();
//    glColor3f(1.0, 0.0, 0.0);
//    glPointSize(5);
//
//    glBegin(GL_POINTS);
//      glVertex3f(g_faceTracker->getLeftEyeMid().x, g_faceTracker->getLeftEyeMid().y, g_faceTracker->getLeftEyeMid().z);
//    glEnd();
//  glPopMatrix();
}


void DrawGazeDirectionVector(float Size, float Radius)
{
  //// draw z axis from the middle of the right eye
  //glPushMatrix();
  //  glColor3f(0.5, 0.5, 1.0);

  //  glTranslatef(g_faceTracker->getRightEyeMid().x, g_faceTracker->getRightEyeMid().y, g_faceTracker->getRightEyeMid().z);
  //  glTranslatef(0, 0, -g_faceTracker->getRightOffsetLen());

  //  glRotatef(g_faceTracker->getGaze3DTheta(), 0, 1, 0);
  //  glRotatef(g_faceTracker->getGaze3DPhi(), 1, 0, 0);

  //  gluCylinder(Quadric, Radius, Radius, Size, 40, 40);
  //  glTranslatef(0, 0, Size);
  //  gluCylinder(Quadric, Radius*2, 0, 1, 40, 40);
  //glPopMatrix();

  //// draw z axis from the middle of the right eye
  //glPushMatrix();
  //  glColor3f(0.5, 0.5, 1.0);

  //  glTranslatef(g_faceTracker->getLeftEyeMid().x, g_faceTracker->getLeftEyeMid().y, g_faceTracker->getLeftEyeMid().z);
  //  glTranslatef(0, 0, -g_faceTracker->getLeftOffsetLen());

  //  glRotatef(g_faceTracker->getGaze3DTheta(), 0, 1, 0);
  //  glRotatef(g_faceTracker->getGaze3DPhi(), 1, 0, 0);

  //  gluCylinder(Quadric, Radius, Radius, Size, 40, 40);
  //  glTranslatef(0, 0, Size);
  //  gluCylinder(Quadric, Radius*2, 0, 1, 40, 40);
  //glPopMatrix();
}


//OpenGL Display Routine
void display(void)
{
  glClear(GL_COLOR_BUFFER_BIT);

  glPushMatrix();
    //glRotatef(g_faceTracker->getRollSave(), 0, 0, 1);
    //glRotatef(g_faceTracker->getPitchSave(), 1, 0, 0);
    //glRotatef(g_faceTracker->getYawSave(), 0, 1, 0);

    //if (g_faceTracker->getPoints3D().size() > 0)
    //{
    //  draw3DModel(3);
    //  Draw_Z_Axis(8, 0.1f);
    //  DrawGazeDirectionVector(4, 0.1f);
    //}
  glPopMatrix();

  glPushMatrix();

    glTranslatef(FaceCoords.x, FaceCoords.y, FaceCoords.z);

    //glRotatef(FaceCoords.roll, 0, 0, 1);
    //glRotatef(FaceCoords.pitch, 1, 0, 0);
    //glRotatef(FaceCoords.yaw, 0, 1, 0);

    // The next function draws the original model onto the object, but it is time consuming
    //Anthropometric3DModel(3);
    //DrawAxes(5, 0.1);

  glPopMatrix();

  glutSwapBuffers();
}


int sendKeystrokesToAcrobatReader(bool Up)
{
  INPUT * keystroke;
  UINT i, character_count, keystrokes_to_send, keystrokes_sent;
  HWND adobe;

  //Get the handle of the Adobe Reader window.
  adobe = FindWindow(L"AcrobatSDIWindow", NULL);
  if(adobe == NULL)
      return 0;

  //Bring the Adobe Reader window to the front.
  if(!SetForegroundWindow(adobe))
      return 0;

  //Fill in the array of keystrokes to send.
  character_count = 1; //wcslen(text);
  keystrokes_to_send = character_count * 2;
  keystroke = new INPUT[ keystrokes_to_send ];
  for(i = 0; i < character_count; ++i)
  {
    keystroke[ i * 2 ].type = INPUT_KEYBOARD;
    keystroke[ i * 2 ].ki.wVk = (Up) ? VK_UP : VK_DOWN;
    keystroke[ i * 2 ].ki.wScan = 0;
    keystroke[ i * 2 ].ki.dwFlags = KEYEVENTF_EXTENDEDKEY;
    keystroke[ i * 2 ].ki.time = 0;
    keystroke[ i * 2 ].ki.dwExtraInfo = GetMessageExtraInfo();

    keystroke[ i * 2 + 1 ].type = INPUT_KEYBOARD;
    keystroke[ i * 2 + 1 ].ki.wVk = (Up) ? VK_UP : VK_DOWN;
    keystroke[ i * 2 + 1 ].ki.wScan = 0;
    keystroke[ i * 2 + 1 ].ki.dwFlags = KEYEVENTF_EXTENDEDKEY | KEYEVENTF_KEYUP;
    keystroke[ i * 2 + 1 ].ki.time = 0;
    keystroke[ i * 2 + 1 ].ki.dwExtraInfo = GetMessageExtraInfo();
  }

  //Send the keystrokes.
  keystrokes_sent = SendInput((UINT)keystrokes_to_send, keystroke, sizeof(*keystroke));
  delete [] keystroke;

  return keystrokes_sent == keystrokes_to_send;
}


//OpenGL Idle Function
void idle(void)
{
  switch (imageSource)
  {
    case ImageSource::IMG_SRC_VIDEO:
    {
      // Step 1: load an image
      std::unique_ptr<cv::Mat> image(new cv::Mat);
      if (!videoLoader.loadImage(image))
        return;

      // Step 2: scaling the image
      cv::resize(*image, *image, cv::Size(320, 240));

      // Step 3: copy it into an IplImage, because AAM fitting works only with that format
      // TODO: omg: do we really need the IplImage format?
      std::unique_ptr<IplImage> iplImage(new IplImage(*image));

      aamWrapper->fitAAMToImage(*iplImage);

      // Step 4: POSIT
      positProcessor->glDoAamPositFromAvi(aamWrapper->getShape(), *image);

      std::vector<CvPoint3D32f> const & points3D = positProcessor->getPoints3D();

      // Step 5: detect the irises
      std::vector<CvPoint2D32f> leftIris;
      std::vector<CvPoint2D32f> rightIris;
      auto eyeBlobLeftFuture = std::async(std::launch::async, [&]() { aamWrapper->detectEyeBlob(*image, leftIris, "left"); });
      auto eyeBlobRightFuture = std::async(std::launch::async, [&]() { aamWrapper->detectEyeBlob(*image, rightIris, "right"); });
      eyeBlobLeftFuture.get();
      eyeBlobRightFuture.get();

      // Step 6: calculate 2D gaze direction
      aamWrapper->getEyes().left.calcEyeMidAndOffsetLen(points3D, cv::Point(16, 17));
      aamWrapper->getEyes().right.calcEyeMidAndOffsetLen(points3D, cv::Point(14, 15));

      // Project the eyeballcenters and eyemids onto the 2D m_image
      CvPoint3D32f faceOrientation =
      {
        positProcessor->getFaceOrientation().roll,
        positProcessor->getFaceOrientation().pitch,
        positProcessor->getFaceOrientation().yaw
      };

      CvPoint2D32f point2DLE = cvPoint2D32f(0.0, 0.0);  // 2D position of the left EyeBallCenter
      CvPoint2D32f point2DLEM = cvPoint2D32f(0.0, 0.0); // 2D position of the middle of the left Eye
      aamWrapper->getEyes().left.calc2DEyeMidAndCenter(faceOrientation, positProcessor->getTranslationVector(), point2DLE, point2DLEM);

      CvPoint2D32f point2DRE = cvPoint2D32f(0.0, 0.0);  // 2D position of the right EyeBallCenter
      CvPoint2D32f point2DREM = cvPoint2D32f(0.0, 0.0); // 2D position of the middle of the right Eye
      aamWrapper->getEyes().right.calc2DEyeMidAndCenter(faceOrientation, positProcessor->getTranslationVector(), point2DRE, point2DREM);

      ////2D point of LeftEyeCenter on the picture
      CvPoint2D32f LEC = cvPoint2D32f(point2DLE.x + FaceTrackerResources::getHalfPictureWidth(), FaceTrackerResources::getHalfPictureHeight() - point2DLE.y);
      circle(*image, LEC, 1, cv::Scalar(255, 0, 255), CV_FILLED);

      //2D point of LeftEyeMiddle on the picture
      CvPoint2D32f LEM = cvPoint2D32f(point2DLEM.x + FaceTrackerResources::getHalfPictureWidth(), FaceTrackerResources::getHalfPictureHeight() - point2DLEM.y);
      circle(*image, LEM, 1, cv::Scalar(255, 0, 0), CV_FILLED);

      //2D point of RightEyeCenter on the picture
      CvPoint2D32f REC = cvPoint2D32f(point2DRE.x + FaceTrackerResources::getHalfPictureWidth(), FaceTrackerResources::getHalfPictureHeight() - point2DRE.y);
      circle(*image, REC, 1, cv::Scalar(255, 0, 255), CV_FILLED);

      //2D point of RightEyeMiddle on the picture
      CvPoint2D32f REM = cvPoint2D32f(point2DREM.x + FaceTrackerResources::getHalfPictureWidth(), FaceTrackerResources::getHalfPictureHeight() - point2DREM.y);
      circle(*image, REM, 1, cv::Scalar(255, 0, 0), CV_FILLED);

      if (rightIris.size() > 0)
      {
        aamWrapper->getEyes().right.drawGazeDirection(*image, &REC, &REM, &rightIris[0]);
      }

      if (leftIris.size() > 0)
      {
        aamWrapper->getEyes().left.drawGazeDirection(*image, &LEC, &LEM, &leftIris[0]);
      }

      // Step 7: calculate 3D gaze direction
      //if (leftIris.size() > 0)
      //{
      //  aamWrapper->getEyes().left.calc3DGazeDirection(aamWrapper->getShape()[32], aamWrapper->getShape()[34], leftIris[0], LEC);
      //}

      //if (rightIris.size() > 0)
      //{
      //  aamWrapper->getEyes().right.calc3DGazeDirection(aamWrapper->getShape()[27], aamWrapper->getShape()[29], rightIris[0], REC);
      //}

      imshow("GazeTracker", *image);

      // TODO: do we need it, it is already in the OpenGL code
      //if (cvWaitKey(1) == 27) // Esc pressed
      //  cvWaitKey(0);

      // TODO: calculate the average of the measured gaze directions, but not here
      //avgTheta = 0.0, avgPhi = 0.0;
      //if ((leftIris.size() > 0) && (rightIris.size() > 0))
      //{
      //  avgTheta = (FaceTrackerResources.leftEye.gaze3DTheta + FaceTrackerResources.rightEye.gaze3DTheta) / 2.0;
      //  avgThetaSave = avgTheta;
      //  avgPhi = (FaceTrackerResources.leftEye.gaze3DPhi + FaceTrackerResources.rightEye.gaze3DPhi) / 2.0;
      //  avgPhiSave = avgPhi;
      //}
      //else if ((leftIris.size() > 0) && (rightIris.size() == 0))
      //{
      //  avgTheta = FaceTrackerResources.leftEye.gaze3DTheta;
      //  avgThetaSave = avgTheta;
      //  avgPhi = FaceTrackerResources.leftEye.gaze3DPhi;
      //  avgPhiSave = avgPhi;
      //}
      //else if ((leftIris.size() == 0) && (rightIris.size() > 0))
      //{
      //  avgTheta = FaceTrackerResources.rightEye.gaze3DTheta;
      //  avgThetaSave = avgTheta;
      //  avgPhi = FaceTrackerResources.rightEye.gaze3DPhi;
      //  avgPhiSave = avgPhi;
      //}
      //else
      //{
      //  if (avgThetaSave != 500.0)
      //    avgTheta = avgThetaSave;

      //  if (avgPhiSave != 500.0)
      //    avgPhi = avgPhiSave;
      //}

      //// Log the gaze direction
      //std::ostringstream gazeData;
      //gazeData << "avg_Theta_Phi" << ";" << avgTheta << ";" << avgPhi << std::endl;
      //FaceTrackerResources.gazeDirectionLogFile.writeFile(gazeData.str());
      break;
    }

    case ImageSource::IMG_SRC_IMAGEFILES:
    {
      // TODO: v2.0
      //glDoAamPosit();
      break;
    }

    case ImageSource::IMG_SRC_CAMERA:
    {
      // TODO: v2.0
      //glDoAamPositWithCamera();
      break;
    }

    default:
      break;
  }

  FaceCoords.orientation.yaw += positProcessor->getFaceOrientation().yaw;
  FaceCoords.orientation.pitch += positProcessor->getFaceOrientation().pitch;
  FaceCoords.orientation.roll += positProcessor->getFaceOrientation().roll;

  //FaceTx += FaceTx1;
  //FaceTy += FaceTy1;
  //FaceTz += FaceTz1;

  // TODO: v4.0
  //// Send VK_UP or VK_DOWN to Adobe Reader,
  //// when the user looks towards the right upper or lower corner of the screen
  //if ((getHorizontalIdx() == 1) && (getVerticalIdx() == 3))
  //  sendKeystrokesToAcrobatReader(true);
  //else if ((getHorizontalIdx() == 3) && (getVerticalIdx() == 3))
  //  sendKeystrokesToAcrobatReader(false);

  //glutPostRedisplay is important to draw face with the newly calculated positions, but it consumes CPU time
  glutPostRedisplay();
}


void visible(int vis)
{
  if (vis == GLUT_VISIBLE)
    glutIdleFunc(idle);
  else
    glutIdleFunc(NULL);
}


void keyboardHandler(unsigned char Key, int x, int y)
{
  switch(Key)
  {
    case 27:
      exit(0); break;

    case 'r': WorldCoords.orientation.roll+=5; break;
    case 'R': WorldCoords.orientation.roll-=5; break;

    case 'p': WorldCoords.orientation.pitch += 5; break;
    case 'P': WorldCoords.orientation.pitch-=5; break;

    case 'y': WorldCoords.orientation.yaw += 5; break;
    case 'Y': WorldCoords.orientation.yaw-=5; break;


    case '7': FaceCoords.orientation.roll += 2; break;
    case '1': FaceCoords.orientation.roll-=2; break;

    case '9': FaceCoords.orientation.pitch += 2; break;
    case '3': FaceCoords.orientation.pitch-=2; break;

    case '/': FaceCoords.orientation.yaw += 2; break;
    case '*': FaceCoords.orientation.yaw-=2; break;

    case '6': FaceCoords.x+=0.5; break;
    case '4': FaceCoords.x-=0.5; break;

    case '8': FaceCoords.y+=0.5; break;
    case '2': FaceCoords.y-=0.5; break;

    case '+': FaceCoords.z+=0.5; break;
    case '-': FaceCoords.z-=0.5; break;

    case 's': break;
  }

  glRotatef(WorldCoords.orientation.roll, 0, 0, 1);
  glRotatef(WorldCoords.orientation.pitch, 0, 1, 0);
  glRotatef(WorldCoords.orientation.yaw, 1, 0, 0);

  WorldCoords.orientation.roll = 0;
  WorldCoords.orientation.pitch=0;
  WorldCoords.orientation.yaw = 0;

  glutPostRedisplay();
}


void specialKeyboardHandler(int Key, int x, int y)
{
  switch(Key){

  case GLUT_KEY_RIGHT: WorldCoords.x+=0.5; break;
  case GLUT_KEY_LEFT: WorldCoords.x-=0.5; break;
  case GLUT_KEY_UP: WorldCoords.y+=0.5; break;
  case GLUT_KEY_DOWN: WorldCoords.y-=0.5; break;
  case GLUT_KEY_PAGE_UP: WorldCoords.z+=0.5; break;
  case GLUT_KEY_PAGE_DOWN: WorldCoords.z-=0.5; break;
  }

  glTranslatef(WorldCoords.x, WorldCoords.y, WorldCoords.z);

  WorldCoords.x=0;
  WorldCoords.y=0;
  WorldCoords.z=0;

  glutPostRedisplay();
}


//Reshape Window Handler
void reshapeWindow(GLsizei w, GLsizei h)
{
  glViewport(0, 0, w, h);

  glMatrixMode (GL_PROJECTION);
  glLoadIdentity();

  gluPerspective(60.0, (GLfloat) w/(GLfloat) h, 1.0, 200.0);

  //Using Orthographic Projection
  //glOrtho (0, w, 0, h, -5.0, 5.0);

  glMatrixMode(GL_MODELVIEW);
  //glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glTranslatef(0.0, 0.0, -25);


  //glGetDoublev(GL_MODELVIEW_MATRIX, R);
}


//Set Texture Mapping Parameters
void init(void){

  //Clear The Background Color
  glClearColor(1.0f, 1.0f, 1.0f, 0.0f);

  glShadeModel(GL_SMOOTH);
}


int main(int argc, char* argv[])
{
  if (!FaceTrackerResources::areFilesLoaded()) // Load the important files just the first time
  {
    // Initializing the window for the output m_image
    //cvNamedWindow("img", CV_WINDOW_AUTOSIZE);
    //cvMoveWindow("img", 0, 0);

    // TODO: v3.0 but somewhere else, not here
    //createTrackBars("img");

    // Opening a file for saving statistic data
    FaceTrackerResources::getHeadPoseLogFile().openFile(FaceTrackerResources::getLogFile());
    FaceTrackerResources::getHeadPoseLogFile().writeFile("Yaw;Pitch;Roll;Distance\n");

    FaceTrackerResources::getGazeDirectionLogFile().openFile(FaceTrackerResources::getGazeLogFile());

    FaceTrackerResources::setFilesLoaded(true);
  }

  videoLoader.init();

  aamWrapper = std::unique_ptr<AAMWrapper>(new AAMWrapper());
  aamWrapper->initVJAndAAM();

  positProcessor = std::unique_ptr<PositProcessor>(new PositProcessor());

  gazer = std::unique_ptr<Gazer>(new Gazer);

  glutInit(&argc, argv);

  // TODO: refactor this with Qt argument parser or gperf
  // Parse command line arguments
  if (argc == 1)
  {
    std::cout << "------" << std::endl;
    std::cout << "Usage: " << std::endl;
    std::cout << "------" << std::endl;

    std::cout << "-logh FILENAME" << std::endl;
    std::cout << "    Writes the headpose into file" << std::endl << std::endl;

    std::cout << "-logg FILENAME" << std::endl;
    std::cout << "    Writes the gazedirection into file" << std::endl << std::endl;

    std::cout << "-v PATH/FILENAME [-aam AAM_FILE] [-cf CASCADE_FILE]" << std::endl;
    std::cout << "    Reads the images from a video (avi)" << std::endl << std::endl;

    std::cout << "-p IMAGEPATH [-aam AAM_FILE] [-cf CASCADE_FILE]" << std::endl;
    std::cout << "    Reads the images from JPG files" << std::endl << std::endl;

    std::cout << "-c [-aam AAM_FILE] [-cf CASCADE_FILE]"<< std::endl;
    std::cout << "    Reads the images from camera source (Default)" << std::endl << std::endl;

    std::cout << "-build 'PATH of images and landmarkpoints' -n AAM_NAME  [-cf CASCADE_FILE]" << std::endl;
    std::cout << "    Builds an AAM with the given AAM_NAME and stores it in bin directory" << std::endl << std::endl;

    std::cout << "-aam AAM_NAME" << std::endl;
    std::cout << "    Reads an AAM from file, path must be relative to bin directory" << std::endl << std::endl;

    std::cout << "-n AAM_NAME" << std::endl;
    std::cout << "    Sets the name for a new model, use with -build. Example: -n MyFirstAAM" << std::endl << std::endl;

    std::cout << "-cf CASCADEPATH/FILENAME" << std::endl;
    std::cout << "    Reads a Cascade-file, path must be relative to bin directory" << std::endl << std::endl;
  }
  else
  {
    std::string const str = reinterpret_cast<char *>(argv[0]);
    resources.setAppPath(str);
  }

  for (uint8_t i = 1; i < argc; i++)
  {
    std::cout << short(i) << ": " << argv[i] << std::endl;

    if (strcmp(argv[i], "-logh") == 0)      // Writes the headpose into file
    {                        // Usage: -logh FILENAME
      std::string const str = reinterpret_cast<char *>(argv[i+1]);
      resources.setLogFile(str);
    }
    else if (strcmp(argv[i], "-logg") == 0)    // Writes the gazedirection into file
    {                        // Usage: -logg FILENAME
      std::string const str = reinterpret_cast<char *>(argv[i+1]);
      resources.setGazeLogFile(str);
    }
    else if (strcmp(argv[i], "-v") == 0)    // Reads the images from a video
    {                        // Usage: -v PATH/FILENAME
      imageSource = ImageSource::IMG_SRC_VIDEO;

      std::string const str = reinterpret_cast<char *>(argv[i+1]);
      resources.setVideoSource(str);
    }
    else if (strcmp(argv[i], "-p") == 0)    // Reads the images from .jpg files
    {                        // Usage: -p IMAGEPATH
      imageSource = ImageSource::IMG_SRC_IMAGEFILES;

      std::string const str = reinterpret_cast<char *>(argv[i+1]);
      resources.setImagePath(str);
    }
    else if (strcmp(argv[i], "-c") == 0)    // Reads the images from camera source
    {                        // Usage: -c
      imageSource = ImageSource::IMG_SRC_CAMERA;
    }
    else if (strcmp(argv[i], "-aam") == 0)    // Reads an AAM from file
    {                        // Usage: -aam AAMPATH/FILENAME
      std::string const str = reinterpret_cast<char *>(argv[i+1]);
      resources.setAAMFile(str);
    }
    else if (strcmp(argv[i], "-n") == 0)    // Reads the images from camera source
    {                        // Usage: -n AAM_NAME
      std::string const str = reinterpret_cast<char *>(argv[i+1]);
      resources.setAAMName(str);
    }
    else if (strcmp(argv[i], "-cf") == 0)    // Reads a Cascade-file
    {                        // Usage: -cf CASCADEPATH/FILENAME
      std::string const str = reinterpret_cast<char *>(argv[i+1]);
      resources.setCascade(str);
    }
    else if (strcmp(argv[i], "-build") == 0)  // Builds an AAM
    {                        // Usage: -build 'path of images and landmarkpoints' -n AAM_NAME [-cf AAM_NAME CASCADE_FILE]
      //aamHandler.buildAAM(); // This is not supported in the v1.0

      std::string const str = reinterpret_cast<char *>(argv[i+1]);
      resources.setPathToPoints(str);
      exit(EXIT_SUCCESS);
    }
  }

  //Init OpenGL With Double Buffer in RGB Mode
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

  glutInitWindowSize(800, 600);
  glutInitWindowPosition(500, 200);

  glutCreateWindow("3D Model");

  //Set Display Handler
  glutDisplayFunc(display);

  //Set Keyboard Handler
  glutKeyboardFunc(keyboardHandler);
  glutSpecialFunc(specialKeyboardHandler);

  glutReshapeFunc(reshapeWindow);
  glutVisibilityFunc(visible);

  init();

  //OpenGL Main Loop
  glutMainLoop();

  return 0;
}
