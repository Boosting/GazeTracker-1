#include "ImageLoader.h"

#include <iostream>

#include "Resources.h"


void VideoImageLoader::init()
{
  if (!FaceTrackerResources::getVideoCapture().open(FaceTrackerResources::getVideoSource()))
  {
    std::cout << "The video " << FaceTrackerResources::getVideoSource() << " could not be loaded!" << std::endl;
    exit(1);
  }

  FaceTrackerResources::getVideoCapture().set(CV_CAP_PROP_CONVERT_RGB, 1); // TODO: what is this for?
}


bool VideoImageLoader::loadImage(std::unique_ptr<cv::Mat> & image)
{
  if (image == nullptr)
    image.reset(new cv::Mat);

  return FaceTrackerResources::getVideoCapture().read(*image);
}
