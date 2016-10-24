#include "Resources.h"

std::string      Resources::ResourcesBase::m_cascade = "resources//haarcascade_frontalface_alt2.xml";
std::string      Resources::ResourcesBase::m_aamFile = "resources//hpeg_9_b.amf";
std::string      Resources::ResourcesBase::m_aamName = "resources//MyFirstAAM.amf";
std::string      Resources::ResourcesBase::m_pointsPath = "";
std::string      Resources::ResourcesBase::m_videoSource = "resources//9_b.avi";
std::string      Resources::ResourcesBase::m_imagePath = "";
std::string      Resources::ResourcesBase::m_logFile = "";
std::string      Resources::ResourcesBase::m_gazeLogFile = "";
std::string      Resources::ResourcesBase::m_appPath = "";
bool             Resources::ResourcesBase::m_areFilesLoaded = false;
cv::Point        Resources::ResourcesBase::m_windowSize = {1880, 1000};
IplImage *       Resources::ResourcesBase::m_image = nullptr;
CvCapture *      Resources::ResourcesBase::m_capture = nullptr;
cv::VideoCapture Resources::ResourcesBase::m_videoCapture;
Resources::LogFile Resources::ResourcesBase::m_headPoseLogFile;
Resources::LogFile Resources::ResourcesBase::m_gazeDirectionLogFile;
int                Resources::ResourcesBase::m_pictureWidth = 320;
int                Resources::ResourcesBase::m_pictureHeight = 240;
double             Resources::ResourcesBase::m_width = m_pictureWidth / 2.0;
double             Resources::ResourcesBase::m_height = m_pictureHeight / 2.0;
