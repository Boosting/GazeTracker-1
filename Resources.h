#pragma once

#include <fstream>
#include <string>
#include <vector>

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\highgui\highgui_c.h>

// TODO: create different resources: e.g. for aam, for filelocations, for resultfiles etc.

namespace Resources
{
  unsigned short const cameraFocalLength = 1000;
  unsigned short const cubeSize = 10;

  std::string const imageFileExtension = ".jpg";
  std::string const dataFileExtension = ".txt";

  class LogFile
  {
    public:
      // Default constructor
      LogFile() :
        m_isOpen(false),
        statFile(nullptr)
      { /* empty */ }

      LogFile(LogFile const & rhs)
      {
        this->m_isOpen = rhs.m_isOpen;
        this->statFile = rhs.statFile; // it cannot be copied
      }

      ~LogFile()
      {
        closeFile();
      }

      void openFile(std::string fileName)
      {
        if ((!m_isOpen) && (fileName != ""))
        {
          statFile->open(fileName);
          m_isOpen = true;
        }
      };

      void closeFile()
      {
        if (m_isOpen)
        {
          statFile->close();
          m_isOpen = false;
        }
      };

      void writeFile(std::string const & s)
      {
        if (m_isOpen)
          *statFile << s;
      };

    private:
      std::ofstream * statFile; // TODO: this thing does not really belong to the resources, or at least not in this form

      bool m_isOpen;
  };


  class ResourcesBase
  {
    public:
      ResourcesBase() = default;

      virtual ~ResourcesBase() = default;

      static void setAppPath(std::string const & app_path)
      {
        // TODO: this seems to be like a bit of a handcrafted stuff, change it with some Qt module
        std::vector<std::string> folders;
        std::istringstream path(app_path);
        std::string folder;

        while (std::getline(path, folder, '/'))
          folders.push_back(folder);

        for (size_t i = 0; i < folders.size() - 1; ++i)
          m_appPath += folders[i] + "/";
      };
      static std::string getAppPath() { return m_appPath; }

      static void setCascade(std::string const & cascadeName) { m_cascade = cascadeName; };
      static std::string getCascade() { return m_cascade; };

      static void setAAMFile(std::string const & aam) { m_aamFile = aam; };
      static std::string getAAMFile() { return m_aamFile; };

      static void setAAMName(std::string const & aam_name) { m_aamName = aam_name; };
      static std::string getAAMName() { return m_aamName; };

      static void setPathToPoints(std::string const & pathToPoints) { m_pointsPath = pathToPoints; };
      static std::string getPathToPoinst() { return m_pointsPath; };

      static void setVideoSource(std::string const & videoSource) { m_videoSource = videoSource; };
      static std::string getVideoSource() { return m_videoSource; };

      static void setImagePath(std::string const & imagePath) { m_imagePath = imagePath; };
      static std::string getImagePath() { return m_imagePath; };

      static void setLogFile(std::string const & log) { m_logFile = log; };
      static std::string getLogFile() { return m_logFile; };

      static void setGazeLogFile(std::string const & log) { m_gazeLogFile = log; };
      static std::string getGazeLogFile() { return m_gazeLogFile; };

      static void setFilesLoaded(bool areFilesLoaded) { m_areFilesLoaded = areFilesLoaded; }
      static bool areFilesLoaded() { return m_areFilesLoaded; }

      static void setHeadPoseLogFile(LogFile headPoseLogFile) { m_headPoseLogFile = headPoseLogFile; }
      static LogFile & getHeadPoseLogFile() { return m_headPoseLogFile; }

      static void setGazeDirectionLogFile(LogFile gazeDirectionLogFile) { m_gazeDirectionLogFile = gazeDirectionLogFile; }
      static LogFile& getGazeDirectionLogFile() { return m_gazeDirectionLogFile; }

      static void setVideoCapture(cv::VideoCapture const & videoCapture) { m_videoCapture = videoCapture; }
      static cv::VideoCapture & getVideoCapture() { return m_videoCapture; }

      static double getHalfPictureWidth() { return m_width; }
      static double getHalfPictureHeight() { return m_height; }

      static double getPictureWidth() { return m_pictureWidth; }
      static double getPictureHeight() { return m_pictureHeight; }

    private:
      static std::string m_cascade;
      static std::string m_aamFile;
      static std::string m_aamName;
      static std::string m_pointsPath;
      static std::string m_videoSource;
      static std::string m_imagePath;
      static std::string m_logFile;
      static std::string m_gazeLogFile;
      static std::string m_appPath;

      static LogFile m_headPoseLogFile;
      static LogFile m_gazeDirectionLogFile;

      static bool m_areFilesLoaded; // Load the important files at the first iteration

      static std::vector<std::string> m_imgFiles;

      static cv::Point m_windowSize;

      static IplImage * m_image; // Image loaded from camera or hdd

      static CvCapture * m_capture; // http://docs.opencv.org/3.1.0/d8/dfe/classcv_1_1VideoCapture.html

      static cv::VideoCapture m_videoCapture;

      static int m_pictureWidth; //  Size of the picture
      static int m_pictureHeight;

      static double m_width;  //  Used for calculations, where the middle of the picture is point (0; 0)
      static double m_height;
  };


  class NonFunctionalRequirements
  {
    public:
      NonFunctionalRequirements() :
        m_counter(0)
      { /* empty */ }

      virtual ~NonFunctionalRequirements() = default;

    private:
      // Variables used for fps calculation
      __time64_t m_start; // TODO: use QElapsedTimer
      __time64_t m_end;
      int m_counter;
  };
}

typedef Resources::ResourcesBase FaceTrackerResources;
