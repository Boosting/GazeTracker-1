#pragma once

#include <memory>

#include <opencv2\core\core.hpp>

/**
 * Proxy type for images
 */
class IImage
{

};

class ImageLoader
{
  public:
    ImageLoader() = default;
    virtual ~ImageLoader() = default;

    virtual void init() = 0;

    virtual bool loadImage(std::unique_ptr<cv::Mat> & image) = 0;
};

class VideoImageLoader : public ImageLoader
{
  public:
    VideoImageLoader() = default;
    ~VideoImageLoader() = default;

    virtual void init() override;

    virtual bool loadImage(std::unique_ptr<cv::Mat> & image) override;
};
