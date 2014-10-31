#ifndef __PIXEL_H__
#define __PIXEL_H__

#include <Eigen/Core>
#include <opencv/cv.h>

struct Pixel {
    typedef unsigned short pixelUnit;
    
    pixelUnit x;
    pixelUnit y;

    inline virtual ~Pixel() {}
    
    inline Pixel(pixelUnit _x, pixelUnit _y) : x(_x), y(_y) {}
    
    inline Pixel(const Eigen::Vector3f& realWorldCoordinatePoint, pixelUnit focalRGBDistance = 525, pixelUnit screenWidth = 640 , pixelUnit screenHeigth = 480) {
        x = (pixelUnit)( ( ( realWorldCoordinatePoint(0) * focalRGBDistance) / realWorldCoordinatePoint(2) ) + (screenWidth / 2) );
        y = (pixelUnit)( ( ( realWorldCoordinatePoint(1) * focalRGBDistance) / realWorldCoordinatePoint(2) ) + (screenHeigth / 2) );
    }

    inline Pixel(pixelUnit left, pixelUnit top, pixelUnit right, pixelUnit bottom) {
        pixelUnit xdiff, ydiff;
        xdiff = right - left;
        ydiff = bottom - top;
        x = (xdiff / 2) + left;
        y = (ydiff / 2) + top;
    }

    inline cv::Point tocvPoint() const {
        return cv::Point(x, y);
    }

    inline void reAnchorOn(const cv::Rect& rectangle) {
        x -= rectangle.x;
        y -= rectangle.y;
    }

    inline bool isBetween(const pixelUnit& left, const pixelUnit& top, const pixelUnit& right, const pixelUnit& bottom) const {
        return x >= left && x <= right && y >= top && y <= bottom;
    }

    inline bool isBetween(Pixel topLeft, Pixel bottomRight) const {
        return x >= topLeft.x && x <= bottomRight.x && y >= topLeft.y && y <= bottomRight.y;
    }

    inline bool isBetween(const Pixel& topLeft, const Pixel& bottomRight) const {
        return x >= topLeft.x && x <= bottomRight.x && y >= topLeft.y && y <= bottomRight.y;
    }

    inline bool in(const cv::Rect& rectangle) const {
        return x >= rectangle.x && x <= (rectangle.x + rectangle.width) && y >= rectangle.y && y <= (rectangle.y + rectangle.height);
    }
};

#endif