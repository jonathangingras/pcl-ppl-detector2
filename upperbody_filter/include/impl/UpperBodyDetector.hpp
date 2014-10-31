bool UpperBodyDetector::isHuman(ground_based_detector::human& person) {
    cv::Rect upperBodyRoi(getUpperBodyRoi(person));

    try {
        image_mutex.lock();
        cv::Mat intermediateCroppedMatrix = (*matrix)(upperBodyRoi);
        croppedMatrix = intermediateCroppedMatrix.clone();
        image_mutex.unlock();
    } catch (cv::Exception& error) {
        return false;
    }

    results.clear();

    //struct timespec timer1, timer2;
    //clock_gettime(0, &timer1);
    upperBodyClassifier.detectMultiScale(croppedMatrix, results, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE );
    //clock_gettime(0, &timer2);
    //ROS_ERROR_STREAM(timer2.tv_sec - timer1.tv_sec << " seconds, " << timer2.tv_nsec - timer1.tv_nsec << " nanoseconds.");

    Pixel personHead(Eigen::Vector3f(person.ttop_x, person.ttop_y, person.ttop_z) + Eigen::Vector3f(0,0.2,0));
    personHead.reAnchorOn(upperBodyRoi);

    if(!results.empty()) {
        for(std::vector<cv::Rect>::iterator it = results.begin(); it != results.end(); ++it) {
            //cv::circle(croppedMatrix, personHead.tocvPoint(), 3, cv::Scalar(255, 0, 0), 3);
            //cv::rectangle(croppedMatrix, *it, cv::Scalar(0, 0, 255), 3);
            
            if(personHead.in(*it)) {
                //cv::imshow("openCV Haar-Cascade Upper Body Classifier results", croppedMatrix);
                //cv::waitKey(30);
                return true;
            }
        }
    }
    
    return false;
}

inline void UpperBodyDetector::preventBordersOverflow(cv::Point& topLeft, cv::Point& bottomRight) const {
    if(bottomRight.x > 640) bottomRight.x = 640;
    if(bottomRight.y > 480) bottomRight.y = 480;
    if(topLeft.x < 0 || topLeft.x > 640) topLeft.x = 0;
    if(topLeft.y < 0 || topLeft.y > 480) topLeft.y = 0;
}

inline cv::Rect UpperBodyDetector::getUpperBodyRoi(ground_based_detector::human& person) const {
    Pixel top(Eigen::Vector3f(person.ttop_x, person.ttop_y, person.ttop_z));
    Pixel centroid(Eigen::Vector3f(person.tcenter_x, person.tcenter_y, person.tcenter_z));
    Pixel::pixelUnit hheight = centroid.y - top.y;
    Pixel::pixelUnit hwidth = hheight * 0.25 * 2.0;
    
    cv::Point topLeft(top.x - hwidth, top.y - (0.25 * hheight) );
    cv::Point bottomRight(top.x + hwidth, centroid.y );
    
    preventBordersOverflow(topLeft, bottomRight);
    return cv::Rect(topLeft, bottomRight);
}