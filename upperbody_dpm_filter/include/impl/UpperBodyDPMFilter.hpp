inline bool pixelsFarerThan(Pixel::pixelUnit nb_pixels, Pixel& p1, Pixel& p2) {
    signed int xdiff = p1.x - p2.x;
    if(xdiff < 0) xdiff = -xdiff;
    signed int ydiff = p1.y - p2.y;
    if(ydiff < 0) ydiff = -ydiff;

    return xdiff > nb_pixels || ydiff > nb_pixels;
}

bool UpperBodyDPMFilter::humanMatchesDPMParts(ground_based_detector::human& person, CandidateArray& _dpmCandidates) {
    dpm_mutex.lock();
    CandidateArray dpmCandidates = _dpmCandidates;
    dpm_mutex.unlock();

    cv::Rect upperBodyRoi(getUpperBodyRoi(person));
    Pixel personTop(Eigen::Vector3f(person.ttop_x, person.ttop_y, person.ttop_z));

    bool foundMatch = false;
    
    for(std::vector<object_recognition_by_parts::Candidate>::iterator candidate = dpmCandidates.candidates.begin(); candidate != dpmCandidates.candidates.end(); ++candidate) {
        Pixel dpmHead(candidate->centers[1].x, candidate->head_top);
        if(pixelsFarerThan(18.75f + (6.25f * person.ttop_z), personTop, dpmHead)) {
            continue;
        }

        int nb_parts_inside_roi = 0;
        for(std::vector<PartCenter>::iterator partCenter = candidate->centers.begin(); partCenter != candidate->centers.end(); ++partCenter) {
            if(Pixel(partCenter->x, partCenter->y).in(upperBodyRoi)) {
                ++nb_parts_inside_roi;
            }
        }
        if(nb_parts_inside_roi == candidate->centers.size()) {
            foundMatch = true;
            break;
        }
    }
    
    return foundMatch;
}

inline void UpperBodyDPMFilter::preventBordersOverflow(cv::Point& topLeft, cv::Point& bottomRight) const {
    if(bottomRight.x > 640) bottomRight.x = 640;
    if(bottomRight.y > 480) bottomRight.y = 480;
    if(topLeft.x < 0 || topLeft.x > 640) topLeft.x = 0;
    if(topLeft.y < 0 || topLeft.y > 480) topLeft.y = 0;
}

inline cv::Rect UpperBodyDPMFilter::getUpperBodyRoi(ground_based_detector::human& person) const {
    Pixel top(Eigen::Vector3f(person.ttop_x, person.ttop_y, person.ttop_z));
    Pixel centroid(Eigen::Vector3f(person.tcenter_x, person.tcenter_y, person.tcenter_z));
    Pixel::pixelUnit hheight = centroid.y - top.y;
    Pixel::pixelUnit hwidth = hheight * 0.25 * 2.0;
    
    cv::Point topLeft(top.x - hwidth, top.y - (0.25 * hheight) );
    cv::Point bottomRight(top.x + hwidth, centroid.y );
    
    preventBordersOverflow(topLeft, bottomRight);
    return cv::Rect(topLeft, bottomRight);
}