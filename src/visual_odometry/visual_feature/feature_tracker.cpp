#include "feature_tracker.h"
#include <opencv2/core/types.hpp>

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


FeatureTracker::FeatureTracker()
{
}

void FeatureTracker::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_left_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt_left[i], make_pair(forw_left_pts[i], ids_left[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    forw_left_pts.clear();
    ids_left.clear();
    track_cnt_left.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_left_pts.push_back(it.second.first);
            ids_left.push_back(it.second.second);
            track_cnt_left.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::addPoints()
{
    for (auto &p : new_tracked_points_left)
    {
        forw_left_pts.push_back(p);
        ids_left.push_back(-1);
        track_cnt_left.push_back(1);
    }
    for (auto& p : new_tracked_points_right) {
        forw_right_pts.push_back(p);
        ids_right.push_back(-1);
        track_cnt_right.push_back(1);
    }

    new_tracked_points_left.clear();
    new_tracked_points_right.clear();
}

void FeatureTracker::readImage(const cv::Mat &left_img, const cv::Mat& right_img, double _cur_time)
{
    cv::Mat left_img_, right_img_;
    TicToc t_r;
    cur_time = _cur_time;

    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(left_img, left_img_);
        clahe->apply(right_img, right_img_);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else {
        left_img_ = left_img;
        right_img_ = right_img;
    }

    if (forw_left_img.empty())
    {
        prev_left_img = cur_left_img = forw_left_img = left_img_;
        prev_right_img = cur_right_img = forw_right_img = right_img_;
    }
    else
    {
        forw_left_img = left_img_;
        forw_right_img = right_img_;
    }

    forw_left_pts.clear();
    forw_right_pts.clear();

    if (cur_left_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status_left, status_right;
        vector<float> err_left, err_right;

        // track previous features from both images
        cv::calcOpticalFlowPyrLK(cur_left_img, forw_left_img, cur_left_pts, forw_left_pts, status_left, err_left, cv::Size(21, 21), 3);
        cv::calcOpticalFlowPyrLK(cur_right_img, forw_right_img, cur_right_pts, forw_right_pts, status_right, err_right, cv::Size(21, 21), 3);

        for (size_t i = 0; i < forw_left_pts.size(); i++) {
            if (status_left[i] == 1 && !inBorder(forw_left_pts[i])) {
                status_left[i] = 0;
            }
        }

        reduceVector(prev_left_pts, status_left);
        reduceVector(cur_left_pts, status_left);
        reduceVector(forw_left_pts, status_left);
        reduceVector(ids_left, status_left);
        reduceVector(cur_undist_left_pts, status_left);
        reduceVector(track_cnt_left, status_left);

        // remove features that weren't tracked in the left image
        reduceVector(prev_right_pts, status_left);
        reduceVector(cur_right_pts, status_left);
        reduceVector(forw_right_pts, status_left);
        reduceVector(ids_right, status_left);
        reduceVector(cur_undist_right_pts, status_left);
        reduceVector(track_cnt_right, status_left);

        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    for (auto &n : track_cnt_left)
        n++;

    for (auto& n : track_cnt_right)
        n++;

    if (PUB_THIS_FRAME)
    {
        rejectWithF();
        /* rejectWithEpipolarConstraint(); */
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;

        // computing how many features are missing
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_left_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_left_img.size())
                cout << "wrong size " << endl;

            // generate new features for the left image
            std::vector<cv::Point2f> n_pts_left_before_filtering, n_pts_right_before_filtering;
            cv::goodFeaturesToTrack(forw_left_img, n_pts_left_before_filtering, MAX_CNT - forw_left_pts.size(), 0.01, MIN_DIST, mask);
            std::cout << "n_pts_left_before_filtering size: " << n_pts_left_before_filtering.size() << std::endl;

            // find them in the rigt image using optical flow
            vector<uchar> status_right;
            vector<float> err_right;
            cv::calcOpticalFlowPyrLK(forw_left_img, forw_right_img, n_pts_left_before_filtering, n_pts_right_before_filtering, status_right, err_right, cv::Size(21, 21), 3);

            // add only new points which satisfy the rectified epipolar constraint
            for (size_t i = 0 ; i < n_pts_right_before_filtering.size() ; ++i) {
                auto left_pt = n_pts_left_before_filtering[i];
                auto right_pt = n_pts_right_before_filtering[i];
                double epipolar_distance = std::abs(left_pt.y - right_pt.y);
                if (status_right[i] == 1 && inBorder(left_pt) && inBorder(right_pt) && epipolar_distance < 1.0) {
                    new_tracked_points_left.push_back(left_pt);
                    new_tracked_points_right.push_back(right_pt);
                }
            }

            ROS_INFO("Rejected %lu new points with epipolar constraint", n_pts_left_before_filtering.size() - new_tracked_points_left.size());

        }
        else {
            new_tracked_points_left.clear();
            new_tracked_points_right.clear();
        }
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }

    // images for next iteration
    // left
    prev_left_img = cur_left_img;
    cur_left_img = forw_left_img;
    
    // right
    prev_right_img = cur_right_img;
    cur_right_img = forw_right_img;

    // points for next iteration
    // left
    prev_left_pts = cur_left_pts;
    prev_undist_left_pts = cur_undist_left_pts;
    cur_left_pts = forw_left_pts;

    // right
    prev_right_pts = cur_right_pts;
    prev_undist_right_pts = cur_undist_right_pts;
    cur_right_pts = forw_right_pts;
        

    undistortedPoints();
    prev_time = cur_time;
}

void FeatureTracker::rejectWithF()
{
    if (forw_left_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_left_pts.size()), un_forw_pts(forw_left_pts.size());
        for (unsigned int i = 0; i < cur_left_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_left_pts[i].x, cur_left_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_left_pts[i].x, forw_left_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_left_pts.size();

        reduceVector(prev_left_pts, status);
        reduceVector(cur_left_pts, status);
        reduceVector(forw_left_pts, status);
        reduceVector(cur_undist_left_pts, status);
        reduceVector(ids_left, status);
        reduceVector(track_cnt_left, status);

        // also remove from the right image feature set
        reduceVector(prev_right_pts, status);
        reduceVector(cur_right_pts, status);
        reduceVector(forw_right_pts, status);
        reduceVector(ids_right, status);
        reduceVector(cur_undist_right_pts, status);
        reduceVector(track_cnt_right, status);

        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_left_pts.size(), 1.0 * forw_left_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

void FeatureTracker::rejectWithEpipolarConstraint() {

    /* // show left and right images with forward features */
    /* cv::Mat left_img = forw_left_img.clone(); */
    /* cv::Mat right_img = forw_right_img.clone(); */
    /* for (auto& pt : forw_left_pts) { */
    /*     cv::circle(left_img, pt, 2, cv::Scalar(0, 0, 255), -1); */
    /* } */
    /* for (auto& pt : forw_right_pts) { */
    /*     cv::circle(right_img, pt, 2, cv::Scalar(0, 0, 255), -1); */
    /* } */
    /* cv::imshow("left", left_img); */
    /* cv::imshow("right", right_img); */
    /* cv::waitKey(0); */

    /* size_t violations{0}; */
    /* std::vector<size_t> good_ixs; */
    /* for (size_t i = 0 ; i < forw_left_pts.size() ; i++) { */
    /*     auto left_pt = forw_left_pts[i]; */
    /*     auto right_pt = forw_right_pts[i]; */
    /*     double epipolar_distance = std::abs(left_pt.y - right_pt.y); */
    /*     if (epipolar_distance > 1.0) { */
    /*         std::cout << "Epipolar constraint violated: " << epipolar_distance << std::endl; */
    /*         violations++; */
    /*     } else { */
    /*         good_ixs.push_back(i); */
    /*     } */
    /* } */

    /* ROS_INFO("Epipolar constraint violations: %lu", violations); */

}

bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids_left.size())
    {
        if (ids_left[i] == -1)
            ids_left[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readLeftCameraParameters(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::setLeftCameraProjectionMatrix(const cv::Mat& P) {
    left_cam_P_ = P;
}

void FeatureTracker::setRightCameraProjectionMatrix(const cv::Mat& P) {
    right_cam_P_ = P;
}

void FeatureTracker::printProjectionMatrices() const {
    cout << "Left camera projection matrix: " << endl;
    cout << left_cam_P_ << endl;
    cout << "Right camera projection matrix: " << endl;
    cout << right_cam_P_ << endl;
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_left_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

void FeatureTracker::undistortedPoints()
{
    cur_undist_left_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_left_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_left_pts[i].x, cur_left_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_undist_left_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids_left[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_undist_left_pts.size(); i++)
        {
            if (ids_left[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids_left[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_undist_left_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_undist_left_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_left_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}
