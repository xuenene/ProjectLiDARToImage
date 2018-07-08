#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>

cv::Vec3f fakeColor(float value) {
    float pos_slope = 255 / 60.0;
    float neg_slope = -255 / 60.0;
    value *= 255;
    cv::Vec3f color;
    if (value < 60) {
        color[0] = 255;
        color[1] = pos_slope * value + 0;
        color[2] = 0;
    } else if (value < 120) {
        color[0] = neg_slope * value + 2 * 255;
        color[1] = 255;
        color[2] = 0;
    } else if (value < 180) {
        color[0] = 0;
        color[1] = 255;
        color[2] = pos_slope * value - 2 * 255;
    } else if (value < 240) {
        color[0] = 0;
        color[1] = neg_slope * value + 4 * 255;
        color[2] = 255;
    } else if (value < 300) {
        color[0] = pos_slope * value - 4 * 255;
        color[1] = 0;
        color[2] = 255;
    } else {
        color[0] = 255;
        color[1] = 0;
        color[2] = neg_slope * value + 6 * 255;
    }
    return color;
}

int main(int argc, char* argv[]) {
    if(argc != 6) {
        cout << "usage: ./ProjectLiDARToImage imgPathName pcdPathName "
                "calib_cam_to_cam.txt calib_cam_to_velo.txt outPathName"
             << endl;
        return -1;
    }
    std::string imgPathName(argv[1]);
    std::string pcdPathName(argv[2]);
    std::string cam2camConf(argv[3]);
    std::string cam2veloConf(argv[4]);
    std::string outPathName(argv[5]);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl->clear();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdPathName, *(pcl)) != 0) {
        printf("cannot open file\n");
        return -1;
    }
    cv::Mat oriCloud = cv::Mat(cv::Size(pcl->points.size(), 3), CV_32FC1);
    for(size_t i = 0; i < pcl->points.size(); ++i) {
        oriCloud.at<float>(0, i) = pcl->points[i].x * 100;
        oriCloud.at<float>(1, i) = pcl->points[i].y * 100;
        oriCloud.at<float>(2, i) = pcl->points[i].z * 100;
    }
    cv::Mat K(3, 3, CV_32FC1), D(1, 4, CV_32FC1);
    cv::Mat R(3, 3, CV_32FC1), T(3, 1, CV_32FC1);
    std::ifstream fin1(cam2camConf), fin2(cam2veloConf);
    if (!fin1.is_open()) {
        std::cout << "open file " << cam2camConf << " failed." << std::endl;
        return false;
    }
    if (!fin2.is_open()) {
        std::cout << "open file " << cam2veloConf << " failed." << std::endl;
        return false;
    }
    float k[9], d[4], r[9], t[3];
    std::string line;
    while (std::getline(fin1, line)) {
        std::string keyStr = line.substr(0, line.find(':'));
        std::string valStr = line.substr(line.find(':') + 2);
        std::stringstream ss(valStr);
        if (keyStr == "K_101") {
            ss >> k[0] >> k[1] >> k[2] >> k[3] >> k[4]
               >> k[5] >> k[6] >> k[7] >> k[8];
            K = cv::Mat(3, 3, CV_32FC1, k);
        } else if (keyStr == "D_101") {
            ss >> d[0] >> d[1] >> d[2] >> d[3];
            D = cv::Mat(1, 4, CV_32FC1, d);
        }
    }
    while (std::getline(fin2, line)) {
        std::string keyStr = line.substr(0, line.find(':'));
        std::string valStr = line.substr(line.find(':') + 2);
        std::stringstream ss(valStr);
        if (keyStr == "R") {
            ss >> r[0] >> r[1] >> r[2] >> r[3] >> r[4]
               >> r[5] >> r[6] >> r[7] >> r[8];
            R = cv::Mat(3, 3, CV_32FC1, r);
        } else if (keyStr == "T") {
            ss >> t[0] >> t[1] >> t[2];
            T = cv::Mat(3, 1, CV_32FC1, t);
        }
    }
    fin1.close();
    fin2.close();

    cv::Mat dist = oriCloud.rowRange(0, 1).mul(oriCloud.rowRange(0, 1)) +
            oriCloud.rowRange(1, 2).mul(oriCloud.rowRange(1, 2)) +
            oriCloud.rowRange(2, 3).mul(oriCloud.rowRange(2, 3));


    R = R.inv();
    T = -R * T * 100;
    cv::Mat img = cv::imread(imgPathName);
    cv::Mat projCloud2d = K * (R * oriCloud + repeat(T, 1, oriCloud.cols));
    float maxDist = 0;
    struct Pt { cv::Point point; float dist; };
    std::vector<Pt> points;
    for(size_t i = 0; i < projCloud2d.cols; ++i) {
        float x = projCloud2d.at<float>(0, i);
        float y = projCloud2d.at<float>(1, i);
        float z = projCloud2d.at<float>(2, i);
        size_t x2d = cvRound(x / z);
        size_t y2d = cvRound(y / z);
        float d = dist.at<float>(0, i);
        if(x2d >= 0 && y2d >= 0 && x2d < img.cols && y2d < img.rows &&
                d > 300 && oriCloud.at<float>(1, i) > 0) {
            maxDist = std::max(maxDist, dist.at<float>(0, i));
            points.push_back(Pt{cv::Point(x2d, y2d), d});
        }
    }
    sort(points.begin(), points.end(), [] (const Pt& a, const Pt& b) {
        return a.dist > b.dist;
    });
    for(size_t i = 0; i < points.size(); ++i) {
        float d = points[i].dist;
        cv::Point pointL = cv::Point(points[i].point.x - 6, points[i].point.y);
        cv::Point pointR = cv::Point(points[i].point.x + 6, points[i].point.y);
        cv::Vec3f color = fakeColor(d / maxDist);
        circle(img, pointL, 3, color, -1);
        circle(img, points[i].point, 3, color, -1);
        circle(img, pointR, 3, color, -1);
    }

    resize(img, img, cv::Size(img.cols / 2, img.rows / 2));
    cv::imwrite(outPathName, img);
    return 0;
}