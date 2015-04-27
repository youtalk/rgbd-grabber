/**
 * @file StereoCameraCalibration.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 17, 2014
 */

#include <opencv2/opencv.hpp>
#include <gflags/gflags.h>
#include <iostream>
#include <string>
#include <sstream>

DEFINE_string(intrinsics, "intrinsics.xml", "intrinsics file");
DEFINE_string(extrinsics, "extrinsics.xml", "extrinsics file");
DEFINE_string(dir, "/tmp", "calibration data directory");
DEFINE_string(suffix, ".png", "file suffix");
DEFINE_int32(size, 1, "number of files");

void loadImages(cv::vector<cv::Mat> &lefts, cv::vector<cv::Mat> &rights,
                const int &fileNum) {
    cv::namedWindow("Left", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    cv::namedWindow("Right", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

    lefts.clear();
    rights.clear();

    for (int i = 0; i < fileNum; i++) {
        std::stringstream lfile, rfile;
        lfile << FLAGS_dir << "/left_" << i << FLAGS_suffix;
        rfile << FLAGS_dir << "/right_" << i << FLAGS_suffix;
        std::cout << "load: " << lfile.str() << ", " << rfile.str() << std::endl;

        cv::Mat left = cv::imread(lfile.str(), 0);
        cv::Mat right = cv::imread(rfile.str(), 0);
        lefts.push_back(left);
        rights.push_back(right);

        cv::imshow("Left", left);
        cv::imshow("Right", right);
        cv::waitKey(100);
    }
}

int findChessboards(
        cv::vector<cv::Mat> &lefts, cv::vector<cv::Mat> &rights,
        cv::vector<cv::vector<cv::vector<cv::Point2f>>> &imagePoints,
        const cv::Size patternSize, const int &fileNum) {
    for (size_t i = 0; i < lefts.size(); ++i) {
        if (cv::findChessboardCorners(
                lefts[i], patternSize, imagePoints[0][i],
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE) &&
            cv::findChessboardCorners(
                rights[i], patternSize, imagePoints[1][i],
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE)) {
            cv::cornerSubPix(
                    lefts[i], imagePoints[0][i], cv::Size(11, 11), cv::Size(-1, -1),
                    cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.01));
            cv::cornerSubPix(
                    rights[i], imagePoints[1][i], cv::Size(11, 11), cv::Size(-1, -1),
                    cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.01));

            cv::drawChessboardCorners(
                    lefts[i], patternSize, (cv::Mat)(imagePoints[0][i]), true);
            cv::drawChessboardCorners(
                    rights[i], patternSize, (cv::Mat)(imagePoints[1][i]), true);

            cv::imshow("Left", lefts[i]);
            cv::imshow("Right", rights[i]);
        } else {
            std::cout << "cannot find all corners" << std::endl;

            lefts.erase(lefts.begin() + i);
            rights.erase(rights.begin() + i);
            imagePoints[0].erase(imagePoints[0].begin() + i);
            imagePoints[1].erase(imagePoints[1].begin() + i);
            i--;
        }

        cv::waitKey(100);
    }

    return lefts.size();
}

void setWorldPoints(cv::vector<cv::vector<cv::Point3f>> &worldPoints,
                    const cv::Size patternSize, double squareSize,
                    const int &fileNum) {
    worldPoints.clear();
    worldPoints.resize(fileNum);

    for (int i = 0; i < fileNum; i++) {
        for (int j = 0; j < patternSize.height; j++)
            for (int k = 0; k < patternSize.width; k++)
                worldPoints[i].push_back(
                        cv::Point3f(k * squareSize, j * squareSize, 0));
    }
}

int main(int argc, char *argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    cv::vector<cv::Mat> lefts, rights;
    const cv::Size patternSize(9, 6);
    cv::vector<cv::vector<cv::Point3f>> worldPoints;
    cv::vector<cv::vector<cv::vector<cv::Point2f>>> imagePoints(2);

    for (size_t i = 0; i < 2; i++)
        imagePoints[i].resize(FLAGS_size);

    loadImages(lefts, rights, FLAGS_size);
    FLAGS_size = findChessboards(lefts, rights, imagePoints, patternSize, FLAGS_size);
    std::cout << "number of correct files = " << FLAGS_size << std::endl;
    setWorldPoints(worldPoints, patternSize, 0.024, FLAGS_size);

    std::cout << "calibrate stereo cameras" << std::endl;
    cv::vector<cv::Mat> cameraMatrix(2);
    cv::vector<cv::Mat> distCoeffs(2);
    cameraMatrix[0] = cv::Mat::eye(3, 3, CV_64FC1);
    cameraMatrix[1] = cv::Mat::eye(3, 3, CV_64FC1);
    distCoeffs[0] = cv::Mat(8, 1, CV_64FC1);
    distCoeffs[1] = cv::Mat(8, 1, CV_64FC1);
    cv::Mat R, T, E, F;

    double rms = stereoCalibrate(
            worldPoints, imagePoints[0], imagePoints[1], cameraMatrix[0],
            distCoeffs[0], cameraMatrix[1], distCoeffs[1], lefts[0].size(),
            R, T, E, F, cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5),
            CV_CALIB_FIX_ASPECT_RATIO +
            CV_CALIB_ZERO_TANGENT_DIST +
            CV_CALIB_SAME_FOCAL_LENGTH +
            CV_CALIB_RATIONAL_MODEL +
            CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
    std::cout << "done with RMS error = " << rms << std::endl;

    double err = 0;
    int npoints = 0;
    for (int i = 0; i < FLAGS_size; i++) {
        int size = (int) imagePoints[0][i].size();
        cv::vector<cv::Vec3f> lines[2];
        cv::Mat imgpt[2];
        for (int k = 0; k < 2; k++) {
            imgpt[k] = cv::Mat(imagePoints[k][i]);
            cv::undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k],
                                cv::Mat(), cameraMatrix[k]);
            cv::computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
        }

        for (int j = 0; j < size; j++) {
            double errij =
                    std::fabs(imagePoints[0][i][j].x * lines[1][j][0] +
                              imagePoints[0][i][j].y * lines[1][j][1] +
                              lines[1][j][2]) +
                    std::fabs(imagePoints[1][i][j].x * lines[0][j][0] +
                              imagePoints[1][i][j].y * lines[0][j][1] +
                              lines[0][j][2]);
            err += errij;
        }
        npoints += size;
    }
    std::cout << "average reprojection error = " << err / npoints << std::endl;

    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect validROI[2];
    stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1],
                  distCoeffs[1], lefts[0].size(), R, T, R1, R2, P1, P2, Q,
                  cv::CALIB_ZERO_DISPARITY, 1, lefts[0].size(),
                  &validROI[0], &validROI[1]);

    {
        cv::FileStorage fs(FLAGS_intrinsics.c_str(), cv::FileStorage::WRITE);
        if (fs.isOpened()) {
            fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0]
                << "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
            fs.release();
        }
    }

    cv::Mat rmap[2][2];
    cv::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1,
                                lefts[0].size(),
                                CV_16SC2,
                                rmap[0][0], rmap[0][1]);
    cv::initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2,
                                lefts[0].size(),
                                CV_16SC2,
                                rmap[1][0], rmap[1][1]);

    {
        cv::FileStorage fs(FLAGS_extrinsics.c_str(), cv::FileStorage::WRITE);
        if (fs.isOpened()) {
            fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2
               << "P1" << P1 << "P2" << P2 << "Q" << Q
               << "V1" << validROI[0] << "V2" << validROI[1];
            fs.release();
        }
    }

    cv::Mat canvas;
    double sf;
    int w, h;

    sf = 600. / MAX(lefts[0].size().width, lefts[0].size().height);
    w = cvRound(lefts[0].size().width * sf);
    h = cvRound(lefts[0].size().height * sf);
    canvas.create(h, w * 2, CV_8UC3);

    cv::namedWindow("Rectified", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

    for (int i = 0; i < FLAGS_size; i++) {
        for (int k = 0; k < 2; k++) {
            if (k == 0) {
                cv::Mat img = lefts[i].clone(), rimg, cimg;
                cv::remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
                cv::cvtColor(rimg, cimg, CV_GRAY2BGR);
                cv::Mat canvasPart = canvas(cv::Rect(w * k, 0, w, h));
                cv::resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);

                cv::Rect vroi(cvRound(validROI[k].x * sf),
                              cvRound(validROI[k].y * sf),
                              cvRound(validROI[k].width * sf),
                              cvRound(validROI[k].height * sf));
                cv::rectangle(canvasPart, vroi, cv::Scalar(0, 0, 255), 3, 8);
            } else {
                cv::Mat img = rights[i].clone(), rimg, cimg;
                cv::remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
                cvtColor(rimg, cimg, CV_GRAY2BGR);
                cv::Mat canvasPart = canvas(cv::Rect(w * k, 0, w, h));
                cv::resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);

                cv::Rect vroi(cvRound(validROI[k].x * sf),
                              cvRound(validROI[k].y * sf),
                              cvRound(validROI[k].width * sf),
                              cvRound(validROI[k].height * sf));
                cv::rectangle(canvasPart, vroi, cv::Scalar(0, 0, 255), 3, 8);
            }
        }

        for (int j = 0; j < canvas.rows; j += 16)
            cv::line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j),
                     cv::Scalar(0, 255, 0), 1, 8);

        cv::imshow("Rectified", canvas);

        if (cv::waitKey(0) == 'q')
            break;
    }

    cv::destroyAllWindows();
    return 0;
}
