/**
 * @file DS325Calibration.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jun 20, 2014
 */

#include <opencv2/opencv.hpp>
#include <gflags/gflags.h>
#include <iostream>
#include <string>
#include <sstream>

using namespace google;

DEFINE_int32(camera, 0, "camera id");
DEFINE_string(dir, "/tmp/calib", "calibration data directory");
DEFINE_string(depth, "depth_", "depth file prefix");
DEFINE_string(color, "color_", "color file prefix");
DEFINE_string(suffix, ".png", "file suffix");
DEFINE_int32(number, 1, "number of files");

void loadImages(cv::vector<cv::Mat> &colors, cv::vector<cv::Mat> &depths,
                const int &fileNum) {
    colors.clear();
    depths.clear();

    for (int i = 0; i < fileNum; ++i) {
        std::stringstream colorFile, depthFile;
        colorFile << FLAGS_dir << "/" << FLAGS_color << i << FLAGS_suffix;
        depthFile << FLAGS_dir << "/" << FLAGS_depth << i << FLAGS_suffix;
        std::cout << colorFile.str() << ", " << depthFile.str() << ": load" << std::endl;

        cv::Mat color = cv::imread(colorFile.str(), 0);
        colors.push_back(color);

        cv::Mat depth = cv::imread(depthFile.str(), CV_LOAD_IMAGE_ANYDEPTH);
        depth.convertTo(depth, CV_8U, 255.0 / 1000.0);
        cv::resize(depth, depth, color.size());
        cv::Mat roi;
        cv::resize(depth(cv::Rect(40, 43, 498, 498 / 4 * 3)), roi, color.size()); // TODO
        depths.push_back(roi);

        cv::imshow("color", colors[i]);
        cv::imshow("depth", depths[i]);
        cv::waitKey(100);
    }
}

int findChessboard(
        cv::vector<cv::Mat> &colors, cv::vector<cv::Mat> &depths,
        cv::vector<cv::vector<cv::vector<cv::Point2f>>> &imagePoints,
        const cv::Size patternSize, const int &fileNum) {
    for (int i = 0; i < colors.size(); ++i) {

        if (cv::findChessboardCorners(
                colors[i], patternSize, imagePoints[0][i],
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE) &&
            cv::findChessboardCorners(
                depths[i], patternSize, imagePoints[1][i],
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE)) {

            cv::cornerSubPix(
                    colors[i], imagePoints[0][i],
                    cv::Size(11, 11), cv::Size(-1, -1),
                    cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.01));
            cv::cornerSubPix(
                    depths[i], imagePoints[1][i],
                    cv::Size(11, 11), cv::Size(-1, -1),
                    cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.01));

            cv::drawChessboardCorners(colors[i], patternSize,
                                      (cv::Mat)(imagePoints[0][i]), true);
            cv::drawChessboardCorners(depths[i], patternSize,
                                      (cv::Mat)(imagePoints[1][i]), true);
            cv::imshow("color", colors[i]);
            cv::imshow("depth", depths[i]);
        } else {
            std::cout << "cannot find all corners" << std::endl;
            colors.erase(colors.begin() + i);
            depths.erase(depths.begin() + i);
            imagePoints[0].erase(imagePoints[0].begin() + i);
            imagePoints[1].erase(imagePoints[1].begin() + i);
            std::cout << colors.size() << std::endl;
            i--;
        }

        cv::waitKey(100);
    }

    return colors.size();
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
    ParseCommandLineFlags(&argc, &argv, true);

    cv::namedWindow("color", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    cv::namedWindow("depth", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

    cv::vector<cv::Mat> colors(0), depths(0);
    int numFile = FLAGS_number;
    const cv::Size patternSize(9, 6);
    cv::vector<cv::vector<cv::Point3f>> worldPoints;
    cv::vector<cv::vector<cv::vector<cv::Point2f>>> imagePoints(2);
    cv::TermCriteria criteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.001);

    for (size_t i = 0; i < 2; i++)
        imagePoints[i].resize(numFile);

    loadImages(colors, depths, FLAGS_number);
    numFile = findChessboard(colors, depths, imagePoints, patternSize, numFile);

    std::cout << "number of correct files:" << numFile << std::endl;
    setWorldPoints(worldPoints, patternSize, 24.0, numFile);

    std::cout << "calibrate streo cameras" << std::endl;
    cv::vector<cv::Mat> cameraMatrix(2);
    cv::vector<cv::Mat> distCoeffs(2);
    cameraMatrix[0] = cv::Mat::eye(3, 3, CV_64FC1);
    cameraMatrix[1] = cv::Mat::eye(3, 3, CV_64FC1);
    distCoeffs[0] = cv::Mat(8, 1, CV_64FC1);
    distCoeffs[1] = cv::Mat(8, 1, CV_64FC1);
    cv::Mat R, T, E, F;
    R = cv::Mat::eye(3, 3, CV_64FC1);
    T = cv::Mat::eye(3, 3, CV_64FC1);
    E = cv::Mat::eye(3, 3, CV_64FC1);
    F = cv::Mat::eye(3, 3, CV_64FC1);

    cv::vector<cv::Mat> rvecs;
    cv::vector<cv::Mat> tvecs;
    double rms1 = calibrateCamera(worldPoints, imagePoints[0], colors[0].size(),
                                  cameraMatrix[0], distCoeffs[0], rvecs, tvecs,
                                  CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);
    double rms2 = calibrateCamera(worldPoints, imagePoints[1], colors[0].size(),
                                  cameraMatrix[1], distCoeffs[1], rvecs, tvecs,
                                  CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

    std::cout << "camera matrix:" << std::endl;
    std::cout << cameraMatrix[0] << std::endl;
    std::cout << cameraMatrix[1] << std::endl;

    std::cout << "dist coeffs:" << std::endl;
    std::cout << distCoeffs[0] << std::endl;
    std::cout << distCoeffs[1] << std::endl;

    double rms = stereoCalibrate(
            worldPoints, imagePoints[0], imagePoints[1], cameraMatrix[0],
            distCoeffs[0], cameraMatrix[1], distCoeffs[1], colors[0].size(),
            R, T, E, F, cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5),
            CV_CALIB_USE_INTRINSIC_GUESS +
            //CV_CALIB_FIX_INTRINSIC +
            CV_CALIB_FIX_PRINCIPAL_POINT + CV_CALIB_FIX_ASPECT_RATIO + CV_CALIB_ZERO_TANGENT_DIST +
            //CV_CALIB_SAME_FOCAL_LENGTH +
            CV_CALIB_RATIONAL_MODEL + CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
    std::cout << "RMS error: " << rms << std::endl;

    std::cout << "camera matrix:" << std::endl;
    std::cout << cameraMatrix[0] << std::endl;
    std::cout << cameraMatrix[1] << std::endl;

    std::cout << "dist coeffs:" << std::endl;
    std::cout << distCoeffs[0] << std::endl;
    std::cout << distCoeffs[1] << std::endl;

    std::cout << "R:" << R << std::endl;
    std::cout << "T:" << T << std::endl;
    std::cout << "E:" << E << std::endl;
    std::cout << "F:" << F << std::endl;

    // CALIBRATION QUALITY CHECK
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    cv::vector<cv::Vec3f> lines[2];
    for (int i = 0; i < numFile; i++) {
        int npt = (int) imagePoints[0][i].size();
        cv::Mat imgpt[2];

        {
            imgpt[0] = cv::Mat(imagePoints[0][i]);
            cv::undistortPoints(imgpt[0], imgpt[0], cameraMatrix[0],
                                distCoeffs[0], cv::Mat(), cameraMatrix[0]);
            cv::computeCorrespondEpilines(imgpt[0], 1, F, lines[0]);
        }
        {
            imgpt[1] = cv::Mat(imagePoints[1][i]);
            cv::undistortPoints(imgpt[1], imgpt[1], cameraMatrix[1],
                                distCoeffs[1], cv::Mat(), cameraMatrix[1]);
            cv::computeCorrespondEpilines(imgpt[1], 2, F, lines[1]);
        }

        for (int j = 0; j < npt; j++) {
            double errij =
                    std::fabs(imagePoints[0][i][j].x * lines[1][j][0] +
                              imagePoints[0][i][j].y * lines[1][j][1] +
                              lines[1][j][2]) +
                    std::fabs(imagePoints[1][i][j].x * lines[0][j][0] +
                              imagePoints[1][i][j].y * lines[0][j][1] +
                              lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    std::cout << "average reprojection error:" << err / npoints << std::endl;

    std::cout << "press key >" << std::endl;
    cv::waitKey(0);

    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect validROI[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1],
                  distCoeffs[1], colors[0].size(), R, T, R1, R2, P1, P2, Q,
                  cv::CALIB_ZERO_DISPARITY, 1, colors[0].size(), &validROI[0],
                  &validROI[1]);

    std::cout << "valid ROI: " << validROI[0] << " " << validROI[1] << std::endl;

    cv::vector<cv::Point2f> allimgpt[2];
    for (int k = 0; k < 2; k++) {
        for (int i = 0; i < numFile; i++)
            std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(),
                      back_inserter(allimgpt[k]));
    }

    // F = cv::findFundamentalMat(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), cv::FM_8POINT, 0, 0);
    // cv::Mat H1, H2;
    // cv::stereoRectifyUncalibrated(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), F, rgb[0].size(), H1, H2, 3);

    // R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
    // R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
    // P1 = cameraMatrix[0];
    // P2 = cameraMatrix[1];

    cv::Mat rmap[2][2];
    cv::Mat nonR1 = cv::Mat::eye(R1.size(), R1.type());
    cv::Mat nonR2 = cv::Mat::eye(R1.size(), R2.type());

    cv::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1,
                                colors[0].size(),
                                CV_16SC2,
                                rmap[0][0], rmap[0][1]);
    cv::initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2,
                                colors[0].size(),
                                CV_16SC2,
                                rmap[1][0], rmap[1][1]);

    cv::Mat_<int> validROIMat(2, 4);
    for (int i = 0; i < 2; ++i) {
        validROIMat.at<int>(i, 0) = validROI[i].x;
        validROIMat.at<int>(i, 1) = validROI[i].y;
        validROIMat.at<int>(i, 2) = validROI[i].width;
        validROIMat.at<int>(i, 3) = validROI[i].height;
    }

    cv::FileStorage fs("streo-params.xml", CV_STORAGE_WRITE);
    if (fs.isOpened()) {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] << "M2"
           << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1
           << "P2" << P2 << "Q" << Q << "validROI" << validROIMat;
        fs.release();
    }

    double apertureWidth = 0, apertureHeight = 0;
    double fovx = 0, fovy = 0;
    double focalLength = 0;
    cv::Point2d principalPoint(0, 0);
    double aspectRatio = 0;

    cv::calibrationMatrixValues(cameraMatrix[0], colors[0].size(), apertureWidth,
                                apertureHeight, fovx, fovy, focalLength,
                                principalPoint, aspectRatio);

    cv::FileStorage wfs("streo-conf.xml", cv::FileStorage::WRITE);

    if (wfs.isOpened()) {
        wfs << "apertureWidth" << apertureWidth;
        wfs << "apertureHeight" << apertureHeight;
        wfs << "fovx" << fovx;
        wfs << "fovy" << fovy;
        wfs << "focalLength" << focalLength;
        wfs << "principalPoint" << principalPoint;
        wfs << "aspectRatio" << aspectRatio;
        wfs.release();
    }

    std::cout << "check quality" << std::endl;
    loadImages(colors, depths, FLAGS_number);

    cv::Mat canvas;
    double sf;
    int w, h;

    sf = 600. / MAX(colors[0].size().width, colors[0].size().height);
    w = cvRound(colors[0].size().width * sf);
    h = cvRound(colors[0].size().height * sf);
    canvas.create(h, w * 2, CV_8UC3);

    cv::namedWindow("rectified", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

    for (int i = 0; i < numFile; i++) {
        for (int k = 0; k < 2; k++) {
            if (k == 0) {
                cv::Mat img = colors[i].clone(), rimg, cimg;

                cv::remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
                cv::cvtColor(rimg, cimg, CV_GRAY2BGR);

                cv::Mat canvasPart = canvas(cv::Rect(w * k, 0, w, h));
                cv::resize(cimg, canvasPart, canvasPart.size(), 0, 0,
                           CV_INTER_AREA);

                cv::Rect vroi(cvRound(validROI[k].x * sf),
                              cvRound(validROI[k].y * sf),
                              cvRound(validROI[k].width * sf),
                              cvRound(validROI[k].height * sf));
                cv::rectangle(canvasPart, vroi, cv::Scalar(0, 0, 255), 3, 8);

            } else {
                cv::Mat img = depths[i].clone(), rimg, cimg;
                cv::remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
                cvtColor(rimg, cimg, CV_GRAY2BGR);
                cv::Mat canvasPart = canvas(cv::Rect(w * k, 0, w, h));
                cv::resize(cimg, canvasPart, canvasPart.size(), 0, 0,
                           CV_INTER_AREA);

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

        cv::imshow("rectified", canvas);

        if (cv::waitKey(0) == 'q')
            break;
    }

    cv::destroyAllWindows();
    return 0;
}
