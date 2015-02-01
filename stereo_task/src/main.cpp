
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/calib3d/calib3d.hpp"

using namespace cv;
using namespace std;
/**
 * Load data for this assignment.
 * @param fname The JSON input filename.
 * @param left_fnames The output left images of the stereo pair.
 * @param right_fnames The output right images of the stereo pair.
 * @param poses The 6D poses of the camera when the images were taken.
 *
 * This will probably throw an exception if there's something wrong
 * with the json file.
 */
void LoadMetadata(const std::string& fname,
                  std::vector<std::string>& left_fnames,
                  std::vector<std::string>& right_fnames,
                  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> >& poses) {
  namespace bpt = boost::property_tree;
  bpt::ptree pt;
  bpt::read_json(fname, pt);
  for (bpt::ptree::iterator itr=pt.begin();
       itr != pt.end(); ++itr) {
    bpt::ptree::value_type v(*itr);
    bpt::ptree entry(v.second);
    std::string left_fname( entry.get<std::string>("left") );
    std::string right_fname( entry.get<std::string>("right") );
    left_fnames.push_back(left_fname);
    right_fnames.push_back(right_fname);
    Eigen::Vector3d t(entry.get<double>("pose.translation.x"),
                      entry.get<double>("pose.translation.y"),
                      entry.get<double>("pose.translation.z"));
    Eigen::Quaterniond q(entry.get<double>("pose.rotation.w"),
                         entry.get<double>("pose.rotation.x"),
                         entry.get<double>("pose.rotation.y"),
                         entry.get<double>("pose.rotation.z"));
    Eigen::Affine3d aff = Eigen::Translation3d(t) * q;
    poses.push_back(aff);
  }
}

/**
 * Load calibration data.
 * Note this is basically the ROS CameraInfo message.
 * See
 * http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
 * http://wiki.ros.org/image_pipeline/CameraInfo
 * for reference.
 *
 * Note: you probably don't need all the parameters ;)
 */
void LoadCalibration(const std::string& fname,
                     int &width,
                     int &height,
                     cv::Mat& D,
                     cv::Mat& K,
                     cv::Mat& R,
                     cv::Mat& P) {
  namespace bpt = boost::property_tree;
  bpt::ptree pt;
  bpt::read_json(fname, pt);
  width = pt.get<int>("width");
  height = pt.get<int>("height");
  {
    bpt::ptree &spt(pt.get_child("D"));
    D.create(5, 1, CV_32FC1);
    int i=0;
    for (bpt::ptree::iterator itr=spt.begin(); itr != spt.end(); ++itr, ++i) {
      D.at<float>(i,0) = itr->second.get<float>("");
    }
  }
  {
    bpt::ptree &spt(pt.get_child("K"));
    K.create(3, 3, CV_32FC1);
    int ix=0;
    for (bpt::ptree::iterator itr=spt.begin(); itr != spt.end(); ++itr, ++ix) {
      int i=ix/3, j=ix%3;
      K.at<float>(i,j) = itr->second.get<float>("");
    }
  }
  {
    bpt::ptree &spt(pt.get_child("R"));
    R.create(3, 3, CV_32FC1);
    int ix=0;
    for (bpt::ptree::iterator itr=spt.begin(); itr != spt.end(); ++itr, ++ix) {
      int i=ix/3, j=ix%3;
      R.at<float>(i,j) = itr->second.get<float>("");
    }
  }
  {
    bpt::ptree &spt(pt.get_child("P"));
    P.create(3, 4, CV_32FC1);
    int ix=0;
    for (bpt::ptree::iterator itr=spt.begin(); itr != spt.end(); ++itr, ++ix) {
      int i=ix/4, j=ix%4;
      P.at<float>(i,j) = itr->second.get<float>("");
    }
  }
}

/**
 * this is just a suggestion, you can
 * organize your program anyway you like.
 */
void ComputeDisparity(const cv::Mat& left, const cv::Mat& right, cv::Mat& disp8)
{
    cv::Mat g1,g2,disp;
    cvtColor(left, g1, CV_BGR2GRAY);
    cvtColor(right, g2, CV_BGR2GRAY);
    cv::StereoSGBM sbm;
    sbm.SADWindowSize = 3;
        sbm.numberOfDisparities = 144;
        sbm.preFilterCap = 63;
        sbm.minDisparity = -39;
        sbm.uniquenessRatio = 10;
        sbm.speckleWindowSize = 100;
        sbm.speckleRange = 32;
        sbm.disp12MaxDiff = 1;
        sbm.fullDP = false;
        sbm.P1 = 216;
        sbm.P2 = 864;
    sbm(g1, g2, disp);
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
}

int main(int argc, char *argv[]) {

  if (argc < 4) {
    std::cerr << "Usage: " << argv[0] << " JSON_DATA_FILE JSON_LEFT_CALIB_FILE JSON_RIGHT_CALIB_FILE\n";
    return -1;
  }

  // load metadata
  std::vector<std::string> left_fnames, right_fnames;
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > poses;
  LoadMetadata(argv[1], left_fnames, right_fnames, poses);



  // load calibration info.
  // note: you should load right as well
  int left_w, left_h;
  cv::Mat left_D, left_K, left_R, left_P;
  LoadCalibration(argv[2], left_w, left_h, left_D, left_K, left_R, left_P);
  int right_w, right_h;
  cv::Mat right_D, right_K, right_R, right_P;
  LoadCalibration(argv[3], right_w, right_h, right_D, right_K, right_R, right_P);

  // here you should load the images from their filenames

  // NOTE: make sure you run the program from the data/ directory
  // for the paths to work.
  // alternatively feel free to modify the input json file or the image
  // filenames at runtime so the images are found.

  // load one of the images as an example.
  std::cout << "loading " << left_fnames[0] << " ... ";
  cv::Mat left = cv::imread(left_fnames[0]);
  if (left.empty()) {
    std::cerr << "image not found.\n";
    return -1;
  } else {
    std::cout << "loaded image file with size " << left.cols << "x" << left.rows << "\n";
  }

  // then you should do some stereo magic. Feel free to use
  // OpenCV, other 3rd party library or roll your own.

  cv::Mat right = cv::imread(right_fnames[0]);
  cv::Mat disp;
  ComputeDisparity(left, right, disp);
  imshow("left", left);
  imshow("right", right);
  imshow("disp", disp);
  waitKey(0);
// etc.

  // finally compute the output point cloud from one or more stereo pairs.
  //
  // This is just a silly example of creating a colorized XYZ RGB point cloud.
  // open it with pcl_viewer. then press 'r' and '5' to see the rgb.
  pcl::PointCloud<pcl::PointXYZRGB> pc;
  for (int i=0; i < left.rows; ++i) {
    for (int j=0; j < left.cols; ++j) {
      pcl::PointXYZRGB p;
      p.x = j;
      p.y = i;
      p.z = 0;
      cv::Vec3b bgr(left.at<cv::Vec3b>(i, j));
      p.b = bgr[0];
      p.g = bgr[1];
      p.r = bgr[2];
      pc.push_back( p );
    }
  }

  std::cout << "saving a pointcloud to out.pcd\n";
  //pcl::io::savePCDFileASCII("out.pcd", pc);
  pcl::PCDWriter w;
  w.writeBinaryCompressed("out.pcd", pc);
  //pcl::io::savePCDBinaryCompressed("out.pcd", pc);

  return 0;
}
