
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <ros/ros.h>

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/core/eigen.hpp>

#include "elas.h"
#include "image.h"
#include <iostream>


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
       itr != pt.end(); ++itr)
  {
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

void process (const char* file_1,const char* file_2)
{

  cout << "Processing: " << file_1 << ", " << file_2 << endl;

  // load images
  image<uchar> *I1,*I2;
  I1 = loadPGM(file_1);
  I2 = loadPGM(file_2);

  // check for correct size
  if (I1->width()<=0 || I1->height() <=0 || I2->width()<=0 || I2->height() <=0 ||
      I1->width()!=I2->width() || I1->height()!=I2->height()) {
    cout << "ERROR: Images must be of same size, but" << endl;
    cout << "       I1: " << I1->width() <<  " x " << I1->height() << 
                 ", I2: " << I2->width() <<  " x " << I2->height() << endl;
    delete I1;
    delete I2;
    return;    
  }

  // get image width and height
  int32_t width  = I1->width();
  int32_t height = I1->height();

  // allocate memory for disparity images
  const int32_t dims[3] = {width,height,width}; // bytes per line = width
  float* D1_data = (float*)malloc(width*height*sizeof(float));
  float* D2_data = (float*)malloc(width*height*sizeof(float));

  // process
  Elas::parameters param;
  param.postprocess_only_left = false;
  Elas elas(param);
  elas.process(I1->data,I2->data,D1_data,D2_data,dims);

  // find maximum disparity for scaling output disparity images to [0..255]
  float disp_max = 0;
  for (int32_t i=0; i<width*height; i++) {
    if (D1_data[i]>disp_max) disp_max = D1_data[i];
    if (D2_data[i]>disp_max) disp_max = D2_data[i];
  }

  // copy float to uchar
  image<uchar> *D1 = new image<uchar>(width,height);
  image<uchar> *D2 = new image<uchar>(width,height);
  for (int32_t i=0; i<width*height; i++) {
    D1->data[i] = (uint8_t)max(255.0*D1_data[i]/disp_max,0.0);
    D2->data[i] = (uint8_t)max(255.0*D2_data[i]/disp_max,0.0);
  }

  // save disparity images
  char output_1[1024];
  char output_2[1024];
  strncpy(output_1,file_1,strlen(file_1)-8);
  strncpy(output_2,file_2,strlen(file_2)-4);
  output_1[strlen(file_1)-14] = '\0';
  output_2[strlen(file_2)-4] = '\0';
  strcat(output_1,"_disp.pgm");
  //strcat(output_2,"_disp.pgm");
  savePGM(D1,output_1);
  //savePGM(D2,output_2);

  // free memory
  delete I1;
  delete I2;
  delete D1;
  delete D2;
  free(D1_data);
  free(D2_data);
}

/**
 * this is just a suggestion, you can
 * organize your program anyway you like.
 */
void ComputeDisparity(const char* file_1,const char* file_2, cv::Mat& disp)
{
/*  cv::Mat g1,g2,disp;
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
*/
    Mat left_image = imread(file_1,1);
    Mat right_image = imread(file_2,1);
    
    Mat left_gray_image,right_gray_image;
    cvtColor( left_image, left_gray_image, CV_BGR2GRAY );
    cvtColor( right_image, right_gray_image, CV_BGR2GRAY );

    char output_1[1024];
    char output_2[1024];
    strncpy(output_1,file_1,strlen(file_1)-4);
    strncpy(output_2,file_2,strlen(file_2)-4);
    output_1[strlen(file_1)-4] = '\0';
    output_2[strlen(file_2)-4] = '\0';
    strcat(output_1,"_gray.pgm");
    strcat(output_2,"_gray.pgm");
    imwrite(output_1, left_gray_image );
    imwrite(output_2, right_gray_image );

    process(output_1,output_2);
    char file_name[1024];
    strncpy(file_name,output_1,strlen(output_1)-14);
    file_name[strlen(output_1)-14] = '\0';
    strcat(file_name,"_disp.pgm");
    printf("NAME:::: %s  ",file_name);
    disp = imread(file_name,CV_LOAD_IMAGE_GRAYSCALE);   
}

int main(int argc, char *argv[])
{

  if (argc < 4)
  {
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
  char ch = '1';
  for(size_t i =0;i<left_fnames.size();i++)
  {
      //cout<<poses[i].rotation()<<endl;
      //cout<<poses[i].translation();
      cv::Mat left_r,left_p,right_r,right_p,Q,rot,trans;
      eigen2cv(poses[i].rotation(),rot);
      Eigen::Vector3d t = poses[i].translation();
      eigen2cv(t,trans);
      // eigen2cv(poses[i].rotation().matrix(),rot);
      //  eigen2cv(poses[i].translation().matrix(),trans);
      cv::stereoRectify(left_K,left_D,right_K,right_D,cv::Size(right_w,right_h),rot,trans,left_r,right_r,left_p,right_p,Q,CALIB_ZERO_DISPARITY);
      cv::Mat left = cv::imread(left_fnames[i]);
      if (left.empty()) {
	  std::cerr << "image not found.\n";
	  return -1;
      } else {
	  std::cout << "loaded image file with size " << left.cols << "x" << left.rows << "\n";
      }

      // then you should do some stereo magic. Feel free to use
      // OpenCV, other 3rd party library or roll your own.

      cv::Mat right = cv::imread(right_fnames[i]);
      cv::Mat disp;
      ComputeDisparity(left_fnames[i].c_str(), right_fnames[i].c_str(), disp);
      // imshow("disp", disp);
      // waitKey(0);
// etc.
      cv::Mat recons3D(disp.size(),CV_32FC3);
//      cv::reprojectImageTo3D(disp,recons3D,Q,true,CV_32F);
      // finally compute the output point cloud from one or more stereo pairs.
      //
      // This is just a silly example of creating a colorized XYZ RGB point cloud.
      // open it with pcl_viewer. then press 'r' and '5' to see the rgb.
      pcl::PointCloud<pcl::PointXYZRGB> pc;
      for (int i=0; i < disp.rows; ++i)
      {
	  for (int j=0; j < disp.cols; ++j)
	  {
	      pcl::PointXYZRGB p;
	      //cv::Vec3b xyzCoordinates(recons3D.at<cv::Vec3b>(i, j));
	      p.x = j;
	      p.y = i;
	      p.z = disp.at<uchar>(i,j);
	      cv::Vec3b bgr(left.at<cv::Vec3b>(i, j));
	      p.b = bgr[0];
	      p.g = bgr[1];
	      p.r = bgr[2];
	      pc.push_back( p );
	  }
      }

      std::cout << "saving a pointcloud to out[i].pcd\n";
      //pcl::io::savePCDFileASCII("out.pcd", pc);
      pcl::PCDWriter w;
      char name[1024] = "pair0";
      name[strlen(name)]=ch;
      name[strlen(name)+1]='\0';
      strcat(name,".pcd");
      w.writeBinaryCompressed(name,pc);
      //pcl::io::savePCDBinaryCompressed("out.pcd", pc);
      ch++;
  }
  return 0;
}
