#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <cmath>


/**
 * This tutorial demonstrates simple plane segmentation from point cloud data.
 */


ros::Publisher vis_pub; //ros publisher to publish markers

/** Function Details
* Function Name     -- extractPlane
* Function Purpose  -- to segment a plane from a point cloud
* Function Input    -- pointCloud, Initialized segmenter
* Function Output   -- Model Coefficients of the form (ax+by+cz+d=0), can be accessed using coefficients.values[i],
										   Inliers that contains the indices of the points that lie inside the segmented plane					
*/
void extractPlane(pcl::PointCloud<pcl::PointXYZ> &ipCloud, pcl::ModelCoefficients &coefficients,pcl::PointIndices &opInliers,pcl::SACSegmentation<pcl::PointXYZ> &seg);


/** Function Details
* Function Name     -- removeInliers
* Function Purpose  -- to remove inliers from a point cloud
* Function Input    -- pointCloud, inlier indices
* Function Output   -- point cloud with inliers removed
*/
void removeInliers(pcl::PointCloud<pcl::PointXYZ> &ipCloud,pcl::PointIndices &inliers);//remove Inliers from the pointcloud


/** Function Details
* Function Name     -- visualizePlanes
* Function Purpose  -- to publish marker to visualize the points belonging to the plane
* Function Input    -- plane id, pointCloud, inliers, point cloud frame, marker Publisher, Model Coefficients in ax+by+cz+d=0
* Function Output   -- marker message sent
*/
void visualizePlanesInliers(int i,pcl::PointCloud<pcl::PointXYZ> &ipCloud,pcl::PointIndices &inliers,std::string &frame,ros::Publisher &visPub,pcl::ModelCoefficients &coefficients);//visualize plane inliers by publishing markers -- This is the function that you will need to change


/** Function Details
* Function Name     -- extractPlanes
* Function Purpose --	 to segment multiple planes from a point cloud and visualize them
* Function Input    -- pointCloud, Initialized segmenter, frame of the point cloud, marker publisher
* Function Output   -- visualized segmented planes
*/
void extractPlanes(pcl::PointCloud<pcl::PointXYZ> &ipCloud,pcl::SACSegmentation<pcl::PointXYZ> &seg,std::string frame, ros::Publisher &visPub); 


/** Function Details
* Function Name     -- ProcessPontCloud2
* Function Purpose --	 callback function that is called everytime we recieve a point cloud
* Function Input    -- pointCloud
* Function Output   -- segmented point cloud visualized
*/
void ProcessInputPointCloud2(const sensor_msgs::PointCloud2ConstPtr& input)
{

	//---------Initializing Segmenter--------------//
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.1);
	//------------Done Initializing Segmenter-----//
	std::string frame;
	
	pcl::PointCloud<pcl::PointXYZ> tempCloud;
	if((input->height*input->width)>0)
	{
		frame = input->header.frame_id;
		pcl::PointCloud<pcl::PointXYZ> cloud;
		fromROSMsg (*input,cloud);
		extractPlanes(cloud,seg,frame,vis_pub);
							
	}
	
};



int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "planeSegmenter");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  vis_pub = n.advertise<visualization_msgs::Marker>("/planesegmentation/planeVisualization", 1);
  
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called ProcessInputPointCloud2.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

	ros::Subscriber sub = n.subscribe("/matcher/cloud", 1, ProcessInputPointCloud2);

  ros::spin();

  return 0;
}



void extractPlane(pcl::PointCloud<pcl::PointXYZ> &ipCloud, pcl::ModelCoefficients &coefficients,pcl::PointIndices &opInliers,pcl::SACSegmentation<pcl::PointXYZ> &seg)
{
	seg.setInputCloud (ipCloud.makeShared());
	seg.segment (opInliers, coefficients);	
			
};

void removeInliers(pcl::PointCloud<pcl::PointXYZ> &ipCloud,pcl::PointIndices &inliers)
{
	pcl::PointCloud<pcl::PointXYZ> tempCloud;
	unsigned int l=0;
	for(unsigned int k=0;k<ipCloud.size();k++)
	{
		if((int)k== inliers.indices[l])
		{	
			l++;
			
		}
		else
		{
			tempCloud.push_back(ipCloud.at(k));
		}
	}	
	ipCloud = tempCloud;
};

void visualizePlanesInliers(int i,pcl::PointCloud<pcl::PointXYZ> &ipCloud,pcl::PointIndices &inliers,std::string &frame,ros::Publisher &visPub,pcl::ModelCoefficients &coefficients)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame;
	marker.header.stamp = ros::Time();
	marker.ns = "plane_fitting";
	marker.id = i;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.a = 0.5;
	marker.lifetime = ros::Duration(0);
	geometry_msgs::Point p;

	
	unsigned int inlier_count = inliers.indices.size();

	double x = coefficients.values[0], y = coefficients.values[1], z = coefficients.values[2], mod;
	mod = sqrt(x*x + y*y + z*z);

	visualization_msgs::Marker marker_arrow;
	marker_arrow.header.frame_id = frame;
	marker_arrow.header.stamp = ros::Time();
	marker_arrow.ns = "plane_fitting_arrow";
	marker_arrow.id = i;
	marker_arrow.type = visualization_msgs::Marker::ARROW;
	marker_arrow.action = visualization_msgs::Marker::ADD;
	marker_arrow.pose.position.x = ipCloud.at(inliers.indices[inlier_count/2]).x;
	marker_arrow.pose.position.y = ipCloud.at(inliers.indices[inlier_count/2]).y;
	marker_arrow.pose.position.z = ipCloud.at(inliers.indices[inlier_count/2]).z;
	marker_arrow.pose.orientation.x = 0;
	marker_arrow.pose.orientation.y = -z;
	marker_arrow.pose.orientation.z = y;
	marker_arrow.pose.orientation.w = mod + x;
	marker_arrow.scale.x = 0.5;
	marker_arrow.scale.y = 0.2;
	marker_arrow.scale.z = 0.2;
	marker_arrow.color.a = 1.0;
	marker_arrow.lifetime = ros::Duration(0);

	for(unsigned int k=0; k<inliers.indices.size();k=k+100)
	{
		p.x = ipCloud.at(inliers.indices[k]).x; p.y = ipCloud.at(inliers.indices[k]).y; p.z = ipCloud.at(inliers.indices[k]).z;
		marker.points.push_back(p);
	}
	/*pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (ipCloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch (0.03);
	ne.compute (*cloud_normals);
	for(unsigned int k=0; k<inliers.indices.size();k=k+100)
	{
		p.x = *cloud_normals.at(inliers.indices[k]).x; p.y = *cloud_normals.at(inliers.indices[k]).y; p.z = *cloud_normals.at(inliers.indices[k]).z;
		marker.points.push_back(p);
		}*/
	
	if (marker.id==0)
	{
	    marker.color.r = 1.0;
	    marker_arrow.color.r=1.0;
	}
	else if (marker.id==1)
	{
	    marker.color.g = 1.0;
	    marker_arrow.color.g=1.0;
	}
	else if (marker.id==2)
	{
	    marker.color.b = 1.0;
	    marker_arrow.color.b=1.0;
	}	
	visPub.publish(marker);
	visPub.publish(marker_arrow);
	marker.points.clear();
	
};


void extractPlanes(pcl::PointCloud<pcl::PointXYZ> &ipCloud,pcl::SACSegmentation<pcl::PointXYZ> &seg,std::string frame, ros::Publisher &visPub)
{	
	pcl::PointCloud<pcl::PointXYZ> tempCloud=ipCloud;	
	for(unsigned int i=0; i<3;i++)
	{	
		pcl::ModelCoefficients coefficients;
		pcl::PointIndices inliers;
		extractPlane(tempCloud,coefficients,inliers,seg);
		
		if(inliers.indices.size() <1)
			break;
		
		visualizePlanesInliers(i,tempCloud,inliers,frame,visPub,coefficients);
		removeInliers(tempCloud,inliers);
	}
	
};
