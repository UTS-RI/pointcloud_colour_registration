#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include <vector>
#include <mutex>
#include <algorithm>


class colour_registration_st
{
private:
    ros::NodeHandle* _pnh;
    image_transport::ImageTransport* _it;
    ros::Publisher _registered_publisher;
    std::string _published_topicname;
    int _seq;

    std::vector<cv::Mat> _current_images;

    std::mutex _worker_mtx;
    bool _worker_ready;
    std::mutex _img_mtx;

    bool _iscompressed;
    std::vector<image_transport::Subscriber> _camera_subscribers;
    std::vector<std::string> _camerasub_topics;

    ros::Subscriber _pointcloud_subscriber;
    std::string _pointcloud_topicname;

    std::vector<std::string> _camera_frames;
    std::string _base_frame;
    std::string _frame_suffix;

    std::vector<Eigen::Matrix3d> _camera_K;
    std::vector<geometry_msgs::Transform> _camera_frame_transforms;

    bool getParams();

protected:
    virtual void processLongString(std::string input, std::string delimiter, std::vector<std::string> &invec);
    virtual int32_t rgb(uchar r, uchar g, uchar b);

    bool obtainIntrinsicParameters(std::vector<std::string>::iterator &it);
    bool obtainTransforms();

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
    void cameraImageCallback(const sensor_msgs::ImageConstPtr &msg);
    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);


    void publishPointcloud(sensor_msgs::PointCloud2 &cloud);
public:
    colour_registration_st(ros::NodeHandle &pnh);
    ~colour_registration_st();
};

