#include <colour_registration/colour_registration_st.h>

colour_registration_st::colour_registration_st(ros::NodeHandle &pnh)
    : _pnh(new ros::NodeHandle(pnh))
    , _it(new image_transport::ImageTransport(pnh))
    , _seq(0)
    , _worker_ready(false)
{
    ROS_DEBUG("Getting parameters");
    if(!getParams())
    {
        ROS_ERROR("Unable to get all necessary parameters from param server");
        return;
    }
    ROS_DEBUG("Getting transforms");
    if(!obtainTransforms())
    {   
        ROS_ERROR("Error while obtaining static transforms between the camera frames.");
        return;
    }
    ROS_DEBUG("Vector size checking");
    // check all vector sizes are the same
    if (   _camera_frames.size() != _camerasub_topics.size()
        || _camera_frames.size() != _camera_K.size()
        || _camera_frames.size() != _camera_frame_transforms.size())
    {
        ROS_ERROR("Vector sizes different between camera_frames, camerasub_topics, and camerainfo_topics. Double check your parameter server");
        return;
    }
    _registered_publisher = _pnh->advertise<sensor_msgs::PointCloud2>(_published_topicname.c_str(),1);
    for(int i=0;i<_camera_frames.size();i++)
    {
        _current_images.push_back(cv::Mat());
    }

    ROS_DEBUG("Starting subscribers. Size of camerasub topics: %lu", _camerasub_topics.size());
    // startup camera subs
    for(auto &p: _camerasub_topics)
    {
        // ros::Subscriber newsub = _pnh->subscribe(p.c_str(),1,&colour_registration_st::cameraImageCallback,this);
        image_transport::Subscriber newsub = _it->subscribe(p.c_str(),1, &colour_registration_st::cameraImageCallback,this);
        _camera_subscribers.push_back(newsub);
    }
    _pointcloud_subscriber = _pnh->subscribe(_pointcloud_topicname.c_str(),1,&colour_registration_st::pointcloudCallback,this);
    // spin for callbacks for image and pointcloud
    ROS_INFO("Pointcloud topic name: %s",_pointcloud_topicname.c_str());
    ROS_INFO("Number of cameras to register: %lu",_camera_K.size());
    ROS_INFO("Colour Registration node launched.");
    ros::spin();
}

colour_registration_st::~colour_registration_st()
{
    for(std::vector<image_transport::Subscriber>::iterator it = _camera_subscribers.begin(); it != _camera_subscribers.end(); it++)
    {
        it->shutdown();
    }
    _registered_publisher.shutdown();
    _pnh->shutdown();
    delete _pnh;
}

bool colour_registration_st::getParams()
{
    bool temp = true;
    if(!_pnh->getParam("published_topicname",_published_topicname))
    {
        ROS_WARN("Unable to find published_topicname parameter.");
        temp = false;
    }
    if(!_pnh->getParam("pointcloud_topicname",_pointcloud_topicname))
    {
        ROS_WARN("Unable to find pointcloud_topicname parameter.");
        temp = false;
    }
    if(!_pnh->getParam("base_frame",_base_frame))
    {
        ROS_WARN("Unable to find base_frame parameter.");
        temp = false;
    }
    std::string tempstring;
    if(!_pnh->getParam("transport_type",tempstring))
    {
        ROS_WARN("Unable to find transport_type parameter for images.");
        temp = false;
    }
    else
    {
        if(!std::strcmp(tempstring.c_str(),"compressed"))
        {
            _iscompressed = true;
            ROS_INFO("Using compressed image transport");
        }
        else if(!std::strcmp(tempstring.c_str(),"raw"))
        {
            _iscompressed = false;
            ROS_INFO("Using raw image transport");
        }
        else
        {
            ROS_WARN("Unknown transport_type parameter: %s",tempstring.c_str());
            temp = false;
        }
    }
    if(!_pnh->getParam("frame_suffix",_frame_suffix))
    {
        ROS_WARN("Unable to find frame_suffix parameter.");
        temp = false;
    }
    if(!_pnh->getParam("camera_frames",tempstring))
    {
        ROS_WARN("Unable to find camera_frames parameter.");
    }
    else
    {
        processLongString(tempstring,",",_camera_frames);
    }
    if(!_pnh->getParam("camera_subtopics",tempstring))
    {
        ROS_WARN("Unable to find camera_subtopics parameter.");
    }
    else
    {
        processLongString(tempstring,",",_camerasub_topics);
    }
    if(!_pnh->getParam("camerainfo_topics",tempstring))
    {
        ROS_WARN("Unable to find camerainfo_topics parameter.");
    }
    else
    {
        // process tempstring for camerainfo intrinsic matrices
        std::vector<std::string> camerainfos;
        processLongString(tempstring,",",camerainfos);
        for(std::vector<std::string>::iterator it = camerainfos.begin(); it != camerainfos.end(); it++)
        {
            ROS_DEBUG("Obtaining intrinsic parameters for camerainfo topic name: %s",it->c_str());
            if(!obtainIntrinsicParameters(it))
            {
                temp = false;
                ROS_ERROR("Error in obtaining intrinsic parameters. Check camerainfo_topics");
                break;
            }
        }
        ROS_DEBUG("Completed obtaining intrinsic parameters");
    }
    return temp;
}

void colour_registration_st::processLongString(std::string input,std::string delimiter, std::vector<std::string> &invec)
{
    size_t pos = 0;
    std::string temp;
    while((pos = input.find(delimiter)) != std::string::npos )
    {
        temp = input.substr(0,pos);
        invec.push_back(temp);
        input.erase(0,pos + delimiter.length());
    }
    invec.push_back(input);
}

bool colour_registration_st::obtainIntrinsicParameters(std::vector<std::string>::iterator &it)
{
    bool temp = true;
    int currentsize = _camera_K.size();
    ros::Subscriber tempsub = _pnh->subscribe(it->c_str(),1,&colour_registration_st::cameraInfoCallback,this);
    ros::Duration(0.1).sleep();
    int count = 0;
    while(_pnh->ok())
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        int size;
        {
            std::lock_guard<std::mutex> lk(_worker_mtx);
            size = _camera_K.size();
        }
        if(size == currentsize + 1)
        {
            break;
        }
        count++;
        if(count > 10)
        {
            temp = false;
            break;
        }
    }
    tempsub.shutdown();
    return temp;
}

bool colour_registration_st::obtainTransforms()
{
    bool temp = true;
    tf::TransformListener listener;
    tf::Transform tf;
    for(auto &p: _camera_frames)
    {
        // each string change to _color_optical_frame
        int startidx = ( *(p.begin()) == '/' ) ? 1 : 0;
        size_t npos = p.find("_link")-startidx;
        std::string opt_frame = p.substr(startidx,npos) + _frame_suffix;
        // each string, check transform
        ROS_DEBUG("Waiting transform between: %s and %s",_base_frame.c_str(),opt_frame.c_str());
        if(!listener.waitForTransform(opt_frame,_base_frame,ros::Time(0),ros::Duration(5.0)))
        {
            temp = false;
            break;
        }
        tf::StampedTransform temptf;
        try
        {
            // transform to convert pointcloud in the frame of the camera when performing image projection calcs
            listener.lookupTransform(opt_frame,_base_frame,ros::Time(0),temptf);
            tf.setOrigin(temptf.getOrigin());
            tf.setRotation(temptf.getRotation());
            geometry_msgs::Transform geometrytf;
            tf::transformTFToMsg(tf,geometrytf);
            _camera_frame_transforms.push_back(geometrytf);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            temp = false;
            break;
        }
    }
    return temp;
}

void colour_registration_st::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
    std::lock_guard<std::mutex> lk(_worker_mtx);
    Eigen::Matrix3d temp;
    temp = Eigen::Matrix3d::Map(&(msg->K)[0],3,3);
    _camera_K.push_back(temp.transpose());
}

void colour_registration_st::cameraImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_DEBUG("Camera image callback");
    // find which index to process
    int index;
    ROS_DEBUG("Frame id: %s",msg->header.frame_id.c_str());
    // take '/' into account for frame id 
    int startidx = ( *(msg->header.frame_id.begin()) == '/') ? 1 : 0;
    
    std::size_t pos = msg->header.frame_id.find(_frame_suffix) - startidx;
    ROS_DEBUG("Token: %s",(msg->header.frame_id.substr(startidx,pos)+"_link").c_str());
    auto it = std::find(_camera_frames.begin(),_camera_frames.end(),(msg->header.frame_id.substr(startidx,pos))+"_link");
    if(it == _camera_frames.end())
    {
        ROS_WARN("End of camera frame vector");
        ROS_ERROR("Invalid camera frame. Please check your param again");
        return;
    }
    index = it - _camera_frames.begin();
    ROS_DEBUG("Index value: %d", index);

    cv_bridge::CvImagePtr cvbridge;
    try
    {
        cvbridge = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    std::lock_guard<std::mutex> lk(_img_mtx);
    _current_images[index] = cvbridge->image;
    ROS_DEBUG("Size of current image [%d]: %u x %u",index,_current_images[index].rows,_current_images[index].cols);
}

void colour_registration_st::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    ROS_DEBUG("Pointcloud Callback");
    ros::Time start = ros::Time::now();
    // copy current images
    std::vector<cv::Mat> images;
    {
        std::lock_guard<std::mutex> lk(_img_mtx);
        images = _current_images;
    }
    bool emptyimg = true;
    for(auto &p: images)
    {
        emptyimg &= images.empty();
    }
    if(emptyimg)
    {
        ROS_WARN("All cvMat in vector empty");
        sensor_msgs::PointCloud2 temp = *msg;
        publishPointcloud(temp);
        ROS_DEBUG("Time taken to process: %f",(ros::Time::now()-start).toSec());
        return;
    }
    // process and calculate
    pcl::PointCloud<pcl::PointXYZ> tempxyz;
    pcl::fromROSMsg(*msg,tempxyz);
    pcl::PointCloud<pcl::PointXYZRGB> colored;
    pcl::copyPointCloud(tempxyz,colored);
    ROS_DEBUG("Point cloud size: %lu",colored.points.size());
    
    // Eigen::Matrix<double, 4, Eigen::Dynamic> eig;
    Eigen::Matrix<double, 3, Eigen::Dynamic> eigZ;
    Eigen::Matrix<double, 3, Eigen::Dynamic> image_projections;
    Eigen::Matrix<double, 3, 4> camera_matrix;
    camera_matrix << Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,1);    
    
    std::vector<cv::Mat>::iterator mat_it = images.begin();
    std::vector<Eigen::Matrix3d>::iterator K_it = _camera_K.begin();
    int count = 0;
    // assumption is that each camera don't have overlapping FOVs - count would be inaccurate if there is (double dipping of coloured points)
    for(std::vector<geometry_msgs::Transform>::iterator it = _camera_frame_transforms.begin(); it != _camera_frame_transforms.end(); it++, mat_it++, K_it++)
    {
        pcl::PointCloud<pcl::PointXYZRGB> cam_frame_colored;
        pcl_ros::transformPointCloud(colored,cam_frame_colored,*it);
        std::vector<double> coloredinputs;
        for(auto &p: cam_frame_colored.points)
        {
            coloredinputs.push_back(p.x);
            coloredinputs.push_back(p.y);
            coloredinputs.push_back(p.z);
            coloredinputs.push_back(1.0);
        }
        Eigen::MatrixXd eigenPC;
        eigenPC = Eigen::MatrixXd::Map(&coloredinputs[0],4,colored.points.size());
        eigZ = eigenPC.row(2).replicate(3,1);
        image_projections = (*K_it) * camera_matrix * eigenPC;
        image_projections = image_projections.array() / eigZ.array();
        for(int idx=0; idx < image_projections.cols(); idx++)
        {
            if( (eigenPC(2,idx) < 0 ) )
            {
                continue;
            }
            int imgx = std::round(image_projections(0,idx));
            int imgy = std::round(image_projections(1,idx));
            if (imgx < 0 || imgy < 0 || imgx > mat_it->cols || imgy > mat_it->rows )
            {
                continue;
            }
            cv::Vec3b colors = mat_it->at<cv::Vec3b>(imgy,imgx);
            colored.points.at(idx).b = colors[0];
            colored.points.at(idx).g = colors[1];
            colored.points.at(idx).r = colors[2];
            count++;
        }
    }
    sensor_msgs::PointCloud2 output_cloud;
    pcl::toROSMsg(colored,output_cloud);
    publishPointcloud(output_cloud);
    ROS_DEBUG("Time taken to process: %f",(ros::Time::now()-start).toSec());
}

void colour_registration_st::publishPointcloud(sensor_msgs::PointCloud2 &cloud)
{
    cloud.header.frame_id = _base_frame;
    cloud.header.seq = _seq++;
    cloud.header.stamp = ros::Time::now();
    _registered_publisher.publish(cloud);
}

int32_t colour_registration_st::rgb(uchar r, uchar g, uchar b)
{
    return (static_cast<uint32_t>(r) << 16 | 
        static_cast<uint32_t>(g) << 8 | 
        static_cast<uint32_t>(b) );
}

