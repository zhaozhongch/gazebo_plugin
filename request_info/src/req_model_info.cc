#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/msgs/pose_animation.pb.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>

#include <gazebo/physics/physics.hh>

#include "req_model_info.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ShowInfo);

/////////////////////////////////////////////////
ShowInfo::ShowInfo()
{
    // int argc;
    // char** argv;
    // ros::init(argc, argv,"model_info_node");
    //rosservice call /model_info [False]
    std::cout<<"service registration, call /model_info to get model information..................................."<<std::endl;
    service_ = nh_.advertiseService("/model_info", &gazebo::ShowInfo::GetModelInfo, this);
}

void ShowInfo::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{


    // Store the pointer to the model
    this->model = _parent;

    //std::cout<<"model loaded....................................................."<<std::endl;

    // auto box_size = this->model->BoundingBox();
    // std::cout<<"model name "<<this->model->GetName()<<std::endl;
    // std::cout<<"the size of the bounding box "<<box_size.XLength()<<","<<box_size.YLength()<<","<<box_size.ZLength()<<std::endl;
    // std::cout<<"the center of the bounding box "<<box_size.Center().X()<<","<<box_size.Center().Y()<<","<<box_size.Center().Z()<<std::endl;

    // //model collosion bounding box
    // auto mc = this->model->CollisionBoundingBox();
    // //collision bbox can give us the correct 3D dimension of the object while the model bbox cannnot always give us the correct value. I am not sure why
    // std::cout<<"collosion bounding box "<<mc.XLength()<<","<<mc.YLength()<<","<<mc.ZLength()<<std::endl;

    // auto all_links = this->model->GetLinks();
    // for(int i = 0; i<all_links.size(); i++){
    //     physics::LinkPtr link = all_links[i];
    //     ignition::math::v4::Box bbox = link->CollisionBoundingBox();
    //     std::cout<<"dimension of link "<<link->GetName()<<","<<bbox.XLength()<<","<<bbox.YLength()<<","<<bbox.ZLength()<<std::endl;
    // }

    /*Child class to the parent class is safe*/
    // entity = _parent;
    // auto cbox = entity->CollisionBoundingBox();
    // same as mc
    // std::cout<<"collosion bounding box "<<cbox.XLength()<<","<<cbox.YLength()<<","<<cbox.ZLength()<<std::endl;

}

bool ShowInfo::GetModelInfo(request_info::GetModelInfo::Request &req, request_info::GetModelInfo::Response &res){
    std_msgs::Float64MultiArray location;
    std_msgs::Float64MultiArray orientation;
    std_msgs::Float64MultiArray bboxes;
    std_msgs::String names;
    std_msgs::Int64 num;
    //if request type is true, then we return the model's position and bounding box, otherwise we return this model's all links position and bounding box
    if(req.request_type.data){
        auto bbox = this->model->CollisionBoundingBox();
        auto pose = this->model->WorldPose();

        names.data += this->model->GetName();

        bboxes.data.push_back(bbox.XLength());
        bboxes.data.push_back(bbox.YLength());
        bboxes.data.push_back(bbox.ZLength());

        location.data.push_back(pose.Pos().X());
        location.data.push_back(pose.Pos().Y());
        location.data.push_back(pose.Pos().Z());

        orientation.data.push_back(pose.Rot().Euler().X());
        orientation.data.push_back(pose.Rot().Euler().Y());
        orientation.data.push_back(pose.Rot().Euler().Z());

        num.data = 1;
    }
    else{
        auto all_links = this->model->GetLinks();
        num.data = all_links.size();
        for(int i = 0; i<all_links.size(); i++){
            physics::LinkPtr link = all_links[i];
            ignition::math::v4::Box bbox = link->CollisionBoundingBox();
            names.data += link->GetName();
            names.data += ",";
            
            location.data.push_back(link->WorldPose().Pos().X());
            location.data.push_back(link->WorldPose().Pos().Y());
            location.data.push_back(link->WorldPose().Pos().Z());

            //order is roll pitch and yaw
            orientation.data.push_back(link->WorldPose().Rot().Euler().X());
            orientation.data.push_back(link->WorldPose().Rot().Euler().Y());
            orientation.data.push_back(link->WorldPose().Rot().Euler().Z());

            bboxes.data.push_back(bbox.XLength());
            bboxes.data.push_back(bbox.YLength());
            bboxes.data.push_back(bbox.ZLength());
        }
        //remove the last ','
        names.data.pop_back();
    }



    //TODO, add orientation
    res.dimensions = bboxes;
    res.locations  = location;
    res.names      = names;
    res.orientations = orientation;
    res.num = num;

    return true;
};