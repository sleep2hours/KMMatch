#include "KMMatch.h"

namespace KMMatch{
    void KMMatch::init(ros::NodeHandle &nh)
    {
        first_init=true;
        nh.param("drone_num", drone_num, -1);
        ROS_WARN("drone_num:%d,",drone_num);
        for(int i=0;i<drone_num;i++){
            double _x,_y,_z;
            present_pos.push_back(Eigen::Vector3d(-20.0,-9.0+double(i)*2.0,0.1));
        }
        target_receive=false;
        target_suber=nh.subscribe("/move_base_simple/goal", 1, &KMMatch::TargetCallback, this);
        for(int i=0;i<drone_num;i++){
           pos_sub.push_back(new message_filters::Subscriber<quadrotor_msgs::PositionCommand>(nh,"/drone_"+std::to_string(i)+"_planning/pos_cmd",1)); 
        }    
        sync=new Synchronizer<MySyncPolicy> (MySyncPolicy(10), *pos_sub[0], *pos_sub[1],*pos_sub[2],*pos_sub[3],*pos_sub[4]);
        sync->registerCallback(boost::bind(&KMMatch::OdomCallback,this, _1, _2,_3,_4,_5));
        for (int i=0;i<drone_num;i++){
            goal_puber.emplace_back(nh.advertise<geometry_msgs::PoseStamped>("/drone_"+std::to_string(i)+"_KMmatch_goal",1));
        }
    }

    void KMMatch::OdomCallback(const quadrotor_msgs::PositionCommandConstPtr &msg1,const quadrotor_msgs::PositionCommandConstPtr &msg2,const quadrotor_msgs::PositionCommandConstPtr &msg3,const quadrotor_msgs::PositionCommandConstPtr &msg4,const quadrotor_msgs::PositionCommandConstPtr &msg5){
        present_pos.clear(); 
        present_pos.emplace_back(Eigen::Vector3d(msg1->position.x,msg1->position.y,msg1->position.z));
        present_pos.emplace_back(Eigen::Vector3d(msg2->position.x,msg2->position.y,msg2->position.z));
        present_pos.emplace_back(Eigen::Vector3d(msg3->position.x,msg3->position.y,msg3->position.z));
        present_pos.emplace_back(Eigen::Vector3d(msg4->position.x,msg4->position.y,msg4->position.z));
        present_pos.emplace_back(Eigen::Vector3d(msg5->position.x,msg5->position.y,msg5->position.z));
    }

    void KMMatch::TargetCallback(const geometry_msgs::PoseStampedPtr &msg){
        if(target_pos.size()==drone_num){
            target_pos.clear();
            target_receive=false;
        }
        if(target_pos.size()<drone_num){
            Eigen::Vector3d _tar;
            _tar(0) = msg->pose.position.x;
            _tar(1) = msg->pose.position.y;
            _tar(2) = msg->pose.position.z;
            target_pos.push_back(_tar);
        }
        if(target_pos.size()==drone_num)
        {
            match();   
            // int count = 0;
            // while (ros::ok() && count++ < 100)
            // {
            //     ros::Duration(0.001).sleep();
            // }
        }
        ROS_WARN("Receive Target:%d",target_pos.size());
    }
    
    void KMMatch::match()
    {
        ROS_WARN("Begin Match!");
        std::vector<std::vector<double>> costs_(drone_num, std::vector<double>(drone_num, 0));
        for (int i = 0; i < drone_num; ++i) {
            for (int j = 0; j < drone_num; ++j) {
                Eigen::Vector3d dp=target_pos[i]-present_pos[j];
                costs_[i][j] = dp.norm();
            }
        }
        optimizer_.costs(costs_);
        std::vector<std::pair<size_t, size_t>> assignments;
        optimizer_.Minimize(&assignments);
        quadrotor_msgs::PositionCommand msg_;
        for(int i=0;i<drone_num;i++){
            int tar_ind=static_cast<int>(assignments[i].first);
            int pos_ind=static_cast<int>(assignments[i].second);
            msg_.position.x=target_pos[tar_ind](0);
            msg_.position.y=target_pos[tar_ind](1);
            msg_.position.z=0.0;
            goal_puber[pos_ind].publish(msg_);
        }

    } 
}
