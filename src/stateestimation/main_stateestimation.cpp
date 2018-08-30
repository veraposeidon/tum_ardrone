 /**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include "EstimationNode.h"
// 这个头文件在EstimationNode中，由配置文件所生成。
//#include "tum_ardrone/StateestimationParamsConfig.h"
#include "ros/ros.h"
#include "PTAMWrapper.h"
#include "MapView.h"


// this global var is used in getMS(ros::Time t) to convert to a consistent integer timestamp used internally pretty much everywhere.
// kind of an artifact from Windows-Version, where only that was available / used.
unsigned int ros_header_timestamp_base = 0;


int main(int argc, char **argv)
{
    // 初始化drone_stateestimation节点
    ros::init(argc, argv, "drone_stateestimation");

    ROS_INFO("Started TUM ArDrone Stateestimation Node.");

    // 建立一个姿态估计 对象
    EstimationNode estimator;

    // 创建了一个参数动态配置的服务端实例，参数配置的类型就是配置文件中描述的类型
    // 该服务端实例会监听客户端的参数配置请求。
    dynamic_reconfigure::Server<tum_ardrone::StateestimationParamsConfig> srv;

    // 定义一个回调函数，并将回调函数和服务端绑定，当客户端请求修改参数时，服务端即可跳转到回调函数中进行处理。
    dynamic_reconfigure::Server<tum_ardrone::StateestimationParamsConfig>::CallbackType f;
    f = boost::bind(&EstimationNode::dynConfCb, &estimator, _1, _2);
    srv.setCallback(f);

    // 启动PTAM封装器
    estimator.ptamWrapper->startSystem();
    // 启动位姿图显示
    estimator.mapView->startSystem();

    // main pose-estimation loop
    estimator.Loop();

    // 关闭
    estimator.mapView->stopSystem();
    estimator.ptamWrapper->stopSystem();

    return 0;
}

