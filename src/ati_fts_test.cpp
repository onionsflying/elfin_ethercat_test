/*
Created on Mon Dec 17 09:02:30 2018

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2018, Han's Robot Co., Ltd.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
// author: Cong Liu

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <elfin_ethercat_driver_v2/elfin_ethercat_manager_v2.h>

int32_t readInput_unit(elfin_ethercat_driver_v2::EtherCatManager* manager, int slave_no, uint8_t channel)
{
    uint8_t map[4];
    for(int i=0; i<4; i++)
    {
        map[i]=manager->readInput(slave_no, channel+i);
    }
    int32_t value_tmp=*(int32_t *)(map);
    return value_tmp;
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"ati_fts_test", ros::init_options::AnonymousName);

  ros::NodeHandle root_nh;
  ros::NodeHandle nh("~");

  std::string ethernet_name;
  ethernet_name=nh.param<std::string>("elfin_ethernet_name", "eth0");

  int slave_number;
  slave_number=nh.param<int>("slave_number", 1);

  std::string ft_frame;
  ft_frame=nh.param<std::string>("ft_frame", "elfin_ft_sensor");

  ros::Publisher wrench_pub=root_nh.advertise<geometry_msgs::WrenchStamped>("ft_sensor/ft_raw", 1);

  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.header.frame_id=ft_frame;
  std::vector<double> ft_buffer;
  ft_buffer.resize(6);

  elfin_ethercat_driver_v2::EtherCatManager ethercat_manager(ethernet_name);

  ros::Rate r(100);

  ROS_INFO("Start publishing wrench messages");

  while(ros::ok())
  {
    wrench_msg.header.stamp=ros::Time::now();
    for(int i=0; i<6; i++)
    {
      int32_t FT=readInput_unit(&ethercat_manager, slave_number, i*4);
      ft_buffer[i]=FT/10000.0;
    }
    wrench_msg.wrench.force.x=ft_buffer[0];
    wrench_msg.wrench.force.y=ft_buffer[1];
    wrench_msg.wrench.force.z=ft_buffer[2];
    wrench_msg.wrench.torque.x=ft_buffer[3];
    wrench_msg.wrench.torque.y=ft_buffer[4];
    wrench_msg.wrench.torque.z=ft_buffer[5];

    wrench_pub.publish(wrench_msg);

    r.sleep();
  }

  return 0;
}
