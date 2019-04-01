/*
 * The MIT License
 *
 * Copyright (c) 2019 Giovanni di Dio Bruno https://gbr1.github.io
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "sengi_ros/Feedback.h"

#include "string.h"

class SengiJoints : public hardware_interface::RobotHW{
    public:
        SengiJoints(){
            std::string joint_names[2];
            joint_names[0]="left_joint";
            joint_names[1]="right_joint";
            for (int i=0; i < 2; i++){
                hardware_interface::JointStateHandle joint_state_handle(joint_names[i],&joints_[i].position,&joints_[i].velocity,&joints_[i].effort);
                joint_state_interface_.registerHandle(joint_state_handle);
                
                hardware_interface::JointHandle joint_handle(joint_state_handle, &joints_[i].velocity_command);
                velocity_joint_interface_.registerHandle(joint_handle);
            }
        registerInterface(&joint_state_interface_);
        registerInterface(&velocity_joint_interface_);
        }

        void callback(const sengi_ros::Feedback& msg){
            joints_[0].position = msg.drivers[0].measured_travel;
            joints_[1].position = msg.drivers[1].measured_travel;
            joints_[0].velocity = msg.drivers[0].measured_velocity;
            joints_[1].velocity = msg.drivers[1].measured_velocity;
            joints_[0].effort = 0.0;
            joints_[1].effort = 0.0;
        }

        float getCommand(int i){
            return static_cast<float>(joints_[i].velocity_command);
        }

    private:
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;


        struct Joint{
            double position;
            double velocity;
            double effort;
            double velocity_command;

            Joint() : position(0), velocity(0), effort(0), velocity_command(0){}
        }
        joints_[2];
};