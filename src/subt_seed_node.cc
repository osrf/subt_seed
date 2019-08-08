/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <geometry_msgs/Twist.h>
#include <subt_msgs/PoseFromArtifact.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include <string>

#include <subt_communication_broker/subt_communication_client.h>
#include <subt_ign/CommonTypes.hh>
#include <subt_ign/protobuf/artifact.pb.h>

/// \brief. Example control class, running as a ROS node to control a robot.
class Controller
{
  /// \brief Constructor.
  /// The controller uses the given name as a prefix of its topics, e.g.,
  /// "/x1/cmd_vel" if _name is specified as "x1".
  /// \param[in] _name Name of the robot.
  public: Controller(const std::string &_name);

  /// \brief A function that will be called every loop of the ros spin
  /// cycle.
  public: void Update();

  /// \brief Callback function for message from other comm clients.
  /// \param[in] _srcAddress The address of the robot who sent the packet.
  /// \param[in] _dstAddress The address of the robot who received the packet.
  /// \param[in] _dstPort The destination port.
  /// \param[in] _data The contents the packet holds.
  private: void CommClientCallback(const std::string &_srcAddress,
                                   const std::string &_dstAddress,
                                   const uint32_t _dstPort,
                                   const std::string &_data);

  /// \brief ROS node handler.
  private: ros::NodeHandle n;

  /// \brief publisher to send cmd_vel
  private: ros::Publisher velPub;

  /// \brief Communication client.
  private: std::unique_ptr<subt::CommsClient> client;

  /// \brief Client to request pose from origin.
  private: ros::ServiceClient originClient;

  /// \brief Service to request pose from origin.
  private: subt_msgs::PoseFromArtifact originSrv;

  /// \brief True if robot has arrived at destination.
  private: bool arrived{false};

  /// \brief True if we have successfully sent the start signal.
  private: bool started{false};
};

/////////////////////////////////////////////////
Controller::Controller(const std::string &_name)
{
  // Create subt communication client
  this->client.reset(new subt::CommsClient(_name));
  this->client->Bind(&Controller::CommClientCallback, this);

  // Create a cmd_vel publisher to control a vehicle.
  this->velPub = this->n.advertise<geometry_msgs::Twist>(_name + "/cmd_vel", 1);

  // Create a cmd_vel publisher to control a vehicle.
  this->originClient = this->n.serviceClient<subt_msgs::PoseFromArtifact>(
      "/subt/pose_from_artifact_origin");
  this->originSrv.request.robot_name.data = _name;
}

/////////////////////////////////////////////////
void Controller::CommClientCallback(const std::string &_srcAddress,
                                    const std::string &_dstAddress,
                                    const uint32_t _dstPort,
                                    const std::string &_data)
{
  subt::msgs::ArtifactScore res;
  if (!res.ParseFromString(_data))
  {
    ROS_ERROR("CommClientCallback(): Error deserializing message.");
  }

  // Add code to handle communication callbacks.
  ROS_INFO("Message from [%s] to [%s] on port [%u]:\n [%s]", _srcAddress.c_str(),
      _dstAddress.c_str(), _dstPort, res.DebugString().c_str());
}

/////////////////////////////////////////////////
void Controller::Update()
{
  if (!this->started)
  {
    // Send start signal
    std_srvs::SetBool::Request req;
    std_srvs::SetBool::Response res;
    req.data = true;
    if (!ros::service::call("/subt/start", req, res))
    {
      ROS_ERROR_ONCE("Unable to send start signal.");
    }

    if (!res.success)
    {
      ROS_ERROR_ONCE("Failed to send start signal [%s]", res.message.c_str());
    }
    else
    {
      this->started = true;
      ROS_INFO("Sent start signal.");
    }
  }

  // Add code that should be processed every iteration.

  if (this->arrived)
    return;

  // Query current robot position w.r.t. entrance
  if (!this->originClient.call(this->originSrv) ||
      !this->originSrv.response.success)
  {
    ROS_ERROR_ONCE("Failed to call pose_from_artifact_origin service, \
robot may not exist, or be outside staging area.");

    // Stop robot
    geometry_msgs::Twist msg;
    this->velPub.publish(msg);

    // If out of range once, we consider it arrived
    this->arrived = true;

    return;
  }

  auto pose = this->originSrv.response.pose.pose;

  // Simple example for robot to go to entrance
  geometry_msgs::Twist msg;

  // Distance to goal
  double dist = pose.position.x * pose.position.x + pose.position.y * pose.position.y;

  // Arrived
  if (dist < 0.3 || pose.position.x >= -0.3)
  {
    msg.linear.x = 0;
    msg.angular.z = 0;
    this->arrived = true;
    ROS_INFO("Arrived at entrance!");

    // Report an artifact
    // Hardcoded to tunnel_circuit_practice_01's exginguisher_3
    subt::msgs::Artifact artifact;
    artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_EXTINGUISHER));
    artifact.mutable_pose()->mutable_position()->set_x(-8.1);
    artifact.mutable_pose()->mutable_position()->set_y(37);
    artifact.mutable_pose()->mutable_position()->set_z(0.004);

    std::string serializedData;
    if (!artifact.SerializeToString(&serializedData))
    {
      ROS_ERROR("ReportArtifact(): Error serializing message [%s]",
          artifact.DebugString().c_str());
    }
    else if (!this->client->SendTo(serializedData, subt::kBaseStationName))
    {
      ROS_ERROR("CommsClient failed to Send serialized data.");
    }
  }
  // Move towards entrance
  else
  {
    // Yaw w.r.t. entrance
    // Quaternion to yaw:
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code_2
    auto q = pose.orientation;
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    auto yaw = atan2(siny_cosp, cosy_cosp);

    auto facingFront = abs(yaw) < 0.1;
    auto facingEast = abs(yaw + M_PI * 0.5) < 0.1;
    auto facingWest = abs(yaw - M_PI * 0.5) < 0.1;

    auto onCenter = abs(pose.position.y) <= 1.0;
    auto westOfCenter = pose.position.y > 1.0;
    auto eastOfCenter = pose.position.y < -1.0;

    double linVel = 3.0;
    double angVel = 1.5;

    // Robot is facing entrance
    if (facingFront && onCenter)
    {
      msg.linear.x = linVel;
      msg.angular.z = angVel * -yaw;
    }
    // Turn to center line
    else if (!facingEast && westOfCenter)
    {
      msg.angular.z = -angVel;
    }
    else if (!facingWest && eastOfCenter)
    {
      msg.angular.z = angVel;
    }
    // Go to center line
    else if (facingEast && westOfCenter)
    {
      msg.linear.x = linVel;
    }
    else if (facingWest && eastOfCenter)
    {
      msg.linear.x = linVel;
    }
    // Center line, not facing entrance
    else if (onCenter && !facingFront)
    {
      msg.angular.z = angVel * -yaw;
    }
    else
    {
      ROS_ERROR("Unhandled case");
    }
  }

  this->velPub.publish(msg);
}

/////////////////////////////////////////////////
int main(int argc, char** argv)
{
  if (argc < 2)
  {
    ROS_ERROR_STREAM("Needs an argument for the competitor's name.");
    return -1;
  }

  // Initialize ros
  ros::init(argc, argv, argv[1]);

  ROS_INFO("Starting seed competitor\n");

  // Create the controller
  Controller controller(argv[1]);

  // This sample code iteratively calls Controller::Update. This is just an
  // example. You can write your controller using alternative methods.
  // To get started with ROS visit: http://wiki.ros.org/ROS/Tutorials
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    controller.Update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
