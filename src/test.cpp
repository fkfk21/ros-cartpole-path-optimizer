// Copyright 2021 seigot. All rights reserved.

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include "path_optimizer/DirectTranscriptSolver.hpp"
#include "path_optimizer/DirectCollocationSolver.hpp"

int main(int argc, char** argv) {
    Settings settings;
    settings.phaseLength = 500;
    settings.time = 5;
    settings._costWeights.control = 1;
    settings.solverVerbosity = 1;
    settings.ipoptLinearSolver = "mumps";

    State initialState, finalState;
    initialState.state.zeros(4);
    finalState.state = {1, 3.14, 0, 0};

    cart_pole_model model = {0.5, 1, 0.3, 9.81};
    constraint_value constraint = {0, 2, -100, 100};

    DirectTransSolver solver(model, constraint);
    solver.setTranscriptMethod(TranscriptMethod::SINGLE_SHOOTING);
    solver.setupProblem(settings);
    bool ok = solver.solve(initialState, finalState);

    casadi::DM sol_state, sol_control;
    if (ok) solver.getSolution(sol_state, sol_control);
    std::cout << "single shooting transcript\n";
    std::cout << "state:\n" << sol_state
              << "\ncontrol:\n" << sol_control << "\n\n";

    ros::init(argc, argv, "cart_pole");
    ros::NodeHandle nh;

    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>(
                                "cartpole_marker", 1);

    ros::Rate loop_rate(settings.phaseLength/settings.time);
    ROS_INFO("size: [%d,%d]", sol_state.size().first, sol_state.size().second);
    int i = 0;
    ros::Duration(5).sleep();
    while (ros::ok()) {
      double p = sol_state(0,i).get_elements().front();
      double theta = sol_state(1,i).get_elements().front();
      double v = sol_state(2,i).get_elements().front();
      double omega = sol_state(3,i).get_elements().front();

      ROS_INFO("time:%f position:%f",
               static_cast<double>(i)/settings.phaseLength*settings.time, p);

      // position
      const float s = 0.1;
      const float l = model.l;
      geometry_msgs::Point start;
      start.x = p;
      start.y = 0;
      start.z = s/2;
      geometry_msgs::Point end;
      end.x = p + l*sin(theta);
      end.y = 0;
      end.z = s/2 - l*cos(theta);

      // cart
      visualization_msgs::Marker cart;
      cart.header.frame_id = "map";
      cart.header.stamp = ros::Time();
      cart.ns = "";
      cart.id = 0;
      cart.type = visualization_msgs::Marker::CUBE;
      cart.action = visualization_msgs::Marker::ADD;
      cart.pose.position = start;
      cart.pose.orientation.x = 0.0;
      cart.pose.orientation.y = 0.0;
      cart.pose.orientation.z = 0.0;
      cart.pose.orientation.w = 1.0;
      cart.scale.x = s;
      cart.scale.y = s;
      cart.scale.z = s;
      cart.color.a = 1.0;  // Don't forget to set the alpha!
      cart.color.r = 0.0;
      cart.color.g = 1.0;
      cart.color.b = 0.0;

      // arrow
      visualization_msgs::Marker pole;
      pole.header.frame_id = "map";
      pole.header.stamp = ros::Time();
      pole.ns = "";
      pole.id = 1;
      pole.type = visualization_msgs::Marker::ARROW;
      pole.action = visualization_msgs::Marker::ADD;
      pole.points.resize(2);
      pole.points[0] = start;
      pole.points[1] = end;
      pole.scale.x = s/7;
      pole.scale.y = s/3;
      pole.scale.z = l/8;
      pole.color.a = 1.0;  // Don't forget to set the alpha!
      pole.color.r = 1.0;
      pole.color.g = 1.0;
      pole.color.b = 0.0;

      // sphere
      visualization_msgs::Marker sphere;
      sphere.header.frame_id = "map";
      sphere.header.stamp = ros::Time();
      sphere.ns = "";
      sphere.id = 2;
      sphere.type = visualization_msgs::Marker::SPHERE;
      sphere.action = visualization_msgs::Marker::ADD;
      sphere.pose.position = end;
      sphere.pose.orientation.x = 0.0;
      sphere.pose.orientation.y = 0.0;
      sphere.pose.orientation.z = 0.0;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = 0.05;
      sphere.scale.y = 0.05;
      sphere.scale.z = 0.05;
      sphere.color.a = 1.0;  // Don't forget to set the alpha!
      sphere.color.r = 0.0;
      sphere.color.g = 0.0;
      sphere.color.b = 1.0;

      visualization_msgs::MarkerArray marker_array;
      marker_array.markers.push_back(cart);
      marker_array.markers.push_back(pole);
      marker_array.markers.push_back(sphere);
      vis_pub.publish(marker_array);

      if (i < settings.phaseLength) {
        i++;
      }
      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
}
