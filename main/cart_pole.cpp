// Copyright 2021 seigot. All rights reserved.

#include <ros/ros.h>
#include <casadi/casadi.hpp>

namespace ca = casadi;

// simulation variables
constexpr double dT = 0.001;



// model values
constexpr double M = 1;
constexpr double m = 0.1;
constexpr double l = 0.8;
constexpr double g = 9.81;


int main(int argc, char** argv) {
  ros::init(argc, argv, "cart_pole");
  ros::NodeHandle nh;

  ca::DaeBuilder dae;

  // state variables
  auto p = dae.add_x("p");
  auto theta = dae.add_x("theta");
  auto v = dae.add_x("v");
  auto omega = dae.add_x("omega");

  // input variables
  auto u = dae.add_u("u");

  // set range
  dae.set_min(u.name(), -15);
  dae.set_max(u.name(), 15);

  // dae
  auto a = (-l*m*sin(theta)*pow(omega,2) + u + g*m*cos(theta)*sin(theta))/
           (M + m - m*pow(cos(theta),2));
  auto dw = (- l*m*cos(theta)*sin(theta)*pow(omega,2)
             + u*cos(theta) + g*m*sin(theta)
             + M*g*sin(theta))/(l*(M + m - m*pow(cos(theta),2)));
  dae.add_ode(p.name(), v);
  dae.add_ode(theta.name(), omega);
  dae.add_ode(v.name(), a);
  dae.add_ode(omega.name(), dw);

  // initial state
  dae.set_start(p.name(), 0);
  dae.set_start(v.name(), 0);
  dae.set_start(theta.name(), ca::pi);
  dae.set_start(omega.name(), 0);

  // add meta information
  dae.ode;

  // result
  // ocp.initialize('theta', [0, 1], [pi, 0])








  ros::Rate loop_rate(100);
  int count = 0;
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

