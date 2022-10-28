// Copyright 2021 seigot. All rights reserved.

#include <vector>
#include <iostream>
#include <casadi/casadi.hpp>

namespace ca = casadi;

int main() {
ca::MX x = ca::MX::sym("x");
ca::MX y = ca::MX::sym("y");
ca::MX z = ca::MX::sym("z");
ca::MX f = pow(x,2)+100*pow(z,2.0);
ca::MX g = z+pow(1-x,2)-y;

ca::MXDict nlp;                 // NLP declaration
nlp["x"] = vertcat(x,y,z);  // decision vars
nlp["f"] = f;               // objective
nlp["g"] = g;               // constraints

// Create solver instance
ca::Function F = ca::nlpsol("F","ipopt",nlp);

// Solve the problem using a guess
auto sol = F(ca::DMDict{{"x0",ca::DM({2.5,3.0,0.75})},{"ubg",0},{"lbg",0}});

for (auto& [a,b] : sol) {
  std::cout << a << "  :  " << b << std::endl;
}
}

// int main() {
//   auto opti = ca::Opti();
//   auto x = opti.variable();
//   auto y = opti.variable();
//   auto z = opti.variable();

//   opti.minimize(x*x + 100*z*z);
//   opti.subject_to(z+(1-x)*(1-x)-y == 0);
//   opti.solver("ipopt");

//   auto sol = opti.solve();

//   std::cout << sol.value(x) << ":" << sol.value(y) << std::endl;
//   for (auto& v : sol.value_variables()) {
//     std::cout << "variable"<< v.name() << " :" << v.get_str() << std::endl;
//   }
//   sol.disp(std::cout);
// }
