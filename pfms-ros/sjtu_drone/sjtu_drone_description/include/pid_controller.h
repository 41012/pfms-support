// Copyright 2023 Georg Novotny
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H


#include <gazebo/gazebo.hh>
#include <gazebo/common/Console.hh>

class PIDController {
public:
  PIDController();
  virtual ~PIDController();
  virtual void Load(sdf::ElementPtr _sdf, const std::string& prefix = "");

  double gain_p;
  double gain_i;
  double gain_d;
  double time_constant;
  double limit;

  double input;
  double dinput;
  double output;
  double p, i, d;

  double update(double input, double x, double dx, double dt);
  void reset();
};

#endif // PIDCONTROLLER_H
