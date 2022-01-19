/*
 * Copyright 2022 CLOBOT Co., Ltd
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

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "clober_serial/clober_serial.hpp"

int main(int argc, char ** argv)
{
  std::cout << "clober_serial" << std::endl;

  rclcpp::init(argc, argv);
  // rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::executors::SingleThreadedExecutor executor;

  auto clober_serial = std::make_shared<CloberSerial>();
  executor.add_node(clober_serial);

  executor.spin();
  rclcpp::shutdown();

  return 0;
}
