#pragma once

#include "rclcpp/rclcpp.hpp"

namespace communicator
{
class Communicator : public rclcpp::Node
{
public:
    Communicator() : Node("communicator")
    {

    }
    ~Communicator() = default;

private:

};
}  // namespace communicator
