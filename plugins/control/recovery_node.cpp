// Copyright 2025 Lihan Chen
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

#include "pb2025_sentry_behavior/plugins/control/recovery_node.hpp"

#include <string>

namespace pb2025_sentry_behavior
{

RecoveryNode::RecoveryNode(const std::string & name, const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf), current_child_idx_(0), num_attempts_(1), retry_count_(0)
{
}

BT::NodeStatus RecoveryNode::tick()
{
  getInput("num_attempts", num_attempts_);
  const unsigned children_count = children_nodes_.size();

  if (children_count != 2) {
    throw BT::BehaviorTreeException("Recovery Node '" + name() + "' must only have 2 children.");
  }

  setStatus(BT::NodeStatus::RUNNING);

  while (current_child_idx_ < children_count && retry_count_ <= num_attempts_) {
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    if (current_child_idx_ == 0) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS: {
          // reset node and return success when first child returns success
          halt();
          return BT::NodeStatus::SUCCESS;
        }

        case BT::NodeStatus::FAILURE: {
          if (retry_count_ < num_attempts_) {
            // halt first child and tick second child in next iteration
            ControlNode::haltChild(0);
            current_child_idx_++;
            break;
          } else {
            // reset node and return failure when max retries has been exceeded
            halt();
            return BT::NodeStatus::FAILURE;
          }
        }

        case BT::NodeStatus::RUNNING: {
          return BT::NodeStatus::RUNNING;
        }

        default: {
          throw BT::LogicError("A child node must never return IDLE");
        }
      }  // end switch

    } else if (current_child_idx_ == 1) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS: {
          // halt second child, increment recovery count, and tick first child in next iteration
          ControlNode::haltChild(1);
          retry_count_++;
          current_child_idx_--;
        } break;

        case BT::NodeStatus::FAILURE: {
          // reset node and return failure if second child fails
          halt();
          return BT::NodeStatus::FAILURE;
        }

        case BT::NodeStatus::RUNNING: {
          return BT::NodeStatus::RUNNING;
        }

        default: {
          throw BT::LogicError("A child node must never return IDLE");
        }
      }  // end switch
    }
  }  // end while loop

  // reset node and return failure
  halt();
  return BT::NodeStatus::FAILURE;
}

void RecoveryNode::halt()
{
  ControlNode::halt();
  retry_count_ = 0;
  current_child_idx_ = 0;
}

BT::PortsList RecoveryNode::providedPorts()
{
  return {BT::InputPort<int>("num_attempts", 999, "Number of retries")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::RecoveryNode>("RecoveryNode");
}
