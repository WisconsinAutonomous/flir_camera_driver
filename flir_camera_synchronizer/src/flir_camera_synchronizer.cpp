#include "flir_camera_synchronizer/flir_camera_synchronizer.h"

#include <functional>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::placeholders;

namespace flir_camera_synchronizer
{

FlirCameraSynchronizer::FlirCameraSynchronizer(const rclcpp::NodeOptions & options)
: rclcpp::Node("flir_camera_synchronizer", options)
{
  // ----------------------
  // Declare/Get Parameters
  // ----------------------
  int num_topics = this->declare_parameter<int>("num_topics", 2);

  if (num_topics < 2 or num_topics > 8) {
    RCLCPP_ERROR(this->get_logger(), "Number of topics must be between 2 and 8. Got %d.", num_topics);
    exit(-1);
  }

  // -----------------------------
  // Create Publishers/Subscribers
  // -----------------------------
  const auto qos = rmw_qos_profile_default;
  for (int i = 0; i < num_topics; i++) {
    auto input = "~/input/topic_" + std::to_string(i);
    auto output = "~/output/topic_" + std::to_string(i);

    RCLCPP_INFO(this->get_logger(), "Creating subscriber for topic '%s'.", input.c_str());
    auto subscriber = std::make_shared<ImageSubscriber>(this, input, qos);
    m_subscribers.push_back(subscriber);


    RCLCPP_INFO(this->get_logger(), "Creating publisher for topic '%s'.", output.c_str());
    auto publisher = image_transport::create_publisher(this, output);
    m_publishers.push_back(publisher);
  }

  // ----------------------------------
  // Create messsage_filter Sync Policy
  // ----------------------------------
  using namespace std::placeholders;
  if (num_topics == 2) {
    m_sync2 = std::make_shared<message_filters::Synchronizer<SyncPolicy2>>(SyncPolicy2(10), *m_subscribers[0], *m_subscribers[1]);
    m_sync2->registerCallback(std::bind(&FlirCameraSynchronizer::callback2, this, _1, _2));
  }
  else if (num_topics == 3) {
    m_sync3 = std::make_shared<message_filters::Synchronizer<SyncPolicy3>>(SyncPolicy3(10), *m_subscribers[0], *m_subscribers[1], *m_subscribers[2]);
    m_sync3->registerCallback(std::bind(&FlirCameraSynchronizer::callback3, this, _1, _2, _3));
  }
  else if (num_topics == 4) {
    m_sync4 = std::make_shared<message_filters::Synchronizer<SyncPolicy4>>(SyncPolicy4(10), *m_subscribers[0], *m_subscribers[1], *m_subscribers[2], *m_subscribers[3]);
    m_sync4->registerCallback(std::bind(&FlirCameraSynchronizer::callback4, this, _1, _2, _3, _4));
  }
  else if (num_topics == 5) {
    m_sync5 = std::make_shared<message_filters::Synchronizer<SyncPolicy5>>(SyncPolicy5(10), *m_subscribers[0], *m_subscribers[1], *m_subscribers[2], *m_subscribers[3], *m_subscribers[4]);
    m_sync5->registerCallback(std::bind(&FlirCameraSynchronizer::callback5, this, _1, _2, _3, _4, _5));
  }
}

FlirCameraSynchronizer::~FlirCameraSynchronizer() {}

void FlirCameraSynchronizer::syncTimeStamps(std::vector<ImageMsgPtr>& msgs) {
  rclcpp::Time min_time;
  for (auto msg : msgs) {
    const rclcpp::Time time = message_filters::message_traits::TimeStamp<ImageMsg>::value(*msg);
    if (time < min_time)
      min_time = time;
  }

  for (auto msg: msgs) 
    msg->header.stamp = min_time;
}

void FlirCameraSynchronizer::callback2(const ImageMsgConstPtr& msg1, const ImageMsgConstPtr& msg2) {
  std::vector<ImageMsgPtr> msgs = {
    std::const_pointer_cast<ImageMsg>(msg1),
    std::const_pointer_cast<ImageMsg>(msg2),
  };
  syncTimeStamps(msgs);

  for (unsigned int i = 0; i < msgs.size(); i++)
    m_publishers[i].publish(*msgs[i]);
}

void FlirCameraSynchronizer::callback3(const ImageMsgConstPtr& msg1, const ImageMsgConstPtr& msg2, const ImageMsgConstPtr& msg3) {
  std::vector<ImageMsgPtr> msgs = {
    std::const_pointer_cast<ImageMsg>(msg1),
    std::const_pointer_cast<ImageMsg>(msg2),
    std::const_pointer_cast<ImageMsg>(msg3),
  };
  syncTimeStamps(msgs);

  for (unsigned int i = 0; i < msgs.size(); i++)
    m_publishers[i].publish(*msgs[i]);
}

void FlirCameraSynchronizer::callback4(const ImageMsgConstPtr& msg1, const ImageMsgConstPtr& msg2, const ImageMsgConstPtr& msg3, const ImageMsgConstPtr& msg4) {
  std::vector<ImageMsgPtr> msgs = {
    std::const_pointer_cast<ImageMsg>(msg1),
    std::const_pointer_cast<ImageMsg>(msg2),
    std::const_pointer_cast<ImageMsg>(msg3),
    std::const_pointer_cast<ImageMsg>(msg4),
  };
  syncTimeStamps(msgs);

  for (unsigned int i = 0; i < msgs.size(); i++)
    m_publishers[i].publish(*msgs[i]);
}

void FlirCameraSynchronizer::callback5(const ImageMsgConstPtr& msg1, const ImageMsgConstPtr& msg2, const ImageMsgConstPtr& msg3, const ImageMsgConstPtr& msg4, const ImageMsgConstPtr& msg5) {
  std::vector<ImageMsgPtr> msgs = {
    std::const_pointer_cast<ImageMsg>(msg1),
    std::const_pointer_cast<ImageMsg>(msg2),
    std::const_pointer_cast<ImageMsg>(msg3),
    std::const_pointer_cast<ImageMsg>(msg4),
    std::const_pointer_cast<ImageMsg>(msg5),
  };
  syncTimeStamps(msgs);

  for (unsigned int i = 0; i < msgs.size(); i++)
    m_publishers[i].publish(*msgs[i]);
}

}  // namespace flir_camera_synchronizer

RCLCPP_COMPONENTS_REGISTER_NODE(flir_camera_synchronizer::FlirCameraSynchronizer)
