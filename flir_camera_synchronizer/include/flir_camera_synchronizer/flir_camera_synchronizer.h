#ifndef FLIR_CAMERA_SYNCHRONIZER_H
#define FLIR_CAMERA_SYNCHRONIZER_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <image_transport/image_transport.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace flir_camera_synchronizer
{

class FlirCameraSynchronizer : public rclcpp::Node
{
public:
  explicit FlirCameraSynchronizer(const rclcpp::NodeOptions & options);
  ~FlirCameraSynchronizer();

  // --------
  // Typedefs
  // --------

  typedef sensor_msgs::msg::Image ImageMsg;
  typedef std::shared_ptr<sensor_msgs::msg::Image> ImageMsgPtr;
  typedef std::shared_ptr<sensor_msgs::msg::Image const> ImageMsgConstPtr;

  typedef message_filters::Subscriber<ImageMsg> ImageSubscriber;
  typedef std::shared_ptr<ImageSubscriber> ImageSubscriberPtr;

  typedef image_transport::Publisher ImagePublisher;

  typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg> SyncPolicy2;
  typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg> SyncPolicy3; 
  typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg, ImageMsg> SyncPolicy4; 
  typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg> SyncPolicy5;
  typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg> SyncPolicy6;
  typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg> SyncPolicy7;
  typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg> SyncPolicy8;

private:
  // -------
  // Helpers
  // -------
  
  void syncTimeStamps(std::vector<ImageMsgPtr>& msgs);

  // ---------
  // Callbacks
  // ---------

  void callback2(const ImageMsgConstPtr&, const ImageMsgConstPtr&);
  void callback3(const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&);
  void callback4(const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&);
  void callback5(const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&);
  void callback6(const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&);
  void callback7(const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&);
  void callback8(const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&, const ImageMsgConstPtr&);

  // ----------
  // Publishers
  // ----------

  std::vector<ImagePublisher> m_publishers;

  // -----------------------
  // Message Filter Entities
  // -----------------------

  std::vector<ImageSubscriberPtr> m_subscribers;

  std::shared_ptr<message_filters::Synchronizer<SyncPolicy2>> m_sync2;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy3>> m_sync3;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy4>> m_sync4;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy5>> m_sync5;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy6>> m_sync6;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy7>> m_sync7;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy8>> m_sync8;
};

}  // namespace flir_camera_synchronizer

#endif  // FLIR_CAMERA_SYNCHRONIZER_H
