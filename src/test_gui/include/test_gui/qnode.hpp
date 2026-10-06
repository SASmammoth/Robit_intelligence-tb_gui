/**
 * @file /include/test_gui/qnode.hpp
 *
 * @brief Communications central!
 **/
#ifndef test_gui_QNODE_HPP_
#define test_gui_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <opencv2/opencv.hpp>
#include "tb_interfaces/msg/detection_array.hpp"
#endif
#include <QThread>
#include <QImage>
#include <QMap>
#include <QString>
#include <atomic>

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode();
  ~QNode();

  // ====== 파라미터 (이름으로 /lineDetect_node, /path_node 중 보낼 곳을 고름)
  void setParams(const QMap<QString, int> &params);
  bool requestParams();                       // 두 노드 모두 요청했으면 true
  void saveParams(const QString &path);       // 두 노드의 현재 값 → YAML
  void loadParams(const QString &path);       // YAML → 두 노드에 설정

  // ====== 미션
  void publishTurn(int turn);                 // 0 직진, 1 좌, 2 우
  void publishParking(bool on);

protected:
  void run();

private:
  static bool isPathParam(const QString &name);
  static std::vector<std::string> lineParamNames();
  static std::vector<std::string> pathParamNames();

  std::shared_ptr<rclcpp::Node> node;

  // 영상
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr usb_sub_, sign_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr mask_sub_, bev_sub_, roi_sub_, skel_sub_;
  std::atomic<int64_t> last_sign_ns_{0};

  // 상태
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr path_src_sub_;
  rclcpp::Subscription<tb_interfaces::msg::DetectionArray>::SharedPtr det_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr turn_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr parking_sub_;

  // 미션
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr turn_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr parking_pub_;

  // 파라미터
  rclcpp::AsyncParametersClient::SharedPtr line_client_, path_client_;
  bool line_requested_ = false, path_requested_ = false;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_event_handler_;
  rclcpp::ParameterEventCallbackHandle::SharedPtr param_event_cb_handle_;

  // signs 감지 콜백
  void onDetections(const tb_interfaces::msg::DetectionArray &msg);
  QMap<QString, rclcpp::Time> last_seen_;

Q_SIGNALS:
  void rosShutDown();

  // 영상
  void usbImageReceived(const QImage &img);
  void maskImageReceived(const QImage &img);
  void bevImageReceived(const QImage &img);
  void roiImageReceived(const QImage &img);
  void skelImageReceived(const QImage &img);

  // 상태
  void pathSourceReceived(const QString &src);
  void detectionsReceived(const QString &text);
  void turnReceived(int turn);
  void parkingReceived(bool on);

  // 파라미터
  void paramLoaded(const QString &name, int value);
  void paramFileStatus(const QString &text);

};

#endif /* test_gui_QNODE_HPP_ */
