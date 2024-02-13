#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "rviz_common/panel.hpp"
#include "rviz_common/load_resource.hpp"

#include "QPushButton"
#include "QLabel"
#include "QHBoxLayout"
#include <QTimer> 

namespace time_count
{
class TimeCountPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TimeCountPanel(QWidget *parent = nullptr)
      : rviz_common::Panel(parent),
        start_time_(std::chrono::steady_clock::now())
  {
    auto *layout = new QVBoxLayout;

    time_label_ = new QLabel("経過時間: 0秒");
    layout->addWidget(time_label_);

    QPushButton *reset_button = new QPushButton("リセット");
    layout->addWidget(reset_button);

    setLayout(layout);

    connect(reset_button, &QPushButton::clicked, this, &TimeCountPanel::resetTimer);

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &TimeCountPanel::updateTime);
    timer_->start(1000);
  }

private Q_SLOTS:
  void resetTimer()
  {
    start_time_ = std::chrono::steady_clock::now();
  }

void updateTime()
{
  auto now = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_);
  int minutes = duration.count() / 60;
  int seconds = duration.count() % 60;
  time_label_->setText("経過時間: " + QString::number(minutes) + "分 " + QString::number(seconds) + "秒");
}

private:
  QLabel *time_label_;
  std::chrono::steady_clock::time_point start_time_;
  QTimer *timer_;
};

}  // namespace time_count
#include "timer_panel.moc"
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(time_count::TimeCountPanel, rviz_common::Panel)
