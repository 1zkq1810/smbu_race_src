// RMUL 2027 哨兵任务层 v0.1
//
// 它只回答三个问题：在哪（读裁判数据）、去哪（选目标点）、走不走（发/取消 NavigateToPose）。
// 它不回答：怎么走（Nav2 的活）、定位准不准（point_lio + small_gicp 的活）、打不打人（云台/自瞄的活）。
//
// 设计约束（都来自规则，不是偏好）：
//   1. 2027 哨兵全自动、0 操作手 → 没有人按按钮，所有决策必须自己判。
//   2. 比赛期间禁止多机通信 → 队友信息拿不到，只能信裁判系统话题。
//   3. 启动区兼补给区 → home 点就是回血点，低血往回跑是唯一天然的自救手段。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "pb_rm_interfaces/msg/buff.hpp"
#include "pb_rm_interfaces/msg/event_data.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

namespace rmul2027
{

// 任务层自己的状态，和 Nav2 的状态无关。
enum class TaskState : uint8_t
{
  WAIT = 0,       // 比赛没开始 / 已结束 / 裁判链路没通 —— 不动
  HOLD = 1,       // 比赛中但不需要动 —— 不动
  GO_HOME = 2,    // 低血回启动区补血
  GO_CENTER = 3,  // 抢中心增益点
  GO_HIGHLAND = 4,// 上高地
  PATROL = 5,     // 按巡逻点巡场
};

const char * stateName(TaskState s)
{
  switch (s) {
    case TaskState::WAIT: return "WAIT";
    case TaskState::HOLD: return "HOLD";
    case TaskState::GO_HOME: return "GO_HOME";
    case TaskState::GO_CENTER: return "GO_CENTER";
    case TaskState::GO_HIGHLAND: return "GO_HIGHLAND";
    case TaskState::PATROL: return "PATROL";
  }
  return "UNKNOWN";
}

struct Waypoint
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

class SentryTaskNode : public rclcpp::Node
{
public:
  SentryTaskNode()
  : Node("sentry_task_node")
  {
    // ---------- 参数 ----------
    dry_run_ = declare_parameter<bool>("dry_run", true);
    navigate_action_ = declare_parameter<std::string>("navigate_action", "navigate_to_pose");
    const double rate_hz = declare_parameter<double>("decision_rate_hz", 2.0);
    low_hp_ratio_ = declare_parameter<double>("low_hp_ratio", 0.50);
    resume_hp_ratio_ = declare_parameter<double>("resume_hp_ratio", 0.90);
    enable_center_gain_ = declare_parameter<bool>("enable_center_gain", true);
    enable_highland_ = declare_parameter<bool>("enable_highland", false);
    enable_patrol_ = declare_parameter<bool>("enable_patrol", false);

    home_.x = declare_parameter<double>("home_x", 0.0);
    home_.y = declare_parameter<double>("home_y", 0.0);
    home_.yaw = declare_parameter<double>("home_yaw", 0.0);

    center_.x = declare_parameter<double>("center_x", 0.0);
    center_.y = declare_parameter<double>("center_y", 0.0);
    center_.yaw = declare_parameter<double>("center_yaw", 0.0);

    highland_.x = declare_parameter<double>("highland_x", 0.0);
    highland_.y = declare_parameter<double>("highland_y", 0.0);
    highland_.yaw = declare_parameter<double>("highland_yaw", 0.0);

    // 巡逻点用扁平数组：x1,y1,yaw1, x2,y2,yaw2, ...
    // 默认给一个三元组而不是空数组，因为空数组参数在部分 ROS 2 版本上声明会失败。
    const auto flat = declare_parameter<std::vector<double>>(
      "patrol_points", std::vector<double>{0.0, 0.0, 0.0});
    if (flat.size() % 3 != 0) {
      RCLCPP_ERROR(
        get_logger(),
        "patrol_points 长度 %zu 不是 3 的整数倍，格式必须是 x,y,yaw 三元组；本参数被忽略",
        flat.size());
    } else {
      for (size_t i = 0; i + 2 < flat.size(); i += 3) {
        patrol_.push_back(Waypoint{flat[i], flat[i + 1], flat[i + 2]});
      }
    }

    // ---------- 订阅：只订阅任务层真正会用到的五个话题 ----------
    game_status_sub_ = create_subscription<pb_rm_interfaces::msg::GameStatus>(
      "referee/game_status", rclcpp::QoS(10),
      [this](pb_rm_interfaces::msg::GameStatus::SharedPtr msg) {
        game_status_ = *msg;
        has_game_status_ = true;
      });

    robot_status_sub_ = create_subscription<pb_rm_interfaces::msg::RobotStatus>(
      "referee/robot_status", rclcpp::QoS(10),
      [this](pb_rm_interfaces::msg::RobotStatus::SharedPtr msg) {
        robot_status_ = *msg;
        has_robot_status_ = true;
      });

    buff_sub_ = create_subscription<pb_rm_interfaces::msg::Buff>(
      "referee/buff", rclcpp::QoS(10),
      [this](pb_rm_interfaces::msg::Buff::SharedPtr msg) { buff_ = *msg; });

    rfid_sub_ = create_subscription<pb_rm_interfaces::msg::RfidStatus>(
      "referee/rfid_status", rclcpp::QoS(10),
      [this](pb_rm_interfaces::msg::RfidStatus::SharedPtr msg) {
        rfid_ = *msg;
        has_rfid_ = true;
      });

    event_sub_ = create_subscription<pb_rm_interfaces::msg::EventData>(
      "referee/event_data", rclcpp::QoS(10),
      [this](pb_rm_interfaces::msg::EventData::SharedPtr msg) { event_ = *msg; });

    // ---------- Nav2 action client ----------
    if (!dry_run_) {
      nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, navigate_action_);
    } else {
      RCLCPP_WARN(get_logger(), "dry_run = true：只打印决定，不会真的发 NavigateToPose");
    }

    // ---------- 决策定时器 ----------
    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, rate_hz));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { tick(); });
  }

private:
  // 每拍算一次：现在该干什么，和上一次不同才动作。
  //
  // first_tick_ 这一位是必须的，不是保险。state_ 的初值就是 WAIT，
  // 如果第一拍算出来也是 WAIT，两者相等、dispatch() 永远不会被调用，
  // 于是 Nav2 里可能残留的旧目标不会被取消 —— 任务层以为自己在待命，
  // 车却在执行一个没人认领的目标。这一行保证启动后第一拍一定执行一次动作。
  void tick()
  {
    const TaskState next = decide();
    if (next == state_ && !first_tick_) {
      return;
    }
    first_tick_ = false;
    RCLCPP_INFO(
      get_logger(), "任务状态 %s -> %s（血量 %u/%u，剩余 %d 秒）",
      stateName(state_), stateName(next),
      static_cast<unsigned>(robot_status_.current_hp),
      static_cast<unsigned>(robot_status_.maximum_hp),
      static_cast<int>(game_status_.stage_remain_time));
    state_ = next;
    dispatch(next);
  }

  // 决策树。顺序即优先级，改顺序就是改战术。
  TaskState decide()
  {
    // 1. 没有比赛状态 = 裁判链路没通，什么都不做（这是保护，不是保守）
    if (!has_game_status_) {
      return TaskState::WAIT;
    }
    if (game_status_.game_progress != pb_rm_interfaces::msg::GameStatus::RUNNING) {
      return TaskState::WAIT;
    }

    // 2. 血量。没有血量数据时按满血处理，不做低血动作。
    const double hp_ratio = hpRatio();

    if (hp_ratio < low_hp_ratio_) {
      low_hp_latched_ = true;
    }
    if (low_hp_latched_ && hp_ratio >= resume_hp_ratio_) {
      low_hp_latched_ = false;
    }
    if (low_hp_latched_) {
      return TaskState::GO_HOME;
    }

    // 3. 抢中心增益点：前提是本栈真的收到了 RFID，且当前不在点上。
    if (enable_center_gain_ && has_rfid_ && !rfid_.center_gain_point) {
      return TaskState::GO_CENTER;
    }

    // 4. 高地
    if (enable_highland_ && has_rfid_ && !rfid_.central_highland_gain_point) {
      return TaskState::GO_HIGHLAND;
    }

    // 5. 巡逻
    if (enable_patrol_ && !patrol_.empty()) {
      return TaskState::PATROL;
    }

    // 6. 兜底：守在 home
    return TaskState::HOLD;
  }

  double hpRatio() const
  {
    if (!has_robot_status_ || robot_status_.maximum_hp == 0) {
      return 1.0;
    }
    return static_cast<double>(robot_status_.current_hp) /
           static_cast<double>(robot_status_.maximum_hp);
  }

  Waypoint targetOf(TaskState s)
  {
    switch (s) {
      case TaskState::GO_HOME: return home_;
      case TaskState::GO_CENTER: return center_;
      case TaskState::GO_HIGHLAND: return highland_;
      case TaskState::PATROL:
        patrol_index_ = (patrol_index_ + 1) % patrol_.size();
        return patrol_[patrol_index_];
      default: return home_;
    }
  }

  void dispatch(TaskState s)
  {
    if (s == TaskState::WAIT || s == TaskState::HOLD) {
      stopMotion();
      return;
    }

    const Waypoint wp = targetOf(s);

    if (dry_run_) {
      RCLCPP_INFO(get_logger(), "[dry_run] 目标 %.3f, %.3f, yaw %.3f", wp.x, wp.y, wp.yaw);
      return;
    }

    if (!nav_client_) {
      return;
    }
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Nav2 action 服务 %s 不在，目标未发出", navigate_action_.c_str());
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose = makePose(wp);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
    opts.goal_response_callback = [this](const GoalHandleNav::SharedPtr & handle) {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "目标被 Nav2 拒绝（goal_response 为空）");
        }
      };
    opts.result_callback = [this, wp](const GoalHandleNav::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(get_logger(), "到达 %.3f, %.3f", wp.x, wp.y);
        } else {
          RCLCPP_WARN(
            get_logger(), "目标 %.3f, %.3f 未到达，code=%d",
            wp.x, wp.y, static_cast<int>(result.code));
        }
      };

    // 新目标会抢占 Nav2 里正在执行的老目标，所以这里不需要先取消。
    nav_client_->async_send_goal(goal, opts);
    RCLCPP_INFO(get_logger(), "发出目标 %.3f, %.3f, yaw %.3f", wp.x, wp.y, wp.yaw);
  }

  void stopMotion()
  {
    if (dry_run_ || !nav_client_) {
      return;
    }
    RCLCPP_INFO(get_logger(), "取消 Nav2 当前所有目标");
    nav_client_->async_cancel_all_goals();
  }

  static geometry_msgs::msg::PoseStamped makePose(const Waypoint & wp)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = wp.x;
    pose.pose.position.y = wp.y;
    pose.pose.position.z = 0.0;
    // 只绕 z 轴转，避免为一次四元数换算引入 tf2 依赖
    const double half = wp.yaw * 0.5;
    pose.pose.orientation.z = std::sin(half);
    pose.pose.orientation.w = std::cos(half);
    return pose;
  }

  // ---------- 参数 ----------
  bool dry_run_{true};
  std::string navigate_action_{"navigate_to_pose"};
  double low_hp_ratio_{0.50};
  double resume_hp_ratio_{0.90};
  bool enable_center_gain_{true};
  bool enable_highland_{false};
  bool enable_patrol_{false};
  Waypoint home_, center_, highland_;
  std::vector<Waypoint> patrol_;

  // ---------- 运行态 ----------
  TaskState state_{TaskState::WAIT};
  bool first_tick_{true};
  bool low_hp_latched_{false};
  size_t patrol_index_{0};

  bool has_game_status_{false};
  bool has_robot_status_{false};
  bool has_rfid_{false};
  pb_rm_interfaces::msg::GameStatus game_status_{};
  pb_rm_interfaces::msg::RobotStatus robot_status_{};
  pb_rm_interfaces::msg::Buff buff_{};
  pb_rm_interfaces::msg::RfidStatus rfid_{};
  pb_rm_interfaces::msg::EventData event_{};

  // ---------- ROS 句柄 ----------
  rclcpp::Subscription<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::Buff>::SharedPtr buff_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::RfidStatus>::SharedPtr rfid_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::EventData>::SharedPtr event_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
};

}  // namespace rmul2027

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rmul2027::SentryTaskNode>());
  rclcpp::shutdown();
  return 0;
}
