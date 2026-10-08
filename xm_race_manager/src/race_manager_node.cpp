// 厦大省赛·任务层 stage3（六幕状态机 + 航点序列 + 真导航）
// ============================================================
// 相比 stage2 的改动：去程不再是"一发直达射击区"，而是
// W1→W6 逐点推进，到一点再发下一点。
//
// 为什么改：赛道4 上通道是梳齿迷宫（收尾墙/竖井/齿A/B/C/
// 中央结构），中间必须绕行。两点直达时 Nav2 自己找路虽然
// 大概率也对，但逐点钉住路线 = 每一段都可控，赛道上少一个
// 变量；哪个点没到、卡在哪，日志一眼可见。
//
// 前提：Nav2 已经在跑（rm_navigation_reality_launch.py 起过）。
// 本节点不启动 Nav2，只使唤它——大脑不兼司机。
//
// 航点户口：x/y 现在是占位值（0.0）。D1-3 用 RViz 在真实地图上
// 标出 6 个点后，把数填进 GO_WAYPOINTS 表，逻辑不动。
// 表里注释的"图纸参考"是赛道4图纸的毫米坐标（Y向南），只用来
// 对位置含义，不能直接填——建图坐标系和图纸坐标系不是一回事。
// ============================================================

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// ---- 六幕（与v1/v2一致） ----
enum class RaceState : uint8_t
{
  IDLE, GOTO_SHOOT, SHOOT, WAIT_DONE, RETURN, STOP
};

class RaceManager : public rclcpp::Node
{
public:
  RaceManager()
  : Node("race_manager_node")
  {
    // 幕1的入口：开始信号
    start_sub_ = create_subscription<std_msgs::msg::Empty>(
      "race/start", 10,
      [this](std_msgs::msg::Empty::SharedPtr) {
        if (state_ == RaceState::IDLE) {
          transition(RaceState::GOTO_SHOOT, "收到开始信号");
          wp_idx_ = 0;
          send_next_go();
        }
      });

    // 状态广播：外面想看进度就订阅这个
    state_pub_ = create_publisher<std_msgs::msg::String>("race/state", 10);

    // Nav2 的客户端。launch 的 namespace 默认为空 -> action 就叫 /nav2/navigate_to_pose
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "nav2/navigate_to_pose");

    // 假导航开关：true=没Nav2也能演完整剧（每点5秒）；false=必须真Nav2
    // VM 演剧 true，实车 false（launch参数传）
    use_fake_nav_ = declare_parameter<bool>("use_fake_nav", true);

    RCLCPP_INFO(get_logger(),
      "race_manager stage3 就绪（假导航=%s，去程%zu个航点）。开始信号: "
      "ros2 topic pub --once /race/start std_msgs/msg/Empty",
      use_fake_nav_ ? "开" : "关", go_wp_.size());
  }

private:
  // ---- 去程航点表：D1-3 标点后只改数字，顺序别动 ----
  // 拓扑来自 10-08 标注版图纸蓝线（见 赛道4_实际路线图.png v2）
  struct Waypoint
  {
    double x;
    double y;
    const char * note;
  };
  std::vector<Waypoint> go_wp_ = {
    {0.0, 0.0, "W1 出袋东行(收尾墙南侧)"},   // 图纸参考(4600,1500)
    {0.0, 0.0, "W2 竖井西侧北上"},           // (4833,900)
    {0.0, 0.0, "W3 绕齿C南下"},              // (6034,1700)
    {0.0, 0.0, "W4 齿B东侧回通道"},          // (7362,650)
    {0.0, 0.0, "W5 齿A西侧南下"},            // (10362,1700)
    {0.0, 0.0, "W6 射击区中心"},             // (11600,1749)
  };
  // 回程：一发直达启动区，原路让 Nav2 自己规划
  const double home_x_ = 0.0;
  const double home_y_ = 0.0;
  size_t wp_idx_ = 0;

  // ---- 去程：按表推进 ----
  void send_next_go()
  {
    const Waypoint & wp = go_wp_[wp_idx_];
    RCLCPP_INFO(get_logger(), "去程航点 [%zu/%zu] %s",
      wp_idx_ + 1, go_wp_.size(), wp.note);
    send_goal(wp.x, wp.y, false);
  }

  // ---- 给 Nav2 发目标（is_home=false 去程序点 / true 回启动区） ----
  void send_goal(double x, double y, bool is_home)
  {
    if (use_fake_nav_) {
      // 假导航：5s后当作"到了"，走同一条推进路——演全剧/验证推进逻辑用
      RCLCPP_INFO(get_logger(), "[假导航] 5秒后到达 x=%.1f y=%.1f (%s)",
        x, y, is_home ? "回启动区" : go_wp_[wp_idx_].note);
      fake_timer_ = create_wall_timer(5s, [this, is_home]() {
        fake_timer_->cancel();
        if (is_home) {
          if (state_ != RaceState::RETURN) return;
          transition(RaceState::STOP, "假到达启动区");
          RCLCPP_INFO(get_logger(), "[STOP] 停稳保持，等待裁判确认");
        } else {
          if (state_ != RaceState::GOTO_SHOOT) return;
          on_arrived_go();
        }
      });
      return;
    }

    if (!nav_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(),
        "Nav2 action server 5秒没应答——先起 Nav2（rm_navigation_reality_launch），再跑本节点");
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.w = 1.0;  // 朝向不控（yaw 6.28 的账），w=1 即"无旋转"

    auto send_goal_opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_opts.result_callback =
      [this, is_home](const GoalHandleNavigateToPose::WrappedResult & result) {
        if (is_home) {
          if (state_ != RaceState::RETURN) return;
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            transition(RaceState::STOP, "Nav2: 已回启动区");
            RCLCPP_INFO(get_logger(), "[STOP] 停稳保持，等待裁判确认");
          } else {
            RCLCPP_ERROR(get_logger(), "回程 Nav2 报失败（code=%d），卡住——人工介入",
              static_cast<int>(result.code));
          }
          return;
        }
        if (state_ != RaceState::GOTO_SHOOT) return;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          on_arrived_go();
        } else {
          RCLCPP_ERROR(get_logger(), "去程航点[%zu] Nav2 报失败（code=%d），卡住——人工介入",
            wp_idx_ + 1, static_cast<int>(result.code));
        }
      };
    nav_client_->async_send_goal(goal, send_goal_opts);
  }

  // ---- 去程到点：还有下一点就发，没点了换幕射击 ----
  void on_arrived_go()
  {
    wp_idx_++;
    if (wp_idx_ < go_wp_.size()) {
      send_next_go();
    } else {
      transition(RaceState::SHOOT, "全部航点走完，已到射击区");
      do_shoot_stub();
    }
  }

  // ---- 桩B：射击 ----
  void do_shoot_stub()
  {
    RCLCPP_INFO(get_logger(), "[桩B] 射击指令接口待电控——现在用3秒假指令代替");
    // TODO(电控): 射击信号怎么发（串口捎带 vs 单独话题）——打靶链问题
    transition(RaceState::WAIT_DONE, "射击指令发完");
    RCLCPP_INFO(get_logger(), "[桩C] 打完判定无源，20s 计时兜底中……");
    wait_timer_ = create_wall_timer(20s, [this]() {
      wait_timer_->cancel();
      if (state_ == RaceState::WAIT_DONE) {
        transition(RaceState::RETURN, "兜底超时，回家");
        send_goal(home_x_, home_y_, true);
      }
    });
  }

  void transition(RaceState next, const std::string & why)
  {
    state_ = next;
    RCLCPP_INFO(get_logger(), ">> 换幕 -> %s（%s）", name(next).c_str(), why.c_str());
    publish_state(name(next));
  }

  static std::string name(RaceState s)
  {
    switch (s) {
      case RaceState::IDLE: return "IDLE";
      case RaceState::GOTO_SHOOT: return "GOTO_SHOOT";
      case RaceState::SHOOT: return "SHOOT";
      case RaceState::WAIT_DONE: return "WAIT_DONE";
      case RaceState::RETURN: return "RETURN";
      case RaceState::STOP: return "STOP";
    }
    return "?";
  }

  void publish_state(const std::string & s)
  {
    std_msgs::msg::String m;
    m.data = s;
    state_pub_->publish(m);
  }

  RaceState state_ = RaceState::IDLE;
  bool use_fake_nav_ = true;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr wait_timer_;
  rclcpp::TimerBase::SharedPtr fake_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RaceManager>());
  rclcpp::shutdown();
  return 0;
}
