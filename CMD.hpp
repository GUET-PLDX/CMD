#pragma once

/* clang-format off */
/* === MODULE MANIFEST V2 ===
module_description: 控制命令模块
constructor_args:
  - mode: CMD::Mode::CMD_OP_CTRL
  - chassis_cmd_topic_name: "chassis_cmd"
  - gimbal_cmd_topic_name: "gimbal_cmd"
  - launcher_cmd_topic_name: "launcher_cmd"
  - dispatch_task_stack_depth: 1024
  - dispatch_thread_priority: LibXR::Thread::Priority::HIGH
=== END MANIFEST === */
/* clang-format on */

/**
 * @file CMD.hpp
 * @brief 控制命令处理模块
 * @details 负责处理来自不同控制源的命令，并将其转发到相应的执行单元
 */

#include <array>
#include <atomic>
#include <cmath>

#include "app_framework.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "mpmc_queue.hpp"
#include "mutex.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timebase.hpp"

/**
 * @class CMD
 * @brief 控制命令处理类
 * @details 接收来自不同控制源的命令，处理并转发到底盘和云台等执行单元
 */
class CMD : public LibXR::Application {
 public:
  /**
   * @brief 控制源枚举
   */
  enum class ControlSource : uint8_t {
    CTRL_SOURCE_RC, /* 遥控器控制源 */
    CTRL_SOURCE_AI, /* AI控制源 */
    CTRL_SOURCE_NUM /* 控制源数量 */
  };

  /**
   * @brief 遥控链路输入源枚举
   */
  enum class RCInputSource : uint8_t {
    RC_INPUT_DR16,
    RC_INPUT_VT13,
    RC_INPUT_NUM
  };

  /**
   * @brief 控制模式枚举
   */
  enum class Mode : uint8_t {
    CMD_OP_CTRL,   /* 操作员控制模式 */
    CMD_AUTO_CTRL, /* 自动控制模式 */
  };

  /**
   * @brief 底盘小模式枚举
   */
  enum class ChasStat : int8_t {
    NONE = 0,    /*无模式*/
    BOOST = 1,   /*加速*/
    STRETCH = 2, /*伸腿*/
  };

  enum class ChassisCommandSource : uint8_t {
    OPERATOR = 0,
    NAVIGATION = 1,
  };

  struct OperatorChassisInput {
    float x;
    float y;
    float z;
  };

  struct NavigationVelocity {
    float vx_mps;
    float vy_mps;
    float wz_rad_s;
  };

  /**
   * @brief 底盘控制命令结构体
   */
  typedef struct {
    ChassisCommandSource source;
    OperatorChassisInput operator_input;
    NavigationVelocity navigation_velocity;
    ChasStat self_define; /* 自定义按钮 */
  } ChassisCMD;

  /**
   * @brief 云台控制命令结构体
   */
  typedef struct {
    float yaw;      /* 偏航角设定值，单位 rad */
    float pit;      /* 俯仰角设定值，单位 rad */
    float rol;      /* 翻滚角设定值，单位 rad */
    float yaw_dot;  /* yaw 角速度，单位 rad/s */
    float yaw_ddot; /* yaw 角加速度，单位 rad/s^2 */
    float pit_dot;  /* pit 角速度，单位 rad/s */
    float pit_ddot; /* pit 角加速度，单位 rad/s^2 */
    float rol_dot;  /* roll 角速度，单位 rad/s */
    float rol_ddot; /* roll 角加速度，单位 rad/s^2 */
  } GimbalCMD;

  /**
   * @brief 发射控制命令结构体
   */
  typedef struct {
    bool isfire;
  } LauncherCMD;

  /**
   * @brief 完整控制命令数据结构体
   */
  typedef struct {
    GimbalCMD gimbal;          /* 云台控制命令 */
    ChassisCMD chassis;        /* 底盘控制命令 */
    LauncherCMD launcher;      /* 发射控制命令 */
    bool chassis_online;       /* 底盘在线状态 */
    bool gimbal_online;        /* 云台在线状态 */
    ControlSource ctrl_source; /* 控制源 */
  } Data;

  /**
   * @brief 控制事件ID
   */
  enum {
    CMD_EVENT_START_CTRL = 0x13212508, /* 开始控制事件ID */
    CMD_EVENT_LOST_CTRL = 0x13212509   /* 丢失控制事件ID */
  };

  /**
   * @brief 获取当前控制模式
   * @return 当前控制模式
   */
  Mode GetCtrlMode() {
    LibXR::Mutex::LockGuard lock(mutex_);
    return this->mode_;
  }

  bool GetAIGimbalStatus() {
    LibXR::Mutex::LockGuard lock(mutex_);
    return this->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_AI)]
        .gimbal_online;
  }

  void SetGimbalSetpoint(float yaw_rad, float pit_rad) {
    LibXR::Mutex::LockGuard lock(mutex_);
    this->gimbal_yaw_rad_ = yaw_rad;
    this->gimbal_pit_rad_ = pit_rad;
  }

  /**
   * @brief 获取CMD模块的事件处理器
   * @return 事件处理器引用
   * @details 用于其他模块绑定事件或激活事件
   */
  LibXR::Event& GetEvent() { return cmd_event_; }

  /**
   * @brief 获取在线状态
   * @return 是否在线
   */
  bool Online() {
    LibXR::Mutex::LockGuard lock(mutex_);
    return this->online_;
  }

  /**
   * @brief 直接写入遥控器控制数据
   * @details 兼容接口，默认按DR16输入源写入
   */
  void FeedRC(const Data& rc_data) {
    this->FeedRC(RCInputSource::RC_INPUT_DR16, rc_data);
  }

  /**
   * @brief 按遥控输入源写入控制数据
   */
  void FeedRC(RCInputSource source, const Data& rc_data) {
    const auto source_index = static_cast<size_t>(source);
    if (source_index >= static_cast<size_t>(RCInputSource::RC_INPUT_NUM)) {
      return;
    }

    DispatchSnapshot snapshot;
    {
      LibXR::Mutex::LockGuard lock(mutex_);
      this->rc_input_data_[source_index] = rc_data;
      this->rc_input_seq_[source_index] = ++this->rc_update_seq_;

      if (rc_data.chassis_online && this->IsRCInputActive(rc_data)) {
        this->active_rc_input_ = source;
      }

      snapshot = BuildDispatchLocked();
      EnqueueDispatchLocked(snapshot);
    }
    this->dispatch_ready_.Post();
  }

  /**
   * @brief 直接写入 AI 控制数据
   */
  void FeedAI(const Data& ai_data) {
    DispatchSnapshot snapshot;
    {
      LibXR::Mutex::LockGuard lock(mutex_);
      this->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_AI)] = ai_data;
      snapshot = BuildDispatchLocked();
      EnqueueDispatchLocked(snapshot);
    }
    this->dispatch_ready_.Post();
  }

  /**
   * @brief CMD构造函数
   * @param hw 硬件容器引用
   * @param app 应用管理器引用
   * @param mode 控制模式，默认为操作员控制模式
   * @param chassis_cmd_topic_name Chassis command topic name.
   * @param gimbal_cmd_topic_name Gimbal command topic name.
   * @param launcher_cmd_topic_name Launcher command topic name.
   * @param dispatch_task_stack_depth Dispatcher stack size in bytes.
   * @param dispatch_thread_priority Dispatcher thread priority.
   */
  CMD(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, Mode mode,
      const char* chassis_cmd_topic_name, const char* gimbal_cmd_topic_name,
      const char* launcher_cmd_topic_name, uint32_t dispatch_task_stack_depth,
      LibXR::Thread::Priority dispatch_thread_priority)
      : mode_(mode),
        chassis_data_tp_(LibXR::Topic::CreateTopic<ChassisCMD>(
            chassis_cmd_topic_name, nullptr, true)),
        gimbal_data_tp_(LibXR::Topic::CreateTopic<GimbalCMD>(
            gimbal_cmd_topic_name, nullptr, true)),
        fire_data_tp_(LibXR::Topic::CreateTopic<LauncherCMD>(
            launcher_cmd_topic_name, nullptr, true)) {
    UNUSED(hw);
    UNUSED(app);
    /* 创建事件回调函数 */
    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, CMD* cmd, uint32_t event_id) {
          UNUSED(in_isr);
          cmd->RequestCtrlMode(static_cast<Mode>(event_id));
        },
        this);
    /* 注册控制模式事件处理回调 */
    this->cmd_event_.Register(static_cast<uint32_t>(Mode::CMD_OP_CTRL),
                              callback);
    this->cmd_event_.Register(static_cast<uint32_t>(Mode::CMD_AUTO_CTRL),
                              callback);
    this->dispatch_thread_.Create(this, DispatchTask, "CMDDispatch",
                                  dispatch_task_stack_depth,
                                  dispatch_thread_priority);
  }

  /**
   * @brief 设置控制模式
   * @param mode 要设置的控制模式
   * @details 根据不同的控制模式配置相应的数据处理回调函数
   */
  void SetCtrlMode(Mode mode) {
    LibXR::Mutex::LockGuard lock(mutex_);
    this->applied_mode_request_state_ =
        this->mode_request_state_.load(std::memory_order_acquire);
    this->mode_ = mode;
  }

  void RequestCtrlMode(Mode mode) {
    const uint32_t MODE_VALUE = static_cast<uint32_t>(mode) & MODE_VALUE_MASK;
    uint32_t expected =
        this->mode_request_state_.load(std::memory_order_relaxed);
    uint32_t desired;
    do {
      desired =
          ((expected + MODE_SEQUENCE_STEP) & ~MODE_VALUE_MASK) | MODE_VALUE;
    } while (!this->mode_request_state_.compare_exchange_weak(
        expected, desired, std::memory_order_release,
        std::memory_order_relaxed));
  }

  /**
   * @brief 事件处理器
   * @param event_id 事件ID
   * @details 处理来自事件系统的控制模式切换请求
   */
  void EventHandler(uint32_t event_id) {
    this->RequestCtrlMode(static_cast<Mode>(event_id));
  }

  /**
   * @brief 注册控制器
   * @tparam SourceDataType 源数据类型
   * @param source 源主题
   * @details 将外部控制源的数据接入CMD系统，并进行预处理和分发
   */
  template <typename SourceDataType>
  void RegisterController(LibXR::Topic& source) {
    UNUSED(source);
  }

  /**
   * @brief 监控函数重写
   */
  void OnMonitor() override {}

  uint32_t GetDispatchOverflowCount() {
    LibXR::Mutex::LockGuard lock(mutex_);
    return this->dispatch_overflow_count_;
  }

 private:
  struct DispatchSnapshot {
    ChassisCMD chassis{};
    GimbalCMD gimbal{};
    LauncherCMD launcher{};
    uint64_t sequence = 0U;
    uint32_t event_id = 0U;
    bool has_event = false;
  };

  static_assert(std::atomic<uint32_t>::is_always_lock_free);
  static constexpr size_t DISPATCH_QUEUE_CAPACITY = 16;
  static constexpr uint32_t MODE_VALUE_MASK = 0x1U;
  static constexpr uint32_t MODE_SEQUENCE_STEP = MODE_VALUE_MASK + 1U;

  LibXR::Mutex mutex_;
  LibXR::MPMCQueue<DispatchSnapshot> dispatch_queue_{DISPATCH_QUEUE_CAPACITY};
  LibXR::Semaphore dispatch_ready_;
  LibXR::Thread dispatch_thread_;
  std::atomic<uint32_t> mode_request_state_{0U};
  DispatchSnapshot overflow_snapshot_{};
  bool overflow_pending_ = false;
  bool online_ = false;    /* 在线状态 */
  Mode mode_;              /* 当前控制模式 */
  LibXR::Event cmd_event_; /* 事件处理器 */
  std::array<Data, static_cast<size_t>(ControlSource::CTRL_SOURCE_NUM)>
      data_{}; /* 各控制源的数据 */
  std::array<Data, static_cast<size_t>(RCInputSource::RC_INPUT_NUM)>
      rc_input_data_{}; /* 各遥控输入源的数据 */
  std::array<uint32_t, static_cast<size_t>(RCInputSource::RC_INPUT_NUM)>
      rc_input_seq_{};              /* 各遥控输入源的数据序号 */
  LibXR::Topic chassis_data_tp_;    /* 底盘命令主题 */
  LibXR::Topic gimbal_data_tp_;     /* 云台命令主题 */
  LibXR::Topic fire_data_tp_;       /* 开火命令主题 */
  LibXR::Topic host_euler_data_tp_; /* 上位机欧拉角主题 */
  RCInputSource active_rc_input_ =
      RCInputSource::RC_INPUT_DR16;          /* 当前活动遥控输入源 */
  uint64_t dispatch_sequence_ = 0U;          /* 命令快照序号 */
  uint32_t dispatch_overflow_count_ = 0U;    /* 调度队列溢出计数 */
  uint32_t applied_mode_request_state_ = 0U; /* 已应用模式请求 */
  uint32_t rc_update_seq_ = 0;               /* 遥控输入数据更新序号 */
  float gimbal_yaw_rad_ = 0.0f;
  float gimbal_pit_rad_ = 0.0f;
  LibXR::MicrosecondTimestamp last_gimbal_integrate_time_{};

  static constexpr float GIMBAL_MAX_SPEED =
      static_cast<float>(LibXR::TWO_PI) * 2.0f;
  static constexpr float GIMBAL_INTEGRATE_MAX_DT_S = 0.05f;

  /*--------------------------工具函数-------------------------------------------------*/
  static bool IsRCInputOnline(const Data& rc_data) {
    return rc_data.chassis_online;
  }

  static bool IsRCInputActive(const Data& rc_data) {
    constexpr float RC_ACTIVITY_EPS = 0.05f;

    return std::fabs(rc_data.chassis.operator_input.x) > RC_ACTIVITY_EPS ||
           std::fabs(rc_data.chassis.operator_input.y) > RC_ACTIVITY_EPS ||
           std::fabs(rc_data.chassis.operator_input.z) > RC_ACTIVITY_EPS ||
           std::fabs(rc_data.gimbal.yaw) > RC_ACTIVITY_EPS ||
           std::fabs(rc_data.gimbal.pit) > RC_ACTIVITY_EPS ||
           std::fabs(rc_data.gimbal.rol) > RC_ACTIVITY_EPS ||
           (rc_data.chassis.self_define != ChasStat::NONE) ||
           rc_data.launcher.isfire;
  }

  static Data MakeOfflineRCData() {
    Data rc_data{};
    rc_data.chassis_online = false;
    rc_data.gimbal_online = false;
    rc_data.ctrl_source = ControlSource::CTRL_SOURCE_RC;
    return rc_data;
  }

  Data SelectRCData() {
    const auto dr16_index = static_cast<size_t>(RCInputSource::RC_INPUT_DR16);
    const auto vt13_index = static_cast<size_t>(RCInputSource::RC_INPUT_VT13);
    const auto active_index = static_cast<size_t>(this->active_rc_input_);

    /* 当前活动源在线则持续使用 */
    if (active_index < static_cast<size_t>(RCInputSource::RC_INPUT_NUM) &&
        this->IsRCInputOnline(this->rc_input_data_[active_index])) {
      return this->rc_input_data_[active_index];
    }

    /* 活动源离线后切换 */
    if (this->active_rc_input_ == RCInputSource::RC_INPUT_DR16) {
      if (this->IsRCInputOnline(this->rc_input_data_[vt13_index])) {
        this->active_rc_input_ = RCInputSource::RC_INPUT_VT13;
        return this->rc_input_data_[vt13_index];
      }
      if (this->IsRCInputOnline(this->rc_input_data_[dr16_index])) {
        this->active_rc_input_ = RCInputSource::RC_INPUT_DR16;
        return this->rc_input_data_[dr16_index];
      }
    } else {
      if (this->IsRCInputOnline(this->rc_input_data_[dr16_index])) {
        this->active_rc_input_ = RCInputSource::RC_INPUT_DR16;
        return this->rc_input_data_[dr16_index];
      }
      if (this->IsRCInputOnline(this->rc_input_data_[vt13_index])) {
        this->active_rc_input_ = RCInputSource::RC_INPUT_VT13;
        return this->rc_input_data_[vt13_index];
      }
    }

    return MakeOfflineRCData();
  }

  DispatchSnapshot BuildDispatchLocked() {
    const uint32_t MODE_REQUEST_STATE =
        this->mode_request_state_.load(std::memory_order_acquire);
    if (MODE_REQUEST_STATE != this->applied_mode_request_state_) {
      this->mode_ = static_cast<Mode>(MODE_REQUEST_STATE & MODE_VALUE_MASK);
      this->applied_mode_request_state_ = MODE_REQUEST_STATE;
    }

    const Data rc_data = this->SelectRCData();
    const Data& ai_data =
        this->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_AI)];
    DispatchSnapshot snapshot;
    snapshot.sequence = ++this->dispatch_sequence_;

    this->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_RC)] = rc_data;

    if (!rc_data.chassis_online && this->online_) {
      this->online_ = false;
      snapshot.event_id = CMD_EVENT_LOST_CTRL;
      snapshot.has_event = true;
    } else if (rc_data.chassis_online && !this->online_) {
      this->online_ = true;
      snapshot.event_id = CMD_EVENT_START_CTRL;
      snapshot.has_event = true;
    }

    /* 遥控失联时优先失能全部执行机构，禁止自动控制继续输出 */
    if (!rc_data.chassis_online) {
      snapshot.chassis = {};
      snapshot.chassis.source = ChassisCommandSource::OPERATOR;
      snapshot.launcher.isfire = false;
      snapshot.gimbal.yaw = this->gimbal_yaw_rad_;
      snapshot.gimbal.pit = this->gimbal_pit_rad_;
      return snapshot;
    }

    if (this->mode_ == Mode::CMD_OP_CTRL) {
      snapshot.chassis = rc_data.chassis;
      snapshot.chassis.source = ChassisCommandSource::OPERATOR;
      snapshot.launcher = rc_data.launcher;
    } else {
      /* CMD_AUTO_CTRL */
      if (ai_data.chassis_online) {
        snapshot.chassis = ai_data.chassis;
        snapshot.chassis.source = ChassisCommandSource::NAVIGATION;
        /* 主机在线：射击需主机与遥控同时请求 */
        snapshot.launcher.isfire =
            (ai_data.launcher.isfire && rc_data.launcher.isfire);
      } else {
        snapshot.chassis = rc_data.chassis;
        snapshot.chassis.source = ChassisCommandSource::OPERATOR;
        /* 主机离线：退回遥控射击请求，与底盘的离线回退一致 */
        snapshot.launcher = rc_data.launcher;
      }
    }
    this->ApplyGimbalCommandLocked(snapshot, rc_data, ai_data);

    return snapshot;
  }

  void ApplyGimbalCommandLocked(DispatchSnapshot& snapshot, const Data& rc_data,
                                const Data& ai_data) {
    const auto NOW = LibXR::Timebase::GetMicroseconds();
    float dt_s = 0.0f;
    if (static_cast<uint64_t>(this->last_gimbal_integrate_time_) != 0U) {
      dt_s = (NOW - this->last_gimbal_integrate_time_).ToSecondf();
    }
    this->last_gimbal_integrate_time_ = NOW;
    if (!std::isfinite(dt_s) || dt_s < 0.0f ||
        dt_s > GIMBAL_INTEGRATE_MAX_DT_S) {
      dt_s = 0.0f;
    }

    const bool USE_AI_GIMBAL =
        this->mode_ == Mode::CMD_AUTO_CTRL && ai_data.gimbal_online;
    if (USE_AI_GIMBAL) {
      snapshot.gimbal = ai_data.gimbal;
      this->gimbal_yaw_rad_ = ai_data.gimbal.yaw;
      this->gimbal_pit_rad_ = ai_data.gimbal.pit;
      return;
    }

    this->gimbal_yaw_rad_ += rc_data.gimbal.yaw * GIMBAL_MAX_SPEED * dt_s;
    this->gimbal_pit_rad_ += rc_data.gimbal.pit * GIMBAL_MAX_SPEED * dt_s;
    snapshot.gimbal = rc_data.gimbal;
    snapshot.gimbal.yaw = this->gimbal_yaw_rad_;
    snapshot.gimbal.pit = this->gimbal_pit_rad_;
    snapshot.gimbal.yaw_dot = rc_data.gimbal.yaw * GIMBAL_MAX_SPEED;
    snapshot.gimbal.pit_dot = rc_data.gimbal.pit * GIMBAL_MAX_SPEED;
    snapshot.gimbal.yaw_ddot = 0.0f;
    snapshot.gimbal.pit_ddot = 0.0f;
  }

  void EnqueueDispatchLocked(const DispatchSnapshot& snapshot) {
    if (this->overflow_pending_) {
      this->HandleDispatchOverflowLocked(snapshot);
      return;
    }

    const LibXR::ErrorCode PUSH_RESULT = this->dispatch_queue_.Push(snapshot);
    if (PUSH_RESULT != LibXR::ErrorCode::OK) {
      this->HandleDispatchOverflowLocked(snapshot);
    }
  }

  void HandleDispatchOverflowLocked(const DispatchSnapshot& snapshot) {
    this->overflow_snapshot_ = {};
    this->overflow_snapshot_.sequence = snapshot.sequence;
    this->overflow_snapshot_.event_id = CMD_EVENT_LOST_CTRL;
    this->overflow_snapshot_.has_event = true;
    this->overflow_pending_ = true;
    ++this->dispatch_overflow_count_;
  }

  bool TakeNextDispatch(DispatchSnapshot& snapshot) {
    LibXR::Mutex::LockGuard lock(mutex_);
    if (this->dispatch_queue_.Pop(snapshot) == LibXR::ErrorCode::OK) {
      return true;
    }
    if (!this->overflow_pending_) {
      return false;
    }

    snapshot = this->overflow_snapshot_;
    this->overflow_pending_ = false;
    this->online_ = false;
    return true;
  }

  static void DispatchTask(CMD* cmd) {
    uint64_t last_sequence = 0U;
    while (true) {
      static_cast<void>(cmd->dispatch_ready_.Wait());
      DispatchSnapshot snapshot;
      while (cmd->TakeNextDispatch(snapshot)) {
        ASSERT(snapshot.sequence > last_sequence);
        last_sequence = snapshot.sequence;
        cmd->Publish(snapshot);
      }
    }
  }

  void Publish(const DispatchSnapshot& snapshot) {
    GimbalCMD gimbal = snapshot.gimbal;
    ChassisCMD chassis = snapshot.chassis;
    LauncherCMD launcher = snapshot.launcher;
    this->gimbal_data_tp_.Publish(gimbal);
    this->chassis_data_tp_.Publish(chassis);
    this->fire_data_tp_.Publish(launcher);
    if (snapshot.has_event) {
      this->cmd_event_.Active(snapshot.event_id);
    }
  }
};
