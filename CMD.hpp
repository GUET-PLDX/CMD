#pragma once

/* clang-format off */
/* === MODULE MANIFEST V2 ===
module_description: 控制命令模块
depends: []
=== END MANIFEST === */
/* clang-format on */

/**
 * @file CMD.hpp
 * @brief 控制命令处理模块
 * @details 负责处理来自不同控制源的命令，并将其转发到相应的执行单元
 */

#include <array>
#include <cmath>

#include "event.hpp"
#include "libxr_def.hpp"
#include "message.hpp"

/**
 * @class CMD
 * @brief 控制命令处理类
 * @details 负责处理来自不同控制源的命令，并将其转发到相应的执行单元
 */
class CMD {
 public:
  /**
   * @brief 控制源枚举
   */
  enum class ControlSource : uint8_t {
    CTRL_SOURCE_RC,  /* 遥控器控制源 */
    CTRL_SOURCE_AI,  ///< 上位机 / 自动控制 Host / automatic control
    CTRL_SOURCE_NUM  /* 控制源数量 */
  };

  /**
   * @brief 遥控链路输入源枚举
   */
  enum class RCInputSource : uint8_t {
    RC_INPUT_DR16,
    RC_INPUT_VT13,
    RC_INPUT_REFEREE,
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
    NONE = 0,  /*无模式*/
    BOOST = 1, /*加速*/
  };

  /**
   * @brief 底盘复合命令：运动目标与动力学前馈（N、N、N·m）。
   */
  typedef struct ChassisCMD {
    float vx = 0.0F;
    float vy = 0.0F;
    float wz = 0.0F;
    float fx_n = 0.0F;
    float fy_n = 0.0F;
    float mz_nm = 0.0F;
    bool ff_valid = false;
    ChasStat self_define = ChasStat::NONE;
  } ChassisCMD;

  /**
   * @brief 云台控制命令结构体
   * @details 遥控 yaw/pit 为归一化输入，由云台按控制周期积分；
   *          AI yaw/pit 为绝对角，导数使用 rad/s 和 rad/s^2。
   */
  typedef struct {
    float yaw;      /* 遥控归一化输入 / AI 偏航角 rad */
    float pit;      /* 遥控归一化输入 / AI 俯仰角 rad */
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
  Mode GetCtrlMode() const { return this->mode_; }

  bool GetAIGimbalStatus() const {
    return this->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_AI)]
        .gimbal_online;
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
  bool Online() const { return this->online_; }

  /**
   * @brief 写入遥控器控制数据并同步发布
   * @details 兼容接口，默认按DR16输入源写入
   */
  void FeedRC(const Data& rc_data) {
    this->FeedRC(RCInputSource::RC_INPUT_DR16, rc_data);
  }

  /**
   * @brief 按遥控输入源写入控制数据并同步发布
   */
  void FeedRC(RCInputSource source, const Data& rc_data) {
    const auto source_index = static_cast<size_t>(source);
    if (source_index >= static_cast<size_t>(RCInputSource::RC_INPUT_NUM)) {
      return;
    }

    this->rc_input_data_[source_index] = rc_data;
    this->rc_input_seq_[source_index] = ++this->rc_update_seq_;
    if (rc_data.chassis_online && IsRCInputActive(rc_data)) {
      this->active_rc_input_ = source;
    }
    ProcessAndPublish();
  }

  /** 分片更新 AI 底盘命令，供导航等独立来源与视觉来源合并提交。 */
  void FeedAIChassis(const Data& ai_data) {
    auto& slot =
        this->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_AI)];
    slot.chassis = ai_data.chassis;
    slot.chassis_online = ai_data.chassis_online;
    ProcessAndPublish();
  }

  /** 分片更新 AI 云台与发射命令，供视觉来源与导航来源合并提交。 */
  void FeedAIGimbal(const Data& ai_data) {
    auto& slot =
        this->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_AI)];
    slot.gimbal = ai_data.gimbal;
    slot.gimbal_online = ai_data.gimbal_online;
    slot.launcher = ai_data.launcher;
    ProcessAndPublish();
  }

  struct ChassisControl {
    ChassisCMD command{};
    Mode mode = Mode::CMD_OP_CTRL;
    bool rc_online = false;
    bool ai_chassis_online = false;
  };

  /**
   * @brief CMD构造函数
   * @param mode 控制模式，默认为操作员控制模式
   * @param chassis_cmd_topic_name Chassis command topic name.
   * @param gimbal_cmd_topic_name Gimbal command topic name.
   * @param launcher_cmd_topic_name Launcher command topic name.
   * @param control_state_topic_name Coherent chassis command and mode topic.
   */
  CMD(Mode mode = CMD::Mode::CMD_OP_CTRL,
      const char* chassis_cmd_topic_name = "chassis_cmd",
      const char* gimbal_cmd_topic_name = "gimbal_cmd",
      const char* launcher_cmd_topic_name = "launcher_cmd",
      const char* control_state_topic_name = "control_state")
      : mode_(mode),
        chassis_data_tp_(LibXR::Topic::CreateTopic<ChassisCMD>(
            chassis_cmd_topic_name, nullptr, true)),
        gimbal_data_tp_(LibXR::Topic::CreateTopic<GimbalCMD>(
            gimbal_cmd_topic_name, nullptr, true)),
        fire_data_tp_(LibXR::Topic::CreateTopic<LauncherCMD>(
            launcher_cmd_topic_name, nullptr, true)),
        control_state_tp_(LibXR::Topic::CreateTopic<ChassisControl>(
            control_state_topic_name, nullptr, true)) {
    /* 创建事件回调函数 */
    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, CMD* cmd, uint32_t event_id) {
          UNUSED(in_isr);
          cmd->EventHandler(event_id);
        },
        this);
    /* 注册控制模式事件处理回调 */
    this->cmd_event_.Register(static_cast<uint32_t>(Mode::CMD_OP_CTRL),
                              callback);
    this->cmd_event_.Register(static_cast<uint32_t>(Mode::CMD_AUTO_CTRL),
                              callback);
  }

  /**
   * @brief 设置控制模式
   * @param mode 要设置的控制模式
   * @details 有效模式变化时立即更新状态、重新仲裁并发布
   */
  void SetCtrlMode(Mode mode) {
    if (mode != Mode::CMD_OP_CTRL && mode != Mode::CMD_AUTO_CTRL) return;
    if (this->mode_ == mode) return;
    this->mode_ = mode;
    ProcessAndPublish();
  }

  /**
   * @brief 事件处理器
   * @param event_id 事件ID
   * @details 处理来自事件系统的控制模式切换请求
   */
  void EventHandler(uint32_t event_id) {
    if (event_id > static_cast<uint32_t>(Mode::CMD_AUTO_CTRL)) return;
    this->SetCtrlMode(static_cast<Mode>(event_id));
  }

 private:
  bool online_ = false;
  Mode mode_;
  LibXR::Event cmd_event_; /* 事件处理器 */
  std::array<Data, static_cast<size_t>(ControlSource::CTRL_SOURCE_NUM)>
      data_{}; /* 各控制源的数据 */
  std::array<Data, static_cast<size_t>(RCInputSource::RC_INPUT_NUM)>
      rc_input_data_{}; /* 各遥控输入源的数据 */
  std::array<uint32_t, static_cast<size_t>(RCInputSource::RC_INPUT_NUM)>
      rc_input_seq_{};           /* 各遥控输入源的数据序号 */
  LibXR::Topic chassis_data_tp_; /* 底盘命令主题 */
  LibXR::Topic gimbal_data_tp_;  /* 云台命令主题 */
  LibXR::Topic fire_data_tp_;    /* 开火命令主题 */
  LibXR::Topic control_state_tp_;
  RCInputSource active_rc_input_ =
      RCInputSource::RC_INPUT_DR16; /* 当前活动遥控输入源 */
  uint32_t rc_update_seq_ = 0;      /* 遥控输入数据更新序号 */
  /*--------------------------工具函数-------------------------------------------------*/
  static bool IsRCInputOnline(const Data& rc_data) {
    return rc_data.chassis_online;
  }

  static bool IsRCInputActive(const Data& rc_data) {
    constexpr float RC_ACTIVITY_EPS = 0.05f;

    return std::fabs(rc_data.chassis.vx) > RC_ACTIVITY_EPS ||
           std::fabs(rc_data.chassis.vy) > RC_ACTIVITY_EPS ||
           std::fabs(rc_data.chassis.wz) > RC_ACTIVITY_EPS ||
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

    // 当前活动源在线则持续使用
    if (active_index < static_cast<size_t>(RCInputSource::RC_INPUT_NUM) &&
        this->IsRCInputOnline(this->rc_input_data_[active_index])) {
      return this->rc_input_data_[active_index];
    }

    // 活动源离线后切换
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

  void ProcessAndPublish() {
    const Data rc_data = this->SelectRCData();
    const Data& ai_data =
        this->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_AI)];
    ChassisCMD out_chassis{};
    GimbalCMD out_gimbal{};
    LauncherCMD out_launcher{};

    this->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_RC)] = rc_data;

    if (!rc_data.chassis_online && this->online_) {
      this->online_ = false;
      this->cmd_event_.Active(CMD_EVENT_LOST_CTRL);
    } else if (rc_data.chassis_online && !this->online_) {
      this->online_ = true;
      this->cmd_event_.Active(CMD_EVENT_START_CTRL);
    }

    if (this->mode_ == Mode::CMD_OP_CTRL) {
      out_chassis = rc_data.chassis;
      out_chassis.fx_n = out_chassis.fy_n = out_chassis.mz_nm = 0.0F;
      out_chassis.ff_valid = false;
      out_launcher = rc_data.launcher;
      out_gimbal = rc_data.gimbal;
    } else {
      /* CMD_AUTO_CTRL */
      out_gimbal = ai_data.gimbal_online ? ai_data.gimbal : rc_data.gimbal;
      if (ai_data.chassis_online) {
        out_chassis = ai_data.chassis;
      } else {
        out_chassis = rc_data.chassis;
        out_chassis.fx_n = out_chassis.fy_n = out_chassis.mz_nm = 0.0F;
        out_chassis.ff_valid = false;
      }
      // Vision and navigation have independent lifetimes. In automatic mode,
      // firing requires a live vision target plus both fire requests.
      out_launcher.isfire = ai_data.gimbal_online && ai_data.launcher.isfire &&
                            rc_data.launcher.isfire;
    }
    this->gimbal_data_tp_.Publish(out_gimbal);
    this->chassis_data_tp_.Publish(out_chassis);
    this->fire_data_tp_.Publish(out_launcher);
    ChassisControl control{out_chassis, mode_, online_, ai_data.chassis_online};
    control_state_tp_.Publish(control);
  }
};
