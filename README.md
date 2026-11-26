# CMD

控制命令中枢，保存遥控输入和独立的导航底盘、视觉云台输入，调用 `FeedRC()`、`FeedAIChassis()`、`FeedAIGimbal()` 后同步仲裁和发布。

ET08A 的 `Data::remote` 在同一帧中携带底盘与云台独立权限、RELAX/FOLLOW/ROTOR 和折叠请求。`GetCtrlMode()` / `GetControlSelection()` 表示底盘权限，`GetGimbalCtrlMode()` 表示云台权限；旧 `SetCtrlMode()` 继续同步切换两者。底盘权限变化时递增 revision，供折展命令核对控制权。

- 活动遥控源在线则保留，另一在线源有操作时接管；失联时寻找在线源。支持 DR16、VT13、Referee、ET08A。
- 手动及 AI 底盘离线回退时，清零力前馈并撤销 ff_valid。导航和视觉各自离线时独立回退遥控数据。
- 遥控 RELAX 清零底盘命令，优先于在线导航；全部遥控离线清零全部输出、禁止开火，先更新 Online() 再触发失控事件。
- 自瞄发射要求视觉在线、AI 开火建议与遥控许可同时成立。手动发射使用遥控请求，最终动作由发射机构决定。
- 手动云台为归一化输入，AI 云台为 rad、rad/s、rad/s²；导航速度为 m/s、rad/s，力前馈为 N、N、N·m。

构造参数依次为 mode、chassis_cmd_topic_name、gimbal_cmd_topic_name、launcher_cmd_topic_name、control_state_topic_name。默认 Topic：

| Topic | 类型 | 用途 |
| --- | --- | --- |
| chassis_cmd | ChassisCMD | 底盘运动与力前馈 |
| gimbal_cmd | GimbalCMD | 云台参考 |
| launcher_cmd | LauncherCMD | 开火许可 |
| control_state | ChassisControl | 同次仲裁的底盘命令、权限、在线状态、显式构型与运动请求 |

ET08A 使用 `RegisterRemoteInput()` 注册唯一的遥控交接回调，Gimbal 在每周期入口调用 `UpdateRemoteInput()`。串口线程只在 ET08A 内排队，由云台控制任务提交 CMD 和执行 SC 事件，避免串口线程在控制计算中途重置模式。模块与 CMD 须具有应用全生命周期，初始化后不重新注册回调。该入口不改变其他既有输入模块的执行方式。

主机集成测试位于 BSP 的 tools/tests/et08a 和 tools/tests/gimbal_autoaim，覆盖独立权限、发射许可、失控恢复与真实控制输出。
