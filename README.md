# CMD

控制命令中枢：所有输入、模式切换、查询和仲裁均由同一任务顺序执行，内部使用普通成员，不创建线程、定时器或异步队列。

- 遥控源独立保存 DR16、VT13、Referee 数据；活动源在线则保留，另一在线源有操作时接管，活动源失联时回退。
- 全部遥控离线时清零底盘、禁止开火；先更新 `Online()` 再触发控制上线/失联事件。
- 自动模式按底盘、云台各自的 AI 在线状态选择输入。开火必须同时满足视觉在线、AI 请求、遥控许可。
- 手动及 AI 底盘离线回退时，清零 `fx_n/fy_n/mz_nm` 并设置 `ff_valid = false`。
- 手动云台使用归一化量；AI 使用 rad、rad/s、rad/s²。底盘力前馈单位为 N、N、N·m。

`FeedRC()`、`FeedAIChassis()`、`FeedAIGimbal()`、`SetCtrlMode()` 立即更新成员，随后 getter 返回最新值。单独调用会同步仲裁发布；无效或重复模式不发布。`ApplyRCFrame()` 将一帧数据与绑定事件合并，`ApplyInput()` 支持需要先处理切换事件的输入。`UpdateInputs()` 在一个同步作用域内执行所有已注册输入回调，有变更时在返回前发布一组输出，避免同帧和同周期中间状态。输入作用域中的 getter 反映已经应用的数据；最终输出在最外层作用域结束时生成。

构造参数依次为 `mode`、`chassis_cmd_topic_name`、`gimbal_cmd_topic_name`、`launcher_cmd_topic_name`、`control_state_topic_name`。四个 Topic 默认名称及类型：

| Topic | 类型 | 用途 |
| --- | --- | --- |
| `chassis_cmd` | `CMD::ChassisCMD` | 底盘运动与力前馈 |
| `gimbal_cmd` | `CMD::GimbalCMD` | 云台参考 |
| `launcher_cmd` | `CMD::LauncherCMD` | 开火许可 |
| `control_state` | `CMD::ChassisControl` | 同次仲裁的底盘命令、模式、RC/AI 底盘在线信息 |

输入模块在构造期 `RegisterInput()`；默认 `OnMonitor()` 调用 `UpdateInputs()`。本工程 Gimbal 调用 `UseExternalRunner()` 后由其 1 ms 控制任务独占执行 `UpdateInputs()`，CMD 的监控钩子不再执行输入。Gimbal 首次监控钩子在全部模块构造与绑定完成后释放启动门。

通信线程只在源模块内缓存数据，释放交接锁后由所属任务提交 CMD。DR16/VT13 使用 `CMD.hpp` 中的 `ControlInbox` 交接带时间戳的原始帧；缓存满时报告一次中断，所属任务先失能，随后新帧恢复，无需重启。停止请求在同一周期内不得被后续开火请求覆盖。

运行约束：只允许一个 runner；模式事件必须来自该任务，禁止 ISR 访问 CMD。输出和控制状态事件回调可查询 CMD，不可重入写入。输入回调仅在初始化时注册；模块和 CMD 须具有应用全生命周期，当前接口不支持动态注销销毁。

完整验收：`python3 tools/tests/run_control_acceptance.py --firmware`。集成说明见根工程 `docs/control-acceptance.md`。
