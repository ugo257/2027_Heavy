# 机械臂 C11 函数包 v0.2

目标：**同一份 C 核心在 MuJoCo 与 DJI C 板复用**。Pinocchio 只作为电脑端数值参考，运行期核心不依赖 Python、Pinocchio、MuJoCo、HAL 或 RTOS。六轴位姿控制，夹爪开合暂不参与。

本版是已通过数值测试、闭环用例和 ARM 编译的**仿真开发版本**。硬件配置默认未标定，Infantry 适配入口会拒绝直接使能；不能把仿真暂定限位、力矩范围和增益当成实物额定值。

2026-11-15 更新：根据原生 SW 配合修正25个零件实例的归属，重新生成质量/惯量和网格；URDF、MJCF、Pinocchio、C统一使用 `tcp`。当前固件参考目录为 `D:\2027_Heavy\2027_Heavy`，该目录的application未接入本包，完整固件尚未验收。工作空间测试含真实DM驱动编解码，结果和失败用例见 `reports/workspace.json`；存在未通过的扰动/边界用例时，不得将其概括为全空间通过。

## 立即查看结果

双击 `查看C核心仿真.bat`，观察真正调用本包 C 控制器的 MuJoCo 运动。批处理按自身目录启动；其中 WSL 发行版和 Python 环境名称仍是本机配置。

文件入口：

| 路径 | 内容 |
|---|---|
| `include/arm_core.h` | 数学、规划、控制的公共接口 |
| `include/arm_io.h` | 电机反馈和关节状态、力矩之间的转换 |
| `src/` | 纯 C11、单精度、无堆分配的算法源码 |
| `config/model.json` | 模型参数、TCP、暂定运动约束、碰撞体和排除表、待标定项目 |
| `../arm_description/model_definition.json` | SW配合证据支持的零件归属修正、唯一TCP与地面定义、明确列出的模型近似 |
| `config/control_profile.json` | 运动约束与PD增益的编辑入口；重新生成C参数时保留此文件的设置 |
| `config/hardware_calibration.template.json` | 装配后填写的电机/传动标定记录模板；当前不自动加载到C板 |
| `config/standard_dh.csv`、`dh_reference.json` | 标准 DH 对照表及配套变换；核心以统一刚体模型计算 |
| `sim/scene.xml` | 具有同一 TCP 和力矩约束的 MJCF |
| `python/arm.py` | 仅负责调用已编译 C 库的电脑适配，不重写算法 |
| `integration/infantry/` | application 适配、驱动修复版本与补丁，原 Infantry 未修改 |
| `build/cortex_m4/libarm_core.a` | Cortex-M4、单精度 hard-float 编译的静态库 |
| `reports/` | 数值、闭环、编译和离线驱动验证结果及曲线数据 |

移植优先使用 `include/`＋`src/`，随目标工程重新编译，链接标准数学库 `-lm`。不要把 Linux 的 `libarm_core.so` 放到单片机上。

当前归档是 `../arm_core_v0.2.zip`；旧 `arm_core_v0.1.zip` 保留作历史版本，不包含本轮修正。源码重新构建DM仿真桥使用 `arm_core` 上两级固件目录中的 motor_def.h 和 daemon.h；归档不包含整套固件或原CAD。模型重新导出所需Python依赖列在 `../arm_description/requirements.txt`。

## 公共约定

- 基座系沿用 CAD，固定底座，+Z 向上；q=0 是 CAD 保存装配姿态。
- 六个关节按肩俯仰、肘俯仰、小臂自转、腕俯仰、腕偏转、末端自转排序。
- 单位：m、rad、rad/s、s、kg、kg·m²、N·m。
- 姿态四元数为 **w,x,y,z**，目标 TCP 位姿相对于基座。
- 雅可比行序为 **vx,vy,vz,wx,wy,wz**，速度在基座方向表达，原点在 TCP。角速度不是欧拉角变化率。
- 公共接口始终是六个关节角，不暴露 Pinocchio 的 12 维连续关节配置。
- 所有模型参数在运行期间保持不变；需要修改时停止控制、验证模型并重新初始化。

TCP 定义为两个相向 T 形主夹持面的面积质心中点，每侧面积402 mm²，相对旧 `tool_mount` 沿 +Z 为86.526865671642 mm。TCP +X 沿夹指开合方向、+Z 朝夹爪前方；变换见配置文件。它是几何工作中心，实际夹持工具偏差可在装配后标定。

## 已实现函数

- `arm_fk`：正运动学。
- `arm_jacobian`、`arm_velocity`：雅可比和速度运动学。
- `arm_com`：含固定底座/调试架的总质心与总质量；动力学只计算六个运动刚体。
- `arm_rnea`、`arm_gravity`、`arm_bias`、`arm_mass_matrix`：逆动力学、重力、科氏/离心加重力项、质量矩阵。满足 `tau=M*qdd+bias`，速度相关项可用 `bias-gravity` 得到。
- `arm_singularity`：最小奇异值，角速度行按0.15 m特征长度缩放；阈值随模型配置。
- `arm_ik`：当前关节解作种子，阻尼最小二乘、有界步长和线搜索，默认最多120次迭代。失败表示本次有界搜索未得到解，**不是全局不可达证明**；不自动切到远处肘形。
- `arm_plan_joint`：先求目标位姿的近解，再生成关节点到点运动。
- `arm_plan_line`：TCP位置直线、姿态最短四元数插值，33个IK节点；关节五次 Hermite 插值与整体五次时间律，起止速度、加速度为零。
- 两种规划均检查暂定关节范围、速度、加速度、奇异性、碰撞代理和逆动力学预测力矩，按约束延长时长，预留15%力矩余量。默认从1秒尝试，不是最短时间优化器。
- `arm_step`：关节PD＋模型重力补偿，输出纯关节力矩，附限幅、力矩斜率限制和到位判定。
- `arm_control_stop`：匹配当前参考位置、速度、加速度后减速停稳，再保持；检查限位、碰撞、奇异性、速度/加速度和预测关节力矩。若停止路径不满足约束，返回失败，调用者按故障策略处理。
- 输入、模型检查及错误状态明确返回，反馈丢失、急停等故障锁存，必须显式复位。

到位基线：位置≤5 mm、姿态≤2°、关节速度≤0.03 rad/s，连续0.5 s。没有额外挂载物体，夹爪自身计入模型。

## 最小调用顺序

下面对象都应使用静态存储，不放进4KB任务栈：

```c
#include "arm_core.h"
static ArmModel model;
static ArmWorkspace work;
static ArmController controller;
static ArmPlan plan;

// 初始化，在已取得有效关节反馈后：
arm_model_default(&model);
arm_control_init(&model, &controller, &state, &work);

// 低频规划任务：target 是 TCP 相对基座的目标位姿。
ArmResult r = arm_plan_joint(&model, state.q, &target, &plan, &work);
if (r == ARM_OK)
    arm_control_start(&model, &controller, &state, &plan);

// 周期控制任务：state 来自反馈快照，dt 为真实周期秒数。
ArmOutput out;
arm_step(&model, &controller, &state, dt, &out, &work);
// out.enable==1: out.tau 进入电机适配层；否则执行失能/故障处理。
```

上述是串行示例。实际多任务必须为规划和控制分别使用工作区，活动 `ArmPlan` 必须在整个执行期间保持不变；提供的 Infantry 示例使用双计划缓冲。

运行中更新目标的入口保留，但第一版明确采用“受控停止后重新规划”：移动中 `arm_control_start` 返回 `ARM_BUSY`，不能直接跳到新参考。新目标失败时不覆盖活动计划。

## 仿真与硬件接口对齐

仿真：`data.qpos/qvel → 模拟电机轴量 → DM反馈编码/实际驱动解码 → arm_motor_state → arm_step → arm_motor_torque → 实际MIT编码/解码 → data.ctrl`。MuJoCo执行器 gear=1，输入为关节输出端力矩。仿真A默认单位阵，P=12.5、V=30、T为暂定约束；这些仅为仿真假设，没有替代实物标定。CAN调度、真实电流环和机械摩擦没有被完整模拟。

实车：`DM反馈 → arm_motor_state → ArmState → arm_step → arm_motor_torque → DMMotorSetTorque → MIT报文`。MIT报文中的 Kp、Kd 为0，PD在本包中计算。

传动采用通用线性映射：

```text
q = A * (phi - phi0)
qd = A * phi_dot
tau_DM = A^T * tau_joint
```

`phi`、`tau_DM` 已是达妙减速电机输出轴量，不能再重复除以内部减速比。A 可以表达反向、外部减速比及耦合。转换后超出电机协议力矩范围会返回错误，不悄悄逐电机截断而改变关节力矩关系。

P/V/T 协议映射范围与关节物理约束分别配置。角度回绕周期显式配置，0表示不展开；不把 PMAX 当作机械一圈。首帧只建立展开位置，实车重启后还需要零位/回零策略，不能仅靠累计值恢复多圈绝对位置。

规划后调用 `arm_motor_plan_validate`，检查映射后的电机位置、速度及预测力矩；现有application适配与仿真均调用该入口。电机侧不可执行的计划会在启动前拒绝。它仍是采样检查，实物持续/峰值能力应同时落实为校准约束。

## Infantry 接入

原项目保持不变。`integration/infantry/dm_driver.patch` 与 `driver/DMmotor.c,h` 是基于当前源文件准备的最小适配版本，修改包括：容量8、越界防护、初始失能、显式使能、故障锁存、10ms命令超时、反馈时间戳、短帧防护、首帧及显式周期角度展开、NaN拒绝、非阻塞发送失败锁存。

修改失能语义会影响原云台代码，**不要未经评审把补丁用于仍运行原云台任务的固件**。机械臂 application 适配依赖这份补丁，不能配原驱动直接使用。当前补丁以新固件目录为基线生成，尚未应用；运行中电机意外从state=1变为state=0会锁存故障，不再自动重使能。

接入示例见 `integration/infantry/arm_application.h,c`：

1. 把本包核心作为模块加入构建，application 包含适配接口。
2. 分配两路 CAN、各电机 CAN ID / Master ID 和逐实例 P/V/T；六个位姿轴建议分3+3，两路仅供机械臂时500Hz发送/反馈负载更有余量。夹爪第七台仅预留容量，不会被本版自动控制。
3. 填入 A、phi0、回绕规则、物理约束，复核后设置模型和映射的 calibrated 标志。
4. `ArmApplication_Init` 注册但不使能；等待失能查询反馈，再由控制任务处理明确的复位请求 `ArmApplication_Reset`。随后等待六台电机均回报运行状态，100ms内未完成则故障失能；等待期间不接受运动目标。
5. 控制任务以1kHz目标持续调用 `ArmApplication_Update`，使用实际dt。规划调用 **必须放在较低优先级任务**，控制任务不能为逆解/规划停住，否则命令看门狗会触发。目标/状态通过框架消息中心交互。
6. 控制任务保存反馈快照并提交全部六轴力矩，规划使用快照和独立工作区；规划完成后检查起点漂移再提交。一个低优先级规划任务拥有计划缓冲，禁止并发规划。

故障锁存后，`out.enable=0`，适配层停止力矩提交并发协议失能。失能不提供重力保持；实车需验证支撑、急停和断电行为。正常受控停止仍保持力矩输出。

## 验证与复现

本机 WSL2 环境 `/home/ugo/.venvs/heavy-arm/bin/python` 已有 Pinocchio、MuJoCo、NumPy、SciPy；本轮补充了 WSL host GCC。

```bash
cd /mnt/d/2027_Heavy/2027_Heavy/tools/arm_core
~/.venvs/heavy-arm/bin/python -B tools/build_host.py
~/.venvs/heavy-arm/bin/python -B tools/run_native.py
~/.venvs/heavy-arm/bin/python -B tests/numerical.py
~/.venvs/heavy-arm/bin/python -B tests/closed_loop.py
~/.venvs/heavy-arm/bin/python -B tests/workspace.py --cases 96
~/.venvs/heavy-arm/bin/python -B sim/viewer.py --line
```

Windows 用 `python -B tools/build_arm.py` 重建 ARM 静态库，用 `tools/check_infantry_arm.py` 对真实 Infantry 头文件检查适配代码与驱动编译。

已完成：300组随机状态的 FK/J/M/g/RNEA/质心/奇异值对照，300次近种子逆解、100组DH对照；10个附近目标的C闭环运动，包含500Hz反馈、2ms延迟、12位力矩量化，部分用例增加10%质量/惯量和额外阻尼；ASan/UBSan native测试、驱动报文及application启动/急停/复位/故障mock测试；Cortex-M4单精度核心和真实Infantry头文件的编译检查。

验证是指定用例，不代表整个工作空间都已验收。没有做实物测试、整套固件链接、C板DWT耗时或栈高水位测量；host耗时不能当成M4耗时。ARM各函数静态栈估计见报告，函数调用链、libm和中断栈仍需上板检查。

## 模型修正与当前限制

本轮根据原生SW的406个配合记录修正底座铝柱、肩肘限位/支承、小臂输出轴、腕部涨紧和末端支承等25个实例。365项可直接解析的配合在76组姿态的几何代入检查中通过；这不是SW原生多姿态求解器验收。复杂配合、装配上下文引用、两项锁姿平行配合和滑环内部转子近似在报告中明确列出。肩端远置带轮内部自转、反射惯量仍未建模，1:1配合不能单独确定实物电机映射A。

碰撞使用20个主要结构组件OBB、164个自碰撞代理对，3对零位支承包络重叠被明确排除；不再整批排除所有相邻连杆对。MuJoCo启用了相同代理对及地面接触，地面统一为z=-0.027m。小零件、电机内部、完整线缆和夹爪接触仍未覆盖；路径256段采样不是连续碰撞证明。模型基座底面与地面仅约2mm间隙，当前规划检查margin=1mm，不能直接给所有固定/运动体统一加大margin。

当前角度、速度、加速度、力矩限制和PD增益全部标为 **PROVISIONAL_SIMULATION_ONLY**；来源不明的实物约束没有伪装成厂商参数。它们通过已记录的仿真用例，但必须等机械装配、材料/质量复核、驱动参数读取、限位与传动标定后重新验证。

装配后仍需：机械/电机参数表、编码器零位与方向、外部传动映射、实际TCP校准、摩擦/负载、CAN总线实测、C板最坏计算耗时、完整固件链接和逐关节实测。算法包已经形成，不表示这些物理准备工作可以省略。
