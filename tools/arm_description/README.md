# 重装机械臂 URDF / MuJoCo / Pinocchio

2026-11-15 更新：依据原生SW配合修正25个零件实例的随动归属，重新聚合惯量；TCP和地面以 `model_definition.json` 为统一来源。365项可解析配合在76组姿态的几何代入检查中通过，复杂配合及内部转动近似仍明确保留。对应C工具包与工作空间结果见 `../arm_core/README.md`。运行实际C控制器请用 `../arm_core/查看C核心仿真.bat`。

已从 `D:\2027_Heavy\27sw\27重装11.3\27重装-机械臂-11.3.SLDASM` 导出，原始 CAD 未修改。当前版本为**固定底座、6 个转动关节、夹爪保持保存的开度**，保留机械臂、电机、夹爪和调试支架外形。第七个电机是夹爪开合驱动，本版未加入其齿轮齿条联动。

## 直接使用

- **Windows 看模型：** 双击 `查看机械臂.bat`。默认只预览模型，鼠标可旋转、缩放。
- **Pinocchio 示例：** 双击 `运行Pinocchio示例.bat`。在 WSL2 中打印末端位姿、雅可比、质量矩阵和重力关节力矩。

也可以在 Windows 本目录中运行：

```powershell
python -B demo_mujoco.py
python -B demo_mujoco.py --simulate
```

`--simulate` 采用当前仿真模型的重力补偿与 PD 保持参考姿态，便于确认动力学接口可运行；这不是实际电机控制器或实物控制参数。

本机已经验证的解释器：

- Windows：`C:\Users\zhou_\miniconda3\python.exe`，MuJoCo 3.7.0。
- Ubuntu-24.04 WSL2：`/home/ugo/.venvs/heavy-arm/bin/python`，Pinocchio 4.1.0、MuJoCo 3.7.0。该环境与已有 `mujoco-work/.venv` 分开；补充了系统运行库 `libgomp1`。

WSL2 中：

```bash
cd /mnt/d/2027_Heavy/2027_Heavy/tools/arm_description
~/.venvs/heavy-arm/bin/python -B demo_pinocchio.py
~/.venvs/heavy-arm/bin/python -B tools/validate_model.py
```

## 文件

| 文件 | 用途 |
|---|---|
| `heavy_arm.urdf` | 两套软件共用的机器人结构、惯量和几何定义 |
| `meshes/` | 7 个刚体的显示网格，必须与模型一并保留 |
| `heavy_arm.xml` | MuJoCo 场景，附有 6 个力矩执行器 |
| `demo_mujoco.py` | MuJoCo 预览与重力补偿运行示例 |
| `demo_pinocchio.py` | Pinocchio 正运动学、雅可比、质量矩阵和重力项示例 |
| `model_config.json` | 坐标、轴线、惯量、待确认项及 CAD 零件归属 |
| `components.csv` | CAD 零件到运动刚体的对应表，供机械核查 |
| `validation_both.json` | 两套引擎的一致性验证结果 |
| `tools/` | 重新导出、转换和验证工具及 CAD 质量属性快照 |

复制到其他位置时，保留该目录结构即可，网格均使用相对路径。批处理中的解释器及项目路径是本机入口，移动后需相应调整。

## 坐标、零位与控制输入

长度为 m，角度为 rad，质量为 kg，惯量为 kg·m²，关节输出力矩为 N·m。

`base_link` 与原 CAD 主装配体坐标系一致，CAD +Z 为上方。`q=0` 为模型保存的装配姿态。关节轴与此前 DH 建模中选择的轴线、正方向一致；URDF 直接记录轴线和相邻坐标变换，不要求使用 DH 坐标系。

| 顺序 | 关节名 | 运动 | 电机 |
|---|---|---|---|
| 1 | `joint_1` | 肩部俯仰 | DM6248 |
| 2 | `joint_2` | 肘部俯仰 | DM4340 |
| 3 | `joint_3` | 小臂自转 | DM-J4310-2EC-V1.1 |
| 4 | `joint_4` | 腕部俯仰 | DM-J4310-2EC-V1.1 |
| 5 | `joint_5` | 腕部偏转 | DM6220 |
| 6 | `joint_6` | 末端自转 | DM3507 |

`tool_mount` 是 `27重装-j6-安装轴-11.3.SLDASM.SLDPRT` 第1实例的局部坐标系；其子坐标系 `tcp` 为两侧主夹持面面积中心的中点，相对安装轴沿+Z平移86.526865671642mm，并绕+Z旋转90°。该固定变换已写入URDF，Pinocchio可直接按 `tcp` 获取末端位姿，无需外部再补一次变换。实物TCP校准仍待完成。

MuJoCo 中 `data.qpos` 和 `data.ctrl` 均按上表顺序排列：

```python
data.ctrl[:] = tau_joint  # 六个关节输出端力矩，N·m
```

这里的 gear=1 表示输入已经是关节输出端力矩。尚未加入电机转子、减速器、同步带的独立动态，也没有自动把它换成电机电流或达妙 MIT 报文；接实物时仍需核对传动关系、编码器零位和正方向。

由于机械限位尚未确认，URDF 暂用 `continuous` 表示转动关节未设置角度限制，**不代表实物可以无限旋转**。这使 Pinocchio 使用 `nq=12, nv=6`：每个连续转动关节的配置是 `(cos q, sin q)`，速度和力矩仍各有 6 个量。示例中的 `configuration(model, angles)` 已封装六个弧度角到 Pinocchio 配置的转换。不要直接把六个角度当作 Pinocchio 的完整 q。

```python
import numpy as np
import pinocchio as pin
from demo_pinocchio import ROOT, configuration

model = pin.buildModelFromUrdf(str(ROOT / 'heavy_arm.urdf'))
q_pin = configuration(model, np.zeros(6))
visual = pin.buildGeomFromUrdf(
    model, str(ROOT / 'heavy_arm.urdf'), pin.GeometryType.VISUAL,
    package_dirs=[str(ROOT)],
)
```

## 质量、惯量及适用范围

质量与惯量来自 SolidWorks 只读重算，包含电机子装配的质量覆盖。整个模型含固定底座和调试支架的 CAD 总质量约 **5.15819 kg**。旧文件保存的总质量缓存为 4.90694 kg，导出采用当前重算值；没有为匹配旧缓存而人为缩放质量。各刚体质心和完整惯量张量已按坐标旋转、平行轴定理合并，惯性积符号也已转换。

当前参考姿态下，导出模型的总质量、总质心、总惯量与 SolidWorks 当前完整装配计算结果一致。但 CAD 材料、质量覆盖值与实际装配是否一致仍需机械核实。

零件随动归属见 `components.csv`，本轮修正及其原生配合证据见 `model_definition.json`。电机整机质量归入安装支撑侧，轴承归入一侧，内部转子、齿轮及轴承内外圈尚未分离。配合几何一致性与SW求解器多姿态验收是不同验证，后者尚未完成。

**尚未完成：** 实物角度/速度/力矩限位、传动映射、摩擦与转子惯量、夹爪开合、实物TCP校准、额外负载和精细接触模型。本目录URDF粗包围盒及基础MuJoCo预览场景不是碰撞验收模型；`arm_core/sim/scene.xml` 已启用20个组件代理体的164对自碰撞接触及统一地面。完整零件/线缆/夹持接触仍未覆盖。

## 已完成验证

使用 MuJoCo 3.7.0 与 Pinocchio 4.1.0：

- URDF、7 个显示网格和 MuJoCo 场景均成功加载。
- CAD 总质量、质心、惯量在导出前后守恒。
- 对 50 组关节角比较 CAD 轴线计算、MuJoCo、Pinocchio 的末端位姿。
- 比较两套引擎的雅可比、质量矩阵与重力项，全部通过。
- MuJoCo 重力补偿运行 1000 步，无数值异常。

这验证了文件可用性和模型实现的一致性，不等同于实物动力学或接触精度验证。

## 重新导出

保持源 CAD 版本不变时，可使用 `tools/export_arm.py` 重建 URDF，再用 `tools/make_mujoco.py` 重建 MJCF。导出工具额外依赖 NumPy、SciPy、Trimesh、fast-simplification；运行示例只需相应引擎和 NumPy。

`tools/cad_reference/` 保存的是本次源装配版本的 SolidWorks 重算质量属性。若修改了 CAD，需要重新读取这些属性及关节轴线，再重新分配零件、生成模型和验证；不能把旧属性直接套到新版本。

接口参考：[MuJoCo URDF 导入说明](https://mujoco.readthedocs.io/en/stable/modeling.html#urdf-extensions)、[Pinocchio 模型加载](https://docs.ros.org/en/rolling/p/pinocchio/doc/b-examples/a-model.html)。

当前目录布局：固件目录下的 `tools/` 同时包含 `arm_core/` 与 `arm_description/`。CAD 默认从固件目录的上一级 `27sw/27重装11.3/` 读取；位置不同时，设置环境变量 `HEAVY_ARM_CAD_DIR` 为包含装配及零件的目录。运行现有模型仿真无需读取 CAD；重新导出或生成碰撞模型需要 CAD。
