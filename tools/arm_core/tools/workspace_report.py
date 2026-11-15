"""Readable, reproducible workspace audit from the final machine report."""
from pathlib import Path
import json,collections
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]

def main():
    report=json.loads((ROOT/'reports/workspace.json').read_text(encoding='utf-8'));cases=report['cases']
    definition=json.loads((ROOT.parent/'arm_description/model_definition.json').read_text(encoding='utf-8'))
    cfg=json.loads((ROOT.parent/'arm_description/model_config.json').read_text(encoding='utf-8'))
    mate=json.loads((ROOT.parent/'arm_description/validation_sw_mates.json').read_text(encoding='utf-8'))
    grouped=collections.Counter((c['type'],c['perturbed'],c['status']) for c in cases)
    nominal_pass=[c for c in cases if not c['perturbed'] and c['status']=='passed']
    lines=['# v0.2 模型修正与工作空间检查（2026-11-15）','',
      '模型导出、C参数、DM仿真接口与测试已更新。**有限采样不能证明连续全空间安全；本次扰动和边界失败如实列出，不宣称全空间已通过。**','',
      '## 已完成的修正','',
      '- 使用原生SW只读接口捕获140个顶层零件、406个配合；原CAD未改动。',
      f"- 按配合证据修正{len(definition['body_overrides'])}个零件实例，重新导出网格并聚合质量、质心和惯量。总质量仍为{cfg['export_mass_kg']:.9f}kg。",
      '- 纠正末端进一步追查结果：内圈压紧件属于第六轴输出侧，限位块1跟随内圈；外圈压紧件属于第五连杆支承侧。',
      '- TCP固定变换现在直接写入URDF，Pinocchio和MuJoCo按tcp名称读取；地面统一为z=-0.027m。',
      '- C检查与MuJoCo共用20个OBB、164个自碰撞对和地面；仅保留3对已列出的零位支承重叠排除。',
      '- 仿真经过实际DM驱动的反馈解码、MIT命令编码以及C电机/关节映射，包含16位位置、12位速度和力矩量化、500Hz刷新和2ms反馈延迟。实际CAN调度和电流环未模拟。',
      '- 停止参考匹配初始加速度，并补充停止路径的奇异性和动力学约束检查；规划后补充电机侧位置/速度/力矩检查。',
      '- 修复配套DM驱动及适配层的运行中意外失能自动重使能问题，增加回归用例。补丁尚未应用到固件。',
      '- 第六轴仿真Kd从0.025调整为0.02以改善量化反馈下的静止判定；其他增益和原暂定限位未缩小以掩盖失败。','',
      '## 模型一致性证据','',
      f"- {mate['checked_mates']}项可直接解析的SW配合，在{mate['poses']}组逐轴/组合姿态的几何代入检查中通过。不是SW求解器驱动后的多姿态验收。",
      '- 300组C/Pinocchio数值对照、300次近种子逆解、100组DH对照、50组URDF/MJCF对照通过。',
      '- 独立按每个CAD零件质量和惯量计算JᵀIJ与重力，100组姿态对照通过，不仅比较已聚合的连杆参数。',
      '- 2048组C/MuJoCo碰撞对照中，C没有漏掉采样到的MuJoCo接触；1组C拒绝但MuJoCo未生成接触。双方并非完全一致，原始姿态保存在component_dynamics.json。',
      '- C11 native及DM mock的ASan/UBSan测试、Cortex-M4静态库和当前2027_Heavy真实头文件的适配编译通过。','',
      '## 工作空间范围与结果','',
      '范围为仿真暂定关节盒，非实物机械限位：',
      f"- 下限rad：{report['joint_min']}",f"- 上限rad：{report['joint_max']}",
      f"- 均匀网格{report['grid_samples']}点，Sobol采样{report['sobol_samples']}点；共{report['grid_samples']+report['sobol_samples']}点。",
      f"- 筛选结果：{report['survey_counts']}。碰撞拒绝表示该姿态不满足当前代理模型，不能据此断言真实结构必然碰撞。",
      f"- 对512个分散可行目标，近种子逆解：{report['IK_counts']['near_seed']}；零位种子逆解：{report['IK_counts']['from_zero']}。UNREACHABLE表示有界迭代未找到解，不是全局不可达证明。",
      f"- 闭环共{len(cases)}例：{report['case_counts']}。规划拒绝不计为已执行控制成功。",'',
      '| 测试类型 | +10%质量/惯量与阻尼 | 结果 | 数量 |','|---|---|---|---|']
    for (kind,pert,status),count in sorted(grouped.items()):lines.append(f'| {kind} | {pert} | {status} | {count} |')
    if nominal_pass:
        lines += ['',f"名义模型通过用例的最差最终误差：位置{max(c['final_position_mm'] for c in nominal_pass):.4f}mm，姿态{max(c['final_orientation_deg'] for c in nominal_pass):.4f}°。",
          f"规划拒绝原因：{dict(collections.Counter(c['reason'] for c in cases if c['status']=='plan_rejected'))}。其中UNREACHABLE是当前有界IK/路径生成失败，不等于全局无解。"]
    lines += ['','## 未通过的已执行用例','', '| 编号 | 类型/扰动 | 结果 | 位置误差mm | 姿态误差deg | 原因 |','|---|---|---|---|---|---|']
    for i in report['executed_failures']:
        c=cases[i];lines.append(f"| {i} | {c['type']} / {c['perturbed']} | {c['status']} | {c.get('final_position_mm','—')} | {c.get('final_orientation_deg','—')} | {c.get('reason','')} |")
    lines += ['','每个用例的start、goal、停止时刻及失败关节状态都保存在reports/workspace.json，可精确复现。',
      '通过的静态保持要求真实仿真位置≤5mm、姿态≤2°、各轴速度≤0.03rad/s持续2秒，且C状态为SETTLED；运动结束后持续1秒。测试超时是5秒收敛窗口加轨迹时长。',
      '', '## 尚未解决的边界','',
      '- 10%质量/惯量误差下的PD+重力补偿存在静态偏差和边界漂移风险；不能只提高增益就声称已解决。应先实测质量/传动，再决定是否加入受限积分或扰动补偿。',
      '- 远置带轮1:1配合、滑环内部转子、两项锁姿平行配合没有被伪造为已经确认的机构动态；见model_definition.json中的明确近似。',
      '- 真实机械限位、DM协议范围、零位、多圈回零和A传动映射仍待标定，硬件使能入口保持禁止未标定。',
      '- 本轮没有修改原SW或2027_Heavy/application；固件仍处于旧应用待重建状态。原审查中的DWT/CAN底层问题、完整固件接入、上板耗时和栈占用仍待后续处理。',
      '- 不使用缩小关节范围、放宽5mm/2°指标、忽略失败用例的方法宣布全空间通过。','',
      '## 复现','',
      '在WSL heavy-arm环境运行 tools/build_host.py，然后运行 tests/numerical.py、tests/dh_check.py、tests/component_dynamics.py、tests/closed_loop.py 和 tests/workspace.py --cases 96。',
      '模型重新生成顺序：arm_description/tools/export_arm.py → make_mujoco.py → validate_sw_mates.py → arm_core/tools/generate_model.py → build_host.py。export_arm.py依赖fast-simplification及原CAD文件。']
    (ROOT/'reports/修正与工作空间检查.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    rows=np.load(ROOT/'reports/workspace_samples.npz')['samples'];fig,axes=plt.subplots(1,2,figsize=(12,5),layout='constrained')
    for ax,dims,title in [(axes[0],(6,8),'TCP projection: X-Z'),(axes[1],(6,7),'TCP projection: X-Y')]:
        for code,color,label in [(5,'#b6bbc4','collision rejected'),(4,'#db8e25','near singular'),(0,'#167d95','accepted configuration')]:
            pts=rows[rows[:,9]==code];pts=pts[::max(1,len(pts)//7000)]
            ax.scatter(pts[:,dims[0]],pts[:,dims[1]],s=2,c=color,alpha=.35,label=label,rasterized=True)
        ax.set_title(title);ax.set_xlabel('X / m');ax.set_ylabel('Z / m' if dims[1]==8 else 'Y / m');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(markerscale=3,fontsize=8)
    fig.suptitle('Finite sampled configuration-space classification (not a continuous workspace proof)')
    fig.savefig(ROOT/'reports/workspace.png',dpi=160)
    print(ROOT/'reports/修正与工作空间检查.md')

if __name__=='__main__':main()
