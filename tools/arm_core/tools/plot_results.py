from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
plt.rcParams['font.sans-serif']=['Microsoft YaHei','DejaVu Sans'];plt.rcParams['axes.unicode_minus']=False
fig,axes=plt.subplots(2,1,figsize=(10,6.8),sharex=True,layout='constrained')
for path,label in [('trace_line_0.csv','名义模型'),('trace_line_1_3.csv','质量/惯量 +10% 与额外阻尼')]:
    a=np.loadtxt(ROOT/'reports'/path,delimiter=',');axes[0].plot(a[:,0],a[:,-2]*1000,label=label);axes[1].plot(a[:,0],np.degrees(a[:,-1]),label=label)
axes[0].axhline(5,color='#a9473a',ls='--',lw=1,label='到位阈值 5 mm');axes[1].axhline(2,color='#a9473a',ls='--',lw=1,label='到位阈值 2°')
axes[0].set_ylabel('TCP 目标位置误差 / mm');axes[1].set_ylabel('TCP 目标姿态误差 / °');axes[1].set_xlabel('时间 / s')
for ax in axes:ax.grid(alpha=.25);ax.legend(fontsize=9);ax.axvline(1,color='#888888',ls=':',lw=1)
fig.suptitle('同一份 C 控制器的 MuJoCo 闭环结果（两条曲线为不同目标用例）\n500 Hz反馈与输出 · 2 ms延迟 · 实际DM编解码',fontsize=12)
fig.savefig(ROOT/'reports/tracking.png',dpi=160);print(ROOT/'reports/tracking.png')
