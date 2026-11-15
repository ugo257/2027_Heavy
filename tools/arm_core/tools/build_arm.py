from pathlib import Path
import subprocess,json,re
ROOT=Path(__file__).resolve().parents[1];CC=Path(r'D:\Embedded-cmaketools\14.3 rel1\bin\arm-none-eabi-gcc.exe');AR=CC.with_name('arm-none-eabi-ar.exe');SIZE=CC.with_name('arm-none-eabi-size.exe')
def main():
    out=ROOT/'build/cortex_m4';out.mkdir(parents=True,exist_ok=True)
    flags=['-std=c11','-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard','-O2','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-fstack-usage','-ffunction-sections','-fdata-sections','-I',str(ROOT/'include')]
    objects=[]
    for source in sorted((ROOT/'src').glob('*.c')):
        obj=out/(source.stem+'.o');subprocess.run([str(CC),*flags,'-c',str(source),'-o',str(obj)],check=True);objects.append(obj)
    subprocess.run([str(AR),'rcs',str(out/'libarm_core.a'),*[str(o) for o in objects]],check=True)
    size=subprocess.check_output([str(SIZE),*[str(o) for o in objects]],text=True)
    stacks=[]
    for path in out.glob('*.su'):
        for line in path.read_text().splitlines():
            parts=line.split('\t')
            if len(parts)>=3:stacks.append({'function':parts[0].split(':')[-1],'bytes':int(parts[1]),'allocation':parts[2]})
    report={'target':'Cortex-M4 fpv4-sp-d16 hard-float C11 O2','library':'build/cortex_m4/libarm_core.a','size_output':size,
      'static_stack_per_function':sorted(stacks,key=lambda x:-x['bytes']),'on_board_WCET_measured':False,
      'warning':'Static per-function stack is not call-chain stack/high-water mark; libm and RTOS overhead not measured. Link -lm. No full firmware or hardware acceptance yet.'}
    (ROOT/'reports/arm_build.json').write_text(json.dumps(report,indent=2));print(size);print('Largest stack functions:',report['static_stack_per_function'][:8])
if __name__=='__main__':main()
