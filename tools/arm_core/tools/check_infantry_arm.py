"""Compile adapter/overlay against real Infantry headers and MCU flags.
No source or build file in Infantry is modified, no firmware is flashed.
"""
from pathlib import Path
import re,json,subprocess
ROOT=Path(__file__).resolve().parents[1];INF=ROOT.parents[1]
def main():
    compiler=r'D:\Embedded-cmaketools\14.3 rel1\bin\arm-none-eabi-gcc.exe'
    directories={p.parent for folder in ['application','bsp','modules','Core/Inc','USB_DEVICE'] for p in (INF/folder).rglob('*.h')}
    directories.update(INF/p for p in ['Drivers/STM32F4xx_HAL_Driver/Inc','Drivers/STM32F4xx_HAL_Driver/Inc/Legacy','Drivers/CMSIS/Include','Drivers/CMSIS/Device/ST/STM32F4xx/Include','Middlewares/Third_Party/FreeRTOS/Source/include','Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2','Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F','Middlewares/ST/ARM/DSP/Inc'])
    flags=['-DARM_MATH_CM4','-DUSE_HAL_DRIVER','-DSTM32F407xx',*['-I'+str(p) for p in sorted(directories)]]
    dst=ROOT/'build/cortex_m4';dst.mkdir(parents=True,exist_ok=True);integ=ROOT/'integration/infantry'
    extra=['-std=c11','-O2','-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-Wno-misleading-indentation','-fstack-usage']
    for source in [integ/'arm_application.c',integ/'driver/DMmotor.c']:
        cmd=[compiler,'-I'+str(integ/'driver'),'-I'+str(integ),'-I'+str(ROOT/'include'),*flags,*extra,'-c',str(source),'-o',str(dst/(source.stem+'_integration.o'))]
        subprocess.run(cmd,check=True)
    report={'real_Heavy_headers':True,'firmware_path':str(INF),'mcu_flags':'Cortex-M4 single-precision hard-float','adapter_and_driver_overlay_compiled':True,
      'original_project_modified':False,'complete_firmware_linked':False,'hardware_tested':False}
    (ROOT/'reports/infantry_compile.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()
