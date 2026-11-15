from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
def main():
    out=ROOT/'build';out.mkdir(exist_ok=True)
    cmd=['gcc','-std=c11','-O2','-fPIC','-shared','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-I',str(ROOT/'include'),*[str(x) for x in sorted((ROOT/'src').glob('*.c'))],'-lm','-o',str(out/'libarm_core.so')]
    subprocess.run(cmd,check=True);print(out/'libarm_core.so')
    fw=ROOT.parents[1];integ=ROOT/'integration/infantry'
    bridge=['gcc','-std=c11','-O2','-fPIC','-shared']
    for inc in [ROOT/'tests/stubs',integ/'driver',fw/'modules/motor',fw/'modules/daemon']:bridge+=['-I',str(inc)]
    bridge += [str(ROOT/'sim/dm_bridge.c'),str(integ/'driver/DMmotor.c'),'-lm','-o',str(out/'libarm_dm_bridge.so')]
    subprocess.run(bridge,check=True);print(out/'libarm_dm_bridge.so')
if __name__=='__main__':main()
