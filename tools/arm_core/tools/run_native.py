from pathlib import Path
import subprocess,json
ROOT=Path(__file__).resolve().parents[1];INF=ROOT.parents[1];out=ROOT/'build';out.mkdir(exist_ok=True)
def run(cmd):subprocess.run([str(x) for x in cmd],check=True)
flags=['gcc','-std=c11','-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-Wall','-Wextra','-Werror','-Wno-misleading-indentation']
src=list((ROOT/'src').glob('*.c'))
run([*flags,'-I',ROOT/'include',ROOT/'tests/native.c',*src,'-lm','-o',out/'native_tests']);run([out/'native_tests'])
integ=ROOT/'integration/infantry'
run([*flags,'-I',ROOT/'tests/stubs','-I',ROOT/'include','-I',integ,'-I',integ/'driver','-I',INF/'modules/motor','-I',INF/'modules/daemon',ROOT/'tests/dm_driver_mock.c',integ/'driver/DMmotor.c',integ/'arm_application.c',*src,'-lm','-o',out/'dm_mock']);run([out/'dm_mock']);run([out/'dm_mock','app']);run([out/'dm_mock','app_timeout'])
(ROOT/'reports/native.json').write_text(json.dumps({'asan_ubsan':True,'core_tests':'PASS','driver_overlay_tests':'PASS','application_workflow_tests':'PASS','application_startup_timeout_test':'PASS','hardware_tested':False},indent=2))
