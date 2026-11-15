"""Export sources, verified libraries and the companion simulation model.

No original CAD or Infantry source tree is copied or modified. Regenerating
geometry or rerunning the Infantry checks still requires those original inputs.
"""
from pathlib import Path
import hashlib,json,zipfile

ROOT=Path(__file__).resolve().parents[1]

def main():
    archive=ROOT.parent/'arm_core_v0.2.zip'
    files=[]
    for directory in [ROOT,ROOT.parent/'arm_description']:
        for p in sorted(directory.rglob('*')):
            if not p.is_file() or '__pycache__' in p.parts or p.suffix in {'.pyc','.o','.su'}:
                continue
            if directory==ROOT and 'build' in p.relative_to(ROOT).parts and p.name not in {'libarm_core.so','libarm_dm_bridge.so','libarm_core.a'}:
                continue
            files.append(p)
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in files:z.write(p,str(p.relative_to(ROOT.parent)))
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        assert 'arm_core/include/arm_core.h' in z.namelist()
        assert 'arm_core/build/cortex_m4/libarm_core.a' in z.namelist()
        assert 'arm_description/heavy_arm.urdf' in z.namelist()
    result={'archive':str(archive),'files':len(files),'bytes':archive.stat().st_size,'sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'zip_integrity':'PASS','original_CAD_included':False,'original_Infantry_included':False}
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
