"""Capture native SolidWorks mate geometry without rebuilding or saving CAD."""
from pathlib import Path
import json,hashlib,collections
import pythoncom,win32com.client

ROOT=Path(__file__).resolve().parents[1]
from cad_paths import ASSEMBLY_PATH
SOURCE=ASSEMBLY_PATH
def value(obj,name,*args):
    x=getattr(obj,name)
    return x(*args) if callable(x) and not hasattr(x,'_oleobj_') else x

def main():
    pythoncom.CoInitialize();sw=win32com.client.Dispatch('SldWorks.Application')
    err=win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0)
    warn=win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0)
    print('Opening assembly read-only...',flush=True)
    doc=sw.OpenDoc6(str(SOURCE),2,3,'',err,warn)
    if doc is None or not value(doc,'IsOpenedReadOnly'):raise RuntimeError('Read-only assembly required')
    components=[]
    for c in doc.GetComponents(True):
        components.append({'name':value(c,'Name2'),'fixed':bool(value(c,'IsFixed')),
          'constraint_status':value(c,'GetConstrainedStatus'),'transform':list(value(value(c,'Transform2'),'ArrayData'))})
    mates=[];feature=value(doc,'FirstFeature')
    while feature:
        if value(feature,'GetTypeName2')=='MateGroup':
            sub=value(feature,'GetFirstSubFeature')
            while sub:
                m=value(sub,'GetSpecificFeature2');entities=[]
                for i in range(value(m,'GetMateEntityCount')):
                    e=m.MateEntity(i);c=value(e,'ReferenceComponent')
                    entities.append({'component':value(c,'Name2') if c else None,'type':value(e,'ReferenceType'),
                      'params':list(value(e,'EntityParams') or [])})
                row={'name':value(sub,'Name'),'type':value(sub,'GetTypeName2'),'suppressed':bool(value(sub,'IsSuppressed')),
                     'error_code':value(sub,'GetErrorCode'),'entities':entities,'properties':{}}
                definition=value(sub,'GetDefinition')
                for key in ['LockRotation','GearRatioNumerator','GearRatioDenominator','Reverse','Distance','MinimumDistance','MaximumDistance']:
                    try:row['properties'][key]=value(definition,key)
                    except (AttributeError,pythoncom.com_error):pass
                mates.append(row);sub=value(sub,'GetNextSubFeature')
        feature=value(feature,'GetNextFeature')
    result={'source_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),'capture_date':'2026-11-14',
      'solidworks_revision':value(sw,'RevisionNumber'),'read_only':True,'rebuilt':False,'open_error':err.value,
      'open_warning':warn.value,'save_flag':value(doc,'GetSaveFlag'),'components':components,'mates':mates}
    (ROOT/'tools/cad_reference/assembly_mates.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    if not result['save_flag']:sw.CloseDoc(value(doc,'GetTitle'))
    print('Captured',len(components),'components,',len(mates),'mates; no CAD save.',flush=True)

if __name__=='__main__':main()
