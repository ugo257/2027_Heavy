from pathlib import Path
import xml.etree.ElementTree as E
import json,struct,re
from cad_io import streams,streams_bytes,display_mesh

from cad_paths import CAD_DIR
SOURCE=CAD_DIR
OUT=Path(__file__).resolve().parents[1]
NS={'s':'http://www.solidworks.com/sw2003/schema'}

def massprop(data):
    r=E.fromstring(data['docProps/ISolidWorksInformation.xml'])
    for x in r.iter():
        if x.attrib.get('name')=='SW-MassProp-Config-0':
            t=' '.join(x.itertext())
            if ',' in t:return [float(x) for x in t.split(',')]
    return None

if __name__=='__main__':
    data=streams(SOURCE/'27重装-机械臂-11.3.SLDASM')
    xml=E.fromstring(data['swXmlContents/COMPINSTANCETREE']);ml=xml.find('s:swModelList',NS)
    models={x.attrib['id']:x for x in ml};files={x.attrib['id']:x.attrib for x in xml.findall('.//s:swFile',NS)}
    for mid,m in models.items():
        if m.findall('s:swReference',NS):continue
        name=m.attrib['swName'];f=files[m.attrib['swFileRef']];p=SOURCE/Path(f['swPath']).name
        if p.exists():d=streams(p)
        else:d=streams_bytes(data.get('ImportedComp/'+Path(f['swPath']).name,b''))
        mp=massprop(d) if 'docProps/ISolidWorksInformation.xml' in d else None
        print(name,'mass',round(mp[5],6) if mp else None,'display',len(d.get('Contents/DisplayLists',b'')),flush=True)
    directory=data['FaceTessellations/Directory']
    hits=list(re.finditer(b'\xff\xfe\xff',directory))
    print('DIRECTORY entries',len(hits))
    for hit in hits[:4]:
        p=hit.start()+3;n=directory[p];end=p+1+2*n
        name=directory[p+1:end].decode('utf-16le','replace')
        print('DIR',name,'TAIL',directory[end:end+65].hex(' '))
