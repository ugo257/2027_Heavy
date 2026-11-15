"""Read verified saved SolidWorks streams and display triangle arrays, read-only."""
from pathlib import Path
import re,struct,zlib
import numpy as np
import trimesh

def streams_bytes(b):
    result={}
    for hit in re.finditer(re.escape(b'\x14\x00\x06\x00\x08\x00'),b):
        i=hit.start()
        try:
            _,_,_,_,crc,cs,us,n,extra=struct.unpack_from('<HHHIIIIHH',b,i)
            if n>250 or us>500_000_000:continue
            name=bytes((v>>4)|((v&15)<<4) for v in b[i+26:i+26+n]).decode('utf-8')
            start=i+26+n+extra;data=zlib.decompress(b[start:start+cs],-15)
            if len(data)==us and zlib.crc32(data)==crc:result[name]=data
        except (ValueError,struct.error,zlib.error,UnicodeDecodeError):continue
    return result

def streams(path):return streams_bytes(Path(path).read_bytes())

def display_mesh(b):
    vp=struct.pack('<iii',12,100,2);ip=struct.pack('<iii',4,8,2);meshes=[]
    for hit in re.finditer(re.escape(vp),b):
        pos=hit.start();n=struct.unpack_from('<i',b,pos+12)[0]
        if not 3<=n<1_000_000:continue
        end=pos+16+n*12
        if b[end:end+12]!=vp or struct.unpack_from('<i',b,end+12)[0]!=n:continue
        prev=b.rfind(ip,0,pos)
        if prev<0:continue
        count=struct.unpack_from('<i',b,prev+12)[0]
        if count<1 or prev+16+count*4!=pos:continue
        counts=np.frombuffer(b,dtype='<i4',count=count,offset=prev+16)
        if counts.sum()!=n or counts.min()<3:continue
        v=np.frombuffer(b,dtype='<f4',count=n*3,offset=pos+16).reshape(-1,3).copy()
        normal=np.frombuffer(b,dtype='<f4',count=n*3,offset=end+16).reshape(-1,3)
        faces=[];s=0
        for c in counts:
            for k in range(int(c)-2):faces.append([s+k,s+k+1,s+k+2] if k%2==0 else [s+k+1,s+k,s+k+2])
            s+=int(c)
        f=np.array(faces);cross=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
        keep=np.linalg.norm(cross,axis=1)>1e-12;f=f[keep];cross=cross[keep]
        reverse=np.einsum('ij,ij->i',cross,normal[f].mean(axis=1))<0
        f[reverse]=f[reverse][:,[0,2,1]]
        meshes.append(trimesh.Trimesh(v,f,process=True))
    if not meshes:raise ValueError('No supported triangle arrays')
    mesh=trimesh.util.concatenate(meshes);mesh.merge_vertices();mesh.fix_normals();return mesh
