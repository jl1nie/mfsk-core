# usage: uv run --no-project --with numpy python scripts/rtltcp/lines.py RECORDING.bin OUT.json
# Tracks the strongest line 240-300 kHz from the centre in ~1 s blocks of an 8-bit 1.024 MS/s recording: a
# sample clock that changed would move it (docs/notes/RTLTCP_RATE.md).
import numpy as np, struct, sys
f=open(sys.argv[1],"rb"); t0,n=struct.unpack("<qI",f.read(12))
fs=1.024e6; N=1<<16; win=np.hanning(N).astype(np.float32)
lo,hi=int((240e3)/fs*N+N/2),int((300e3)/fs*N+N/2)   # fftshift'ed bins for 240..300 kHz
buf=bytearray(); first=True
sec=[]; acc=None; nacc=0; total=0
def block(raw):
    a=np.frombuffer(raw,dtype=np.uint8).astype(np.float32)-127.5
    z=a[0::2]+1j*a[1::2]
    return np.abs(np.fft.fftshift(np.fft.fft(z*win)))**2
out=[]
for _ in range(n):
    t,k=struct.unpack("<dI",f.read(12)); b=f.read(k)
    if first: first=False; continue
    buf+=b
    while len(buf)>=2*N:
        P=block(bytes(buf[:2*N])); del buf[:2*N]; total+=N
        acc=P if acc is None else acc+P; nacc+=1
        if nacc==16:        # ~1 s of samples at 1.024 MS/s
            seg=acc[lo:hi]; j=int(np.argmax(seg)); jj=lo+j
            y0,y1,y2=np.log(acc[jj-1]+1e-9),np.log(acc[jj]+1e-9),np.log(acc[jj+1]+1e-9)
            d=0.5*(y0-y2)/(y0-2*y1+y2)
            out.append((float(total/fs),float((jj-N/2+d)*fs/N), float(acc[jj]/np.median(seg))))
            acc=None; nacc=0
import json
json.dump(out,open(sys.argv[2],"w"))
print("blocks",len(out))
