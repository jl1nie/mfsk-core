# usage: python3 -I scripts/rtltcp/clock_log.py rsp_clock.log [NOMINAL]  (rsp_tcp dbg3, see docs/notes/RTLTCP_RATE.md)
# Server-side clock log: samples against the server's own UTC, per second.
import sys, statistics as st
ft0 = 116444736000000000   # 100 ns from 1601 to 1970
rows=[]
for l in open(sys.argv[1]):
    p=l.split()
    if len(p)==6 and p[0]=="C":
        rows.append((int(p[1]),int(p[2]),int(p[3]),int(p[4]),int(p[5])))
print("records",len(rows))
t=[(r[0]-ft0)/1e7 for r in rows]; n=[r[1] for r in rows]
print("span %.1f s, samples %d, mean rate %.0f S/s (nominal %s)"%(t[-1]-t[0],n[-1]-n[0],(n[-1]-n[0])/(t[-1]-t[0]),sys.argv[2] if len(sys.argv)>2 else "?"))
resets=[(round(t[i]-t[0],2),rows[i][3]) for i in range(len(rows)) if rows[i][3]]
print("reset records:",len(resets),resets[:10])
# rate per 10 s
k=0
print("per 10 s: wall-time rate (S/s)")
while k<len(t):
    j=k
    while j<len(t) and t[j]-t[k]<10: j+=1
    if j>=len(t): break
    print("  %6.0f s  %9.0f"%(t[k]-t[0],(n[j]-n[k])/(t[j]-t[k])))
    k=j
# gaps between records longer than 80 ms
gaps=[(round(t[i]-t[0],2),round((t[i]-t[i-1])*1e3)) for i in range(1,len(t)) if t[i]-t[i-1]>0.08]
print("record gaps >80 ms:",len(gaps),gaps[:10])
