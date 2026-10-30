"""Compare recorded PID variants; references reconstructed from current planner."""
from pathlib import Path
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from planner import planner, TRAJECTORY_PLANNER
ROOT=Path(__file__).resolve().parent/'Plots'; OUT=ROOT/'pid_versions'; OUT.mkdir(exist_ok=True)
def read(p,ns=False):
    with p.open() as f: rows=list(csv.reader(f))[1:]
    a=np.array([[float(v) for v in r if v.strip()] for r in rows if any(v.strip() for v in r)])
    assert np.isfinite(a).all()
    if ns:a[:,-1]/=1e9
    assert (np.diff(a[:,-1])>=0).all()
    return a
runs={}; metrics=[]
for shape in ('parabola','sigmoid'):
    ref=np.array(planner(TRAJECTORY_PLANNER,trajectory=shape).plan())
    for version in ((1,2,3,4) if shape == 'parabola' else (1,2)):
        folder=ROOT/shape/f'pid_{version}'
        if not folder.exists():folder=ROOT/shape/f'pid{version}'
        pose=read(folder/'robot_pose.csv',True);lin=read(next(folder.glob('*linear.csv')));ang=read(next(folder.glob('*angular.csv')))
        start=pose[0,-1]
        for a in (pose,lin,ang):a[:,-1]-=start
        seg=np.diff(ref,axis=0);off=pose[:,None,:2]-ref[None,:-1,:]
        u=np.clip(np.sum(off*seg,axis=2)/np.sum(seg**2,axis=1),0,1)
        err=np.linalg.norm(off-u[:,:,None]*seg,axis=2).min(axis=1)
        duration=pose[-1,-1]
        metrics.append([shape,f'PID {version}',duration,100*np.sqrt(np.trapz(err**2,pose[:,-1])/duration),100*err.max(),100*np.linalg.norm(pose[-1,:2]-ref[-1])])
        runs[shape,version]=(pose,lin,ang,ref,err)
plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
def save(fig,name):
    fig.tight_layout(rect=(0,0,1,.94));fig.savefig(OUT/f'{name}.png',dpi=170);fig.savefig(OUT/f'{name}.pdf');plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(12,5));fig.suptitle('PID run comparison — recorded paths',fontsize=17)
for ax,shape in zip(axes,('parabola','sigmoid')):
    for v,c,ls in ([(1,'#2563eb','-'),(2,'#dc2626','--'),(3,'#059669','-.'),(4,'#9333ea',':')] if shape == 'parabola' else [(1,'#2563eb','-'),(2,'#dc2626','--')]):
        pose,lin,ang,ref,err=runs[shape,v]
        ax.plot(pose[:,0],pose[:,1],color=c,ls=ls,label=f'PID {v}')
        ax.scatter(*pose[-1,:2],color=c,s=25)
    ax.plot(ref[:,0],ref[:,1],'k:',label='Current planner reference')
    ax.set(title=shape.capitalize(),xlabel='x (m)',ylabel='y (m)');ax.axis('equal');ax.grid(alpha=.25);ax.legend()
save(fig,'paths')
for name,titles in [('errors',['Path distance (cm)','Logged goal distance (m)','Logged lookahead heading error (rad)']),('states',['x (m)','y (m)','Heading (rad)']),('rates',['Goal-distance error rate (m/s)','Heading-error rate (rad/s)']),('phase',['Goal distance','Heading to lookahead'])]:
    fig,axes=plt.subplots(len(titles),2,figsize=(12,3*len(titles)),squeeze=False);fig.suptitle(f'PID run comparison — {name}',fontsize=17)
    for col,shape in enumerate(('parabola','sigmoid')):
        for v,c,ls in ([(1,'#2563eb','-'),(2,'#dc2626','--'),(3,'#059669','-.'),(4,'#9333ea',':')] if shape == 'parabola' else [(1,'#2563eb','-'),(2,'#dc2626','--')]):
            pose,lin,ang,ref,err=runs[shape,v]
            if name=='errors':series=[(pose[:,-1],100*err),(lin[:,-1],lin[:,0]),(ang[:,-1],ang[:,0])]
            elif name=='states':series=[(pose[:,-1],pose[:,i]) for i in range(3)]
            elif name=='rates':series=[(a[:,-1],a[:,1]) for a in (lin,ang)]
            else:series=[(a[:,0],a[:,1]) for a in (lin,ang)]
            for row,(x,y) in enumerate(series):axes[row,col].plot(x,y,color=c,ls=ls,label=f'PID {v}')
        for row,title in enumerate(titles):
            ax=axes[row,col];ax.set(title=f'{shape.capitalize()}: {title}',xlabel='Elapsed time (s)' if name!='phase' else ('Error (m)' if row==0 else 'Error (rad)'),ylabel=title if name!='phase' else ('Error rate (m/s)' if row==0 else 'Error rate (rad/s)'));ax.grid(alpha=.25);ax.legend()
    save(fig,name)
with (OUT/'metrics.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['Trajectory','Run','Recorded duration (s)','Time-weighted RMS path error (cm)','Maximum path error (cm)','Final goal error (cm)']);w.writerows(metrics)
for row in metrics:print(row)
