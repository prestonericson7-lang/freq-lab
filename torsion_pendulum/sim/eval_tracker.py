import numpy as np, sys
t=np.genfromtxt('truth.csv',delimiter=',',names=True)
m=np.genfromtxt(sys.argv[1],delimiter=',',names=True)
n=len(t)
print("  rotor_ok all:", bool(m['rotor_ok'].all()), " ref_ok all:", bool(m['ref_ok'].all()), " acquisitions:", int(m['acq'][-1]))
dth = t['theta']-t['theta'][0]
roll_at = 3*n//4
for name in ['rotor_angle','angle']:
    dm = m[name]-m[name][0]
    s = np.sign(np.dot(dm,dth)) or 1
    err = s*dm - dth
    print(f"  {name:12s} sign {s:+.0f}  error rms {np.std(err[:roll_at])*1e3:.4f} mrad, mean {np.mean(err[:roll_at])*1e3:+.4f} "
          f"| after the 2 mrad camera roll: mean err {np.mean(err[roll_at+2:])*1e3:+.4f} mrad")
dm=m['angle']-m['angle'][0]; s=np.sign(np.dot(dm,dth))
a,b,c=n//4,n//2,roll_at
step = np.mean(s*dm[b:c]) - np.mean(s*dm[a:b]); true_step=np.mean(dth[b:c])-np.mean(dth[a:b])
print(f"  10 mrad step measured {step*1e3:.4f} mrad (truth {true_step*1e3:.4f})")
