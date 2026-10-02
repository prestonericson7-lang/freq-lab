import numpy as np, sys
for name in sys.argv[1:]:
    t = np.genfromtxt(f'truth_{name}.csv', delimiter=',', names=True)
    m = np.genfromtxt(f'tracked_{name}.csv', delimiter=',', names=True)
    dth = t['theta'] - t['theta'][0]
    dm = m['angle'] - m['angle'][0]
    s = np.sign(np.dot(dm, dth)) or 1
    err = s * dm - dth
    big = np.abs(dth).max()
    print(f"{name:6s} frames {len(m)}  rotor_ok {int(m['rotor_ok'].sum())}/{len(m)}  ref_ok {int(m['ref_ok'].sum())}/{len(m)}"
          f"  acquisitions {int(m['acq'][-1])}  swing up to {big*1e3:.0f} mrad  error rms {np.std(err)*1e3:.3f} mrad,"
          f" worst {np.abs(err - err.mean()).max()*1e3:.3f} mrad, scale {np.polyfit(dth, s*dm, 1)[0]:.4f}")
