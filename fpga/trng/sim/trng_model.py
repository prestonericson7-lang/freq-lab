#!/usr/bin/env python3
"""
trng_model.py -- bit-exact reference model of the trng_core pipeline, matching
hdl/trng_core.v exactly. Used by check_trng.py to verify the hardware against
a known raw-bit stream.
"""


def rct(raw, cutoff):
    """Repetition Count Test. Returns (list of alarm indices, fail_count).
    Matches the HDL: counts repeats of the current value; on reaching `cutoff`
    in a row, raise one alarm and re-arm (so a long stuck burst counts once per
    `cutoff` bits)."""
    alarms = []
    have = False
    last = 0
    run = 1
    for i, b in enumerate(raw):
        if not have:
            have, last, run = True, b, 1
        elif b == last:
            run += 1
            if run >= cutoff:
                alarms.append(i)
                run = 1
                have = False
        else:
            last, run = b, 1
    return alarms, len(alarms)


def apt(raw, window, cutoff):
    """Adaptive Proportion Test. Fixed windows of `window` bits; the first bit of
    each window is the reference; if the running count equal to it reaches
    `cutoff`, raise an alarm (the HDL raises one alarm per sample that is at/above
    cutoff within a window, so count those)."""
    alarms = 0
    have = False
    ref = 0
    n = 0
    cnt = 0
    for b in raw:
        if not have:
            have, ref, n, cnt = True, b, 1, 1
        else:
            n += 1
            cnt += 1 if b == ref else 0
            if cnt >= cutoff:
                alarms += 1
            if n >= window:
                have = False
    return alarms


def von_neumann(raw, healthy_mask):
    """von Neumann debiaser on the bits where healthy_mask[i] is True (the HDL
    stops feeding VN while an alarm is latched). Pairs are formed only from the
    bits that pass through. 01->0, 10->1, 00/11->nothing. Returns the output bits."""
    out = []
    have = False
    first = 0
    for b, ok in zip(raw, healthy_mask):
        if not ok:
            continue
        if not have:
            have, first = True, b
        else:
            have = False
            if first != b:
                out.append(first)
    return out


def lfsr_words(vn_bits):
    """LFSR conditioning + 32-bit packer, matching the HDL exactly:
    x^32 + x^22 + x^2 + x + 1, feedback XOR the entropy bit, one word per 32 bits."""
    lfsr = 0xDEADBEEF
    words = []
    bitcnt = 0
    for vb in vn_bits:
        fb = ((lfsr >> 31) ^ (lfsr >> 21) ^ (lfsr >> 1) ^ lfsr ^ vb) & 1
        lfsr = ((lfsr << 1) | fb) & 0xFFFFFFFF
        if bitcnt == 31:
            bitcnt = 0
            words.append(lfsr)
        else:
            bitcnt += 1
    return words


def health_mask(raw, rct_cut, apt_win, apt_cut):
    """Per-bit 'healthy' flag: True while no alarm has been latched. An alarm
    latches from the bit that triggers it until cleared (the model never clears,
    matching a run with no CTRL clear). Used to gate von Neumann like the HDL."""
    rct_alarms, _ = rct(raw, rct_cut)
    first_rct = rct_alarms[0] if rct_alarms else None
    # APT: find the first bit index where cnt reaches cutoff
    first_apt = None
    have = False
    ref = n = cnt = 0
    for i, b in enumerate(raw):
        if not have:
            have, ref, n, cnt = True, b, 1, 1
        else:
            n += 1
            cnt += 1 if b == ref else 0
            if cnt >= apt_cut and first_apt is None:
                first_apt = i
            if n >= apt_win:
                have = False
    latch = None
    for c in (first_rct, first_apt):
        if c is not None:
            latch = c if latch is None else min(latch, c)
    # the triggering bit is still consumed (the alarm reg latches the next clock),
    # so bits with index <= latch are healthy and index > latch are gated.
    mask = [latch is None or i <= latch for i in range(len(raw))]
    return mask, first_rct, first_apt


def pipeline(raw, rct_cut=20, apt_win=1024, apt_cut=660):
    """Full model: returns a dict of everything the HDL exposes."""
    mask, first_rct, first_apt = health_mask(raw, rct_cut, apt_win, apt_cut)
    vn = von_neumann(raw, mask)
    words = lfsr_words(vn)
    _, rct_fc = rct(raw, rct_cut)
    apt_fc = apt(raw, apt_win, apt_cut)
    return dict(raw_count=len(raw), vn_count=len(vn), words=words,
                rct_fail=first_rct is not None, apt_fail=first_apt is not None,
                rct_fail_count=rct_fc, apt_fail_count=apt_fc,
                first_rct=first_rct, first_apt=first_apt)
