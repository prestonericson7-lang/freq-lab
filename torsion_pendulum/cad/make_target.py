#!/usr/bin/env python3
"""make_target.py -- the paper disc glued to the base's floor (two per US Letter page).

Print at 100 % / "Actual size" on plain white paper; the 50 mm bar must measure 50 mm.
The disc is 95 mm (the floor is 97), with the two squares at +/-30 mm so their
tracking boxes stay inside the 93 mm backlit window.
"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

W_MM, H_MM = 215.9, 279.4                      # US Letter
CUT_D = 95.0                                   # the base's floor is 97 mm across
SQ = 8.0                                       # reference squares
SQ_X = 30.0                                    # at +/-30 mm along the line that matches the lid arrow

fig = plt.figure(figsize=(W_MM / 25.4, H_MM / 25.4))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W_MM)
ax.set_ylim(0, H_MM)
ax.set_aspect("equal")
ax.axis("off")

for cy in (H_MM * 0.73, H_MM * 0.30):
    cx = W_MM / 2
    ax.add_patch(Circle((cx, cy), CUT_D / 2, fill=False, lw=0.6, ec="0.55", ls=(0, (4, 3))))
    for sx in (-1, 1):
        ax.add_patch(Rectangle((cx + sx * SQ_X - SQ / 2, cy - SQ / 2), SQ, SQ, fc="black", ec="none"))
    ax.plot([cx - 1.5, cx + 1.5], [cy, cy], color="0.8", lw=0.4)          # faint centre cross
    ax.plot([cx, cx], [cy - 1.5, cy + 1.5], color="0.8", lw=0.4)
    # notes outside the cut line only (inside the disc everything except the squares stays white)
    ax.annotate("", xy=(cx + 66, cy), xytext=(cx + 54, cy), arrowprops=dict(arrowstyle="-|>", color="0.35", lw=0.8))
    ax.text(cx + 68, cy, "squares line up\nwith the lid's arrow", va="center", fontsize=7, color="0.35")
    ax.text(cx, cy - CUT_D / 2 - 5, "cut on the dashed circle (95 mm); glue-stick it to the base's floor, squares along the arrow",
            ha="center", fontsize=7, color="0.35")

# scale check
ax.plot([20, 70], [12, 12], color="black", lw=1.2)
for x in (20, 70):
    ax.plot([x, x], [10, 14], color="black", lw=1.2)
ax.text(20, 15, "50 mm - measure this; if it is not 50 mm, print again at 100 % / Actual size", ha="left",
        fontsize=7)
ax.text(W_MM / 2, H_MM - 10, "Torsion pendulum paper target (freq-lab #2)", ha="center", fontsize=9)
fig.savefig("out/paper_target.pdf")
fig.savefig("out/paper_target.png", dpi=60)
print("wrote out/paper_target.pdf")
