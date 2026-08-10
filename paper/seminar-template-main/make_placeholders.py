#!/usr/bin/env python3
"""Generate placeholder figure images so the paper compiles before the real
photos exist. Replace the PNGs in figures/ with the real images, keeping the
file names, and nothing in content.tex has to change.
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

PLACEHOLDERS = [
    ("figures/teaser.png", 12.0, 3.6,
     "PLACEHOLDER — Fig. 1 (teaser, full width)",
     "Photo: the arm handing a filled glass to a seated person,\n"
     "or the arm mid-mime with the user's hand raised in frame.\n"
     "Should read as interaction, not as hardware."),
    ("figures/palm.png", 4.0, 3.0,
     "PLACEHOLDER — Fig. 3(a)",
     "Robot speaking; user\nraises an open palm."),
    ("figures/mime.png", 4.0, 3.0,
     "PLACEHOLDER — Fig. 3(b)",
     "Arm mimes a drink\n(coffee or water pose)."),
    ("figures/thumb.png", 4.0, 3.0,
     "PLACEHOLDER — Fig. 3(c)",
     "User answers with a\nthumbs-up; arm holds pose."),
    ("figures/sink.png", 5.6, 3.4,
     "PLACEHOLDER — Fig. 4",
     "Arm holding the glass under the tap,\nlab light visible if possible."),
]

for path, w, h, title, body in PLACEHOLDERS:
    fig, ax = plt.subplots(figsize=(w, h))
    ax.add_patch(Rectangle((0, 0), 1, 1, transform=ax.transAxes,
                           facecolor="0.93", edgecolor="0.45",
                           linewidth=1.6, linestyle="--"))
    ax.text(0.5, 0.68, title, ha="center", va="center", fontsize=11,
            color="0.25", weight="bold", transform=ax.transAxes)
    ax.text(0.5, 0.38, body, ha="center", va="center", fontsize=9.5,
            color="0.35", transform=ax.transAxes)
    ax.set_axis_off()
    fig.savefig(path, dpi=200, bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    print("wrote", path)
