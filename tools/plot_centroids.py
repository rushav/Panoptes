#!/usr/bin/env python3
"""Quick look at a centroid_dump CSV: spatial scatter + area histogram."""
import sys
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv(sys.argv[1])

fig, ax = plt.subplots(1, 2, figsize=(14, 5))

sc = ax[0].scatter(df.x, df.y, c=df.area, s=4, cmap='viridis',
                   norm=plt.matplotlib.colors.LogNorm())
ax[0].invert_yaxis()
ax[0].set_aspect('equal')
ax[0].set(title='centroids (colour = area)', xlabel='x [px]', ylabel='y [px]')
plt.colorbar(sc, ax=ax[0])

ax[1].hist(df.area, bins=100, log=True)
ax[1].set(title='blob area', xlabel='area [px]', ylabel='count (log)')

plt.tight_layout()
plt.show()

print(f"frames      {df.frame_id.nunique()}")
print(f"blobs/frame {len(df) / df.frame_id.nunique():.1f}")
print(f"area > 100  {(df.area > 100).sum()}  ({(df.area > 100).sum() / len(df) * 100:.1f}%)")
