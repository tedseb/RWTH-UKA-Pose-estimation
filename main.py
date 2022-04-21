import numpy as np
import sys
from numpy import size
import pandas as pd
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.font_manager import FontProperties
import matplotlib.pyplot as plt

data = []
with open('sample.csv', 'r') as sample_file:
    for line in sample_file.readlines():
        line = line.replace("\n", "")
        line_data = line.split(",")
        for speed_index in range(6, 12):
            speed = float(line_data[speed_index])
            speed = f"{speed:.1f}ms / {1000/speed:.1f}fps"
            line_data[speed_index] = speed
        data.append(line_data)
#print(data)
data = sorted(data, key=lambda k: k[3])
smpl_model_index = 0
# for i, model_data in enumerate(data):
#     if model_data[0] == "metrabs_rn101_y4" and model_data[1] == "5":
#         smpl_model_index = i + 1
#sys.exit(0)
#plt.figure(figsize=(8, 6), dpi=80)
fig, ax = plt.subplots()
fig.set_size_inches(12, 6)
# hide axes
fig.patch.set_visible(False)
ax.axis('off')
ax.axis('tight')

columns = ["model", "num aug", "MPJPE", "MPJPE PA", "PCK", "Start T", "C1/S1", "C1/S2", "C2/S1", "C2/S2", "C4/S1", "C4/S2"]
cell_ratios = [1, 0.5, 0.5, 0.5, 0.5, 0.5, 1, 1, 1, 1, 1, 1]
df = pd.DataFrame(data, columns=columns)

size_divider = sum(cell_ratios)
for i in range(len(cell_ratios)):
    cell_ratios[i] = cell_ratios[i] / size_divider
table = ax.table(cellText=df.values, colLabels=df.columns, loc='center', colWidths=cell_ratios)
table.auto_set_font_size(False)
table.set_fontsize(8)
#table.scale(2, 4)

for (row, col), cell in table.get_celld().items():
    if (row == 0):
        cell.set_text_props(fontproperties=FontProperties(weight='bold', size=7 ))
        cell.set_facecolor("#9ffc9f")
        continue
    cell.set_text_props(ha="center")
    if (row % 2 == 0):
        cell.set_facecolor("#dbdbdb")
    if row == smpl_model_index:
        cell.set_facecolor("#ffef94")

fig.tight_layout()
plt.show()
pp = PdfPages("foo.pdf")
pp.savefig(fig, bbox_inches='tight')
pp.close()