from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

for csv_file in Path(".").glob("stats_*.csv"):

    df = pd.read_csv(csv_file)

    cols = df.columns[-4:]
    corr = df[cols].corr(method="pearson")

    print(f"\n=== {csv_file.name} ===")
    print(corr)

    # =========================
    # NEW: stats summary
    # =========================
    print("\nSummary statistics:")
    print(f"Number of points: {len(df)}")

    for c in cols:
        mean = df[c].mean()
        std = df[c].std()
        print(f"{c}: mean={mean:.4f}, std={std:.4f}")

    # =========================
    # sorted output (unchanged)
    # =========================
    df_sorted = df.sort_values(
        by=["sum_relevance", "sum_distance1", "sum_distance2", "stdev"],
        ascending=[False, False, False, True]
    )

    suffix = csv_file.stem.removeprefix("stats_")
    sorted_filename = f"sorted_{suffix}.csv"
    df_sorted.to_csv(sorted_filename, index=False)

    print(f"\nSorted data written to {sorted_filename}")




# =========================
# NEW: stacked histograms (all files in one figure)
# =========================

param_ranges = {
    cols[0]: (100, 300),
    cols[1]: (70, 300),
    cols[2]: (100, 700),
}

files = list(Path(".").glob("stats_*.csv"))

fig, axes = plt.subplots(
    len(files), 3,
    figsize=(15, 3 * len(files)),
    squeeze=False
)

for i, csv_file in enumerate(files):

    df = pd.read_csv(csv_file)
    suffix = csv_file.stem.removeprefix("stats_")

    for j, col in enumerate(cols[:3]):

        ax = axes[i][j]

        lo, hi = param_ranges[col]
        data = df[col].dropna()

        ax.hist(data, bins=30, range=(lo, hi), alpha=0.75)

        if i == 0:
            ax.set_title(col)

        if j == 0:
            ax.set_ylabel(suffix)

        ax.set_xlim(lo, hi)

plt.tight_layout()

fig_filename = "hist_all_conditions.png"
plt.savefig(fig_filename, dpi=150)
plt.close()

print(f"Histogram saved to {fig_filename}")