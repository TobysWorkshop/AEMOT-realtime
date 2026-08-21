import numpy as np
import argparse
import os
import struct
import matplotlib.pyplot as plt
import matplotlib.cm as cm

RECORD_DTYPE = np.dtype([
    ("track_id", "<u8"),
    ("ts",       "<f8"),
    ("x_hat",    "<f8", (10,)),
])
HEADER_SIZE = 24  # magic(8) + version(4) + n_state(4) + record_size(4) + reserved(4)

def load_kalman_log(path):
    """Returns {track_id: {"ts": np.ndarray, "x_hat": np.ndarray[N,10]}}."""
    with open(path, "rb") as f:
        f.seek(HEADER_SIZE)  # skip KalmanLogFileHeader
        records = np.fromfile(f, dtype=RECORD_DTYPE)

    tracks = {}
    for tid in np.unique(records["track_id"]):
        mask = records["track_id"] == tid
        tracks[int(tid)] = {
            "ts": records["ts"][mask],
            "x_hat": records["x_hat"][mask],
        }
    return tracks

def read_log(path):
    with open(path, "rb") as f:
        header_bytes = f.read(HEADER_SIZE)
        magic, version, n_state, record_size, reserved = struct.unpack("<8sIIII", header_bytes)
        print(f"Magic: {magic}, Version: {version}, n_state: {n_state}, record_size: {record_size}, reserved: {reserved}")

        if magic != b"AEMOTLOG":
            raise ValueError("Invalid magic number in log file - wrong file or format has changed...")

        if RECORD_DTYPE.itemsize != record_size:
            raise ValueError(f"Record size mismatch: expected {RECORD_DTYPE.itemsize}, got {record_size}")

        data = np.fromfile(f, dtype=RECORD_DTYPE)
        return data

def plot_tracks(data, background_image=None, output_path="temp.png", x_idx=0, y_idx=1, invert_y=True):
    track_ids = np.unique(data["track_id"])
    colors = cm.get_cmap("tab20", len(track_ids))

    fig, ax = plt.subplots(figsize=(10, 8))

    if background_image is not None:
        img = plt.imread(background_image)
        ax.imshow(img, cmap="gray" if img.ndim == 2 else None)
    elif invert_y:
        ax.invert_yaxis()

    for i, tid in enumerate(track_ids):
        mask = data["track_id"] == tid
        xs = data["x_hat"][mask, x_idx]
        ys = data["x_hat"][mask, y_idx]
        color = colors(i % colors.N) if len(track_ids) <= 20 else cm.hsv(i / len(track_ids))
        ax.plot(xs, ys, "-", linewidth=1, alpha=0.8, color=color)
        ax.plot(xs[0], ys[0], "o", markersize=4, color=color)
        ax.annotate(str(tid), (xs[-1], ys[-1]), fontsize=6, color=color)

    ax.set_title(f"{len(track_ids)} validated tracks")
    ax.set_xlabel("x (px)")
    ax.set_ylabel("y (px)")
    ax.set_aspect("equal", adjustable="box")
    fig.tight_layout()
    fig.savefig(output_path, dpi=300)
    print(f"Saved track plot to {output_path}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="name of the .bees log file inside the track_logs/ folder")
    parser.add_argument("--background", help="path to background image for plotting")   
    parser.add_argument("--output", help="path to save the plot image", default="temp.png")
    args = parser.parse_args()

    path = os.path.join(os.getcwd(), "track_logs", args.file)
    output_path = os.path.join(os.getcwd(), "track_logs", args.output)
    print(f"Loading Kalman log from {path}...")

    tracks = read_log(path)
    track_ids = np.unique(tracks["track_id"])
    for tid in track_ids:
        mask = tracks["track_id"] == tid
        n_records = mask.sum()
        print(f"Track {tid}: {n_records} records")
        # print(f"Track {tid}: {tracks['x_hat'][mask]}")
    print(f"[end] Total tracks: {len(track_ids)}")

    plot_tracks(tracks, background_image=args.background, output_path=output_path)

if __name__ == "__main__":
    main()