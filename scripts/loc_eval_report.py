#!/usr/bin/env python3
"""loc_eval.py が書き出した CSV を周回ごと・waypoint ごとに集計し、図を出す.

使い方:
  loc_eval_report.py <loc_eval_YYYYmmdd_HHMMSS ディレクトリ> [--waypoints <waypoint.json>] [--bag <rosbag ディレクトリ>]

出力 (同じディレクトリ):
  summary.txt        周回ごとの指標 (照合の失敗率、一致率、補正量、見失いの疑い)
  per_lap.csv        同じ内容の CSV
  scan_check.csv     (--bag 指定時) 停止中のスキャン同士の照合による、推定位置の周回間の誤差
  report.png         軌跡・一致率・補正量・高さと傾きの図

スキャン同士の照合 (--bag):
  停止 waypoint (type=stop) で止まっている間 (cmdvel_stopper の出力 /cmd_vel が 8 秒以上ゼロの区間) の
  点群を rosbag から取り出し、同じ waypoint の 1 回目の停止のスキャンと k 回目のスキャンを ICP で直接合わせる.
  停止位置に最も近い waypoint を割り当て、その waypoint での停止の順番を lap とする. 地図にも自己位置推定にも依らずに
  「1 周目と k 周目の停止位置が実際にどれだけ離れていたか」が求まるので、推定位置の差との
  食い違いがその周回での自己位置の誤差 (1 周目基準の再現性) になる. 巻尺での実測の代わり.
"""
import argparse
import csv
import json
import math
import os

import numpy as np



def read_csv(path):
    # 電源断などで末尾に NUL や書きかけの行が残ることがあるので、それらは読み飛ばす
    with open(path, newline='') as f:
        text = f.read().replace('\x00', '')
    rows = [r for r in csv.DictReader(text.splitlines()) if None not in r.values() and None not in r]
    out = {}
    for key in rows[0].keys() if rows else []:
        values = [r[key] for r in rows]
        try:
            out[key] = np.array([float(v) for v in values])
        except ValueError:
            out[key] = np.array(values)
    return out


def pct(a, q):
    a = a[np.isfinite(a)]
    return float(np.percentile(a, q)) if len(a) else float('nan')


def lap_summary(reg, events):
    rows = []
    for lap in np.unique(reg['lap']).astype(int):
        m = reg['lap'] == lap
        ev = events['lap'] == lap if events else np.zeros(0, bool)
        dist = reg['dist'][m]
        rows.append({
            'lap': lap,
            'n_reg': int(m.sum()),
            'dist_m': round(float(dist.max() - dist.min()), 2),
            'not_converged_pct': round(100 * float(np.mean(reg['converged'][m] == 0)), 2),
            'inlier0.2_mean': round(float(np.mean(reg['inlier_0.2'][m])), 3),
            'inlier0.2_p5': round(pct(reg['inlier_0.2'][m], 5), 3),
            'inlier0.2_min': round(float(np.min(reg['inlier_0.2'][m])), 3),
            'rmse_mean_m': round(float(np.nanmean(reg['rmse_in'][m])), 4),
            'jump_xy_p95_m': round(pct(reg['jump_xy'][m], 95), 3),
            'jump_xy_max_m': round(float(np.nanmax(reg['jump_xy'][m])), 3),
            'jump_yaw_max_deg': round(float(np.nanmax(reg['jump_yaw_deg'][m])), 2),
            'drift_xy_end_m': round(float(reg['drift_xy'][m][-1]), 3),
            'drift_yaw_end_deg': round(float(reg['drift_yaw_deg'][m][-1]), 2),
            'suspect_pct': round(100 * float(np.mean(reg['suspect'][m])), 2),
            'lost_events': int(np.sum(events['event'][ev] == 'lost_suspect')) if events else 0,
        })
    return rows


def write_rows(path, rows, fmt='{:.4f}'):
    if not rows:
        return
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow({k: fmt.format(v) if isinstance(v, float) else v for k, v in r.items()})


# --- 停止中のスキャン同士の照合 ---

SCAN_VOXEL = 0.05
SCAN_MAX_RANGE = 30.0
# 停止の前後は減速・発進で動いているので使わない [s]
STOP_MARGIN = 2.0
# cmdvel_stopper の出力がこの時間以上ゼロなら停止 waypoint で止まっていたとみなす [s]
MIN_STOP_DURATION = 8.0
# 停止位置から最も近い waypoint がこれより遠ければ、waypoint 以外での停止として扱う [m]
STOP_WP_MAX_DIST = 2.0


def pose_matrix(p, q):
    from scipy.spatial.transform import Rotation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def read_stop_scans(bag, windows, cloud_topic, pose_topic):
    """windows: {key: (t0, t1)} (bag の受信時刻). 各区間の点群 (センサ座標) と平均の姿勢を返す.

    RSF と PC の時計はずれることがあるので、header の時刻ではなく受信時刻で区切る.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic in (cloud_topic, pose_topic):
        if topic not in types:
            raise SystemExit(f'{bag} に {topic} がありません')
    pose_type = get_message(types[pose_topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[cloud_topic, pose_topic]))
    clouds = {k: [] for k in windows}
    poses = {k: [] for k in windows}
    while reader.has_next():
        topic, raw, recv_ns = reader.read_next()
        stamp = recv_ns * 1e-9
        key = next((k for k, (t0, t1) in windows.items() if t0 <= stamp <= t1), None)
        if key is None:
            continue
        if topic == cloud_topic:
            a = point_cloud2.read_points(deserialize_message(raw, PointCloud2),
                                         field_names=['x', 'y', 'z'], skip_nans=True)
            clouds[key].append(np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64))
        else:
            m = deserialize_message(raw, pose_type)
            pose = m.pose.pose if hasattr(m.pose, 'pose') else m.pose
            poses[key].append(pose_matrix(pose.position, pose.orientation))
    out = {}
    for key in windows:
        if not clouds[key] or not poses[key]:
            continue
        pts = np.vstack(clouds[key])
        pts = pts[np.linalg.norm(pts, axis=1) < SCAN_MAX_RANGE]
        T = poses[key][len(poses[key]) // 2].copy()
        T[:3, 3] = np.mean([p[:3, 3] for p in poses[key]], axis=0)
        # 停止中に推定位置が揺れていないか (照合の補正で跳ぶと大きくなる)
        spread = float(np.ptp([p[:3, 3] for p in poses[key]], axis=0)[:2].max())
        out[key] = {'points': voxel(pts, SCAN_VOXEL), 'pose': T, 'n_clouds': len(clouds[key]),
                    'pose_spread': spread}
    return out


def voxel(points, size):
    keys = np.floor(points / size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[idx]


def icp_point_to_plane(src, tgt, T0, iterations=40):
    """src を tgt に合わせる 4x4 変換を返す. 戻り値: T, 0.1 m 以内の割合, その RMSE."""
    from scipy.spatial import cKDTree
    from scipy.spatial.transform import Rotation
    tree = cKDTree(tgt)
    _, nn = tree.query(tgt, k=10, workers=-1)
    nb = tgt[nn] - tgt[nn].mean(axis=1, keepdims=True)
    normals = np.linalg.eigh(np.einsum('nki,nkj->nij', nb, nb))[1][:, :, 0]
    T = T0.copy()
    max_dist = 1.0
    for _ in range(iterations):
        X = src @ T[:3, :3].T + T[:3, 3]
        d, i = tree.query(X, distance_upper_bound=max_dist, workers=-1)
        ok = np.isfinite(d)
        if ok.sum() < 100:
            break
        x, q, n = X[ok], tgt[i[ok]], normals[i[ok]]
        r = np.einsum('ij,ij->i', n, x - q)
        J = np.hstack([np.cross(x, n), n])
        delta = np.linalg.solve(J.T @ J + 1e-6 * np.eye(6), -J.T @ r)
        dT = np.eye(4)
        dT[:3, :3] = Rotation.from_rotvec(delta[:3]).as_matrix()
        dT[:3, 3] = delta[3:]
        T = dT @ T
        max_dist = max(0.2, max_dist * 0.8)
        if np.linalg.norm(delta[3:]) < 1e-4 and np.linalg.norm(delta[:3]) < 1e-5:
            break
    d, _ = tree.query(src @ T[:3, :3].T + T[:3, 3], distance_upper_bound=0.1, workers=-1)
    ok = np.isfinite(d)
    return T, float(ok.mean()), float(np.sqrt(np.mean(d[ok] ** 2))) if ok.any() else float('nan')


def yaw_of(T):
    return math.atan2(T[1, 0], T[0, 0])


def detect_stops(bag, cmd_topic):
    """cmd_topic (cmdvel_stopper の出力) が MIN_STOP_DURATION 以上ゼロだった区間 (受信時刻) を返す.

    waypoint_manager のログ (/rosout) は取りこぼすことがあるので、実際の速度指令で判定する.
    """
    import rosbag2_py
    from geometry_msgs.msg import Twist
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    if cmd_topic not in {t.name for t in reader.get_all_topics_and_types()}:
        raise SystemExit(f'{bag} に {cmd_topic} がありません')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[cmd_topic]))
    stops = []
    start = last = None
    while reader.has_next():
        _, raw, recv_ns = reader.read_next()
        m = deserialize_message(raw, Twist)
        t = recv_ns * 1e-9
        zero = abs(m.linear.x) < 1e-4 and abs(m.angular.z) < 1e-4
        if zero and start is None:
            start = t
        elif not zero and start is not None:
            stops.append((start, last))
            start = None
        last = t
    if start is not None:
        stops.append((start, last))
    return [(t0, t1) for t0, t1 in stops if t1 - t0 >= MIN_STOP_DURATION]


def scan_check(bag, waypoints, cloud_topic, pose_topic, cmd_topic):
    """停止ごとに、同じ waypoint での最初の停止のスキャンに対する推定位置の誤差を求める."""
    stops = detect_stops(bag, cmd_topic)
    windows = {i: (t0 + STOP_MARGIN, t1 - STOP_MARGIN) for i, (t0, t1) in enumerate(stops)}
    found = read_stop_scans(bag, windows, cloud_topic, pose_topic)
    # 停止位置に最も近い waypoint を割り当て、同じ waypoint での出現順を周回番号にする
    scans = {}
    count = {}
    for i in sorted(found):
        xy = found[i]['pose'][:2, 3]
        wp = -1
        if waypoints:
            d = [math.hypot(xy[0] - x, xy[1] - y) for x, y in waypoints]
            if min(d) <= STOP_WP_MAX_DIST:
                wp = int(np.argmin(d))
        count[wp] = count.get(wp, 0) + 1
        found[i]['t0'] = stops[i][0]
        scans[(count[wp], wp)] = found[i]
    rows = []
    plot_data = {}
    for wp in sorted({k[1] for k in scans}):
        keys = sorted(k for k in scans if k[1] == wp)
        ref = scans[keys[0]]
        for key in keys:
            cur = scans[key]
            E1, Ek = ref['pose'], cur['pose']
            # 推定どおりなら k 周目のセンサ姿勢は 1 周目のセンサ座標で E1^-1 Ek. ICP で実際の値に直す
            T, fitness, rmse = icp_point_to_plane(cur['points'], ref['points'], np.linalg.inv(E1) @ Ek)
            true_k = E1 @ T
            err = Ek[:3, 3] - true_k[:3, 3]
            plot_data[key] = {'points': cur['points'], 'est': Ek, 'true': true_k}
            rows.append({
                'wp': wp, 'lap': key[0], 'ref_lap': keys[0][0], 'n_clouds': cur['n_clouds'],
                'est_dx': float(Ek[0, 3] - E1[0, 3]), 'est_dy': float(Ek[1, 3] - E1[1, 3]),
                'scan_dx': float(true_k[0, 3] - E1[0, 3]), 'scan_dy': float(true_k[1, 3] - E1[1, 3]),
                'err_x': float(err[0]), 'err_y': float(err[1]), 'err_xy': float(math.hypot(err[0], err[1])),
                'err_yaw_deg': math.degrees((yaw_of(Ek) - yaw_of(true_k) + math.pi) % (2 * math.pi) - math.pi),
                'icp_fitness': fitness, 'icp_rmse': rmse, 'pose_spread': cur['pose_spread'],
                'stop_time': cur['t0'],
            })
    return rows, plot_data


# 1 周目は基準なので灰色. 2 周目以降は決まった順の色 (周回数が増えても既存の周の色は変えない)
REF_COLOR = '#8a8984'
LAP_COLORS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#6250d6', '#e34948']
TEXT = '#0b0b0b'
TEXT_SECONDARY = '#52514e'
GRID = '#e4e3df'
PLOT_HALF_WIDTH = 8.0
PLOT_MAX_POINTS = 15000


def lap_color(lap, ref_lap):
    return REF_COLOR if lap == ref_lap else LAP_COLORS[(lap - ref_lap - 1) % len(LAP_COLORS)]


def plot_scan_check(run_dir, rows, plot_data):
    """停止ごとに「推定位置で重ねたスキャン」「推定位置と実際の位置」「周回ごとの誤差」を描く."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    wps = sorted({r['wp'] for r in rows})
    fig, axes = plt.subplots(len(wps), 3, figsize=(17, 5.6 * len(wps)), squeeze=False,
                             gridspec_kw={'width_ratios': [1.25, 1, 1]})
    fig.patch.set_facecolor('#fcfcfb')
    rng = np.random.default_rng(0)
    for ax_row, wp in zip(axes, wps):
        wp_rows = sorted((r for r in rows if r['wp'] == wp), key=lambda r: r['lap'])
        ref_lap = wp_rows[0]['ref_lap']
        center = plot_data[(ref_lap, wp)]['est'][:3, 3]

        # 1) 推定位置で地図座標に置いたスキャン (自己位置が正しければ壁が重なる)
        a = ax_row[0]
        # 基準の周を最後に (一番上に) 描き、他の周の壁がそこからずれているかを見えるようにする
        for r in wp_rows[1:] + wp_rows[:1]:
            d = plot_data[(r['lap'], wp)]
            pts = d['points']
            # 床と天井を除き、センサ高さ付近の壁だけを上から見る
            pts = pts[(pts[:, 2] > -0.3) & (pts[:, 2] < 1.0)]
            if len(pts) > PLOT_MAX_POINTS:
                pts = pts[rng.choice(len(pts), PLOT_MAX_POINTS, replace=False)]
            X = pts @ d['est'][:3, :3].T + d['est'][:3, 3]
            is_ref = r['lap'] == ref_lap
            a.scatter(X[:, 0], X[:, 1], s=0.4 if is_ref else 1.2, color=lap_color(r['lap'], ref_lap),
                      linewidths=0, alpha=0.9, zorder=3 if is_ref else 2, rasterized=True,
                      label=f'lap {r["lap"]} (reference)' if is_ref else f'lap {r["lap"]}')
        a.set_xlim(center[0] - PLOT_HALF_WIDTH, center[0] + PLOT_HALF_WIDTH)
        a.set_ylim(center[1] - PLOT_HALF_WIDTH, center[1] + PLOT_HALF_WIDTH)
        a.set_aspect('equal')
        a.set_title(f'WP{wp}: scans placed by estimated pose (walls overlap if localization is right)',
                    fontsize=10, color=TEXT, loc='left')
        a.set_xlabel('map x [m]')
        a.set_ylabel('map y [m]')
        a.legend(markerscale=12, fontsize=8, loc='upper right', frameon=False)

        # 2) 推定位置 (丸) と、スキャン照合で求めた実際の位置 (×). 線の長さが誤差
        a = ax_row[1]
        for r in wp_rows:
            d = plot_data[(r['lap'], wp)]
            c = lap_color(r['lap'], ref_lap)
            e = (d['est'][:2, 3] - center[:2]) * 100
            t = (d['true'][:2, 3] - center[:2]) * 100
            a.plot([t[0], e[0]], [t[1], e[1]], '-', color=c, lw=2)
            a.plot(*t, 'x', color=c, ms=10, mew=2)
            a.plot(*e, 'o', color=c, ms=9, mec='#fcfcfb', mew=2)
            a.annotate(f'lap {r["lap"]}', e, textcoords='offset points', xytext=(8, 6), fontsize=8,
                       color=TEXT_SECONDARY)
        lim = max(10.0, 1.3 * max(np.abs((plot_data[(r['lap'], wp)][k][:2, 3] - center[:2]) * 100).max()
                                  for r in wp_rows for k in ('est', 'true')))
        a.set_xlim(-lim, lim)
        a.set_ylim(-lim, lim)
        a.set_aspect('equal')
        a.plot([], [], 'o', color=TEXT_SECONDARY, label='estimated position')
        a.plot([], [], 'x', color=TEXT_SECONDARY, mew=2, label='actual position (scan-to-scan ICP)')
        a.legend(fontsize=8, loc='upper right', frameon=False)
        a.set_title(f'WP{wp}: stop positions relative to lap {ref_lap} estimate', fontsize=10, color=TEXT, loc='left')
        a.set_xlabel('map x [cm]')
        a.set_ylabel('map y [cm]')

        # 3) 周回ごとの位置誤差. 方位誤差と ICP の fitness はラベルで添える
        a = ax_row[2]
        laps = [r['lap'] for r in wp_rows]
        err = [r['err_xy'] * 100 for r in wp_rows]
        a.bar(range(len(laps)), err, width=0.6, color=[lap_color(l, ref_lap) for l in laps])
        for i, r in enumerate(wp_rows):
            if r['lap'] == ref_lap:
                a.annotate('reference', (i, 0), textcoords='offset points', xytext=(0, 4), ha='center',
                           va='bottom', fontsize=8, color=TEXT_SECONDARY)
                continue
            note = f'{r["err_xy"] * 100:.1f} cm\n{r["err_yaw_deg"]:+.2f}°\nfit {r["icp_fitness"]:.2f}'
            if r['icp_fitness'] < 0.5:
                note += '\n(unreliable)'
            a.annotate(note, (i, err[i]), textcoords='offset points', xytext=(0, 4), ha='center',
                       va='bottom', fontsize=8, color=TEXT if r['icp_fitness'] >= 0.5 else TEXT_SECONDARY)
        a.set_xticks(range(len(laps)), [f'lap {l}' for l in laps])
        a.set_ylim(0, max(5.0, max(err) * 1.6))
        a.set_ylabel('position error [cm]')
        a.set_title(f'WP{wp}: localization error per lap (vs lap {ref_lap})', fontsize=10, color=TEXT, loc='left')

        for a in ax_row:
            a.set_facecolor('#fcfcfb')
            a.grid(color=GRID, lw=0.6)
            a.set_axisbelow(True)
            for side in ('top', 'right'):
                a.spines[side].set_visible(False)
            for side in ('left', 'bottom'):
                a.spines[side].set_color(GRID)
            a.tick_params(colors=TEXT_SECONDARY, labelsize=8)
            a.xaxis.label.set_color(TEXT_SECONDARY)
            a.yaxis.label.set_color(TEXT_SECONDARY)
    fig.tight_layout()
    path = os.path.join(run_dir, 'scan_check.png')
    fig.savefig(path, dpi=110, facecolor=fig.get_facecolor())
    plt.close(fig)
    return path

def plot(run_dir, reg, events, waypoints):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    laps = np.unique(reg['lap']).astype(int)
    colors = plt.cm.tab10(np.arange(len(laps)) % 10)
    fig, ax = plt.subplots(3, 2, figsize=(14, 13))

    a = ax[0, 0]
    for lap, c in zip(laps, colors):
        m = reg['lap'] == lap
        a.plot(reg['est_x'][m], reg['est_y'][m], '-', color=c, lw=1, label=f'lap {lap}')
    s = reg['suspect'] == 1
    a.plot(reg['est_x'][s], reg['est_y'][s], 'rx', ms=5, label='suspect')
    if waypoints:
        wx, wy = zip(*waypoints)
        a.plot(wx, wy, 'ks', mfc='none', ms=9, label='waypoint')
        for i, (x, y) in enumerate(waypoints):
            a.annotate(str(i), (x, y), textcoords='offset points', xytext=(5, 5))
    a.set_aspect('equal')
    a.set_xlabel('x [m]')
    a.set_ylabel('y [m]')
    a.set_title('estimated trajectory (map)')
    a.legend(fontsize=8)
    a.grid(alpha=0.3)

    def series(a, keys, title, ylabel):
        for key in keys:
            a.plot(reg['dist'], reg[key], lw=0.8, label=key)
        for i in np.where(np.diff(reg['lap']) != 0)[0]:
            a.axvline(reg['dist'][i + 1], color='gray', ls='--', lw=0.8)
        if events:
            lost = events['event'] == 'lost_suspect'
            for d in events['dist'][lost]:
                a.axvline(d, color='r', lw=1)
        a.set_xlabel('distance [m]  (dashed: lap boundary, red: lost suspect)')
        a.set_ylabel(ylabel)
        a.set_title(title)
        a.legend(fontsize=8)
        a.grid(alpha=0.3)

    series(ax[0, 1], ['inlier_0.1', 'inlier_0.2', 'inlier_0.3'], 'scan-to-map inlier ratio', 'ratio')
    ax[0, 1].set_ylim(0, 1.05)
    series(ax[1, 0], ['jump_xy'], 'correction per registration (map->odom change)', 'm')
    series(ax[1, 1], ['jump_yaw_deg'], 'correction per registration (yaw)', 'deg')
    series(ax[2, 0], ['drift_xy', 'drift_yaw_deg'], 'accumulated LIO drift absorbed by map matching', 'm / deg')
    series(ax[2, 1], ['est_z', 'est_roll_deg', 'est_pitch_deg'], 'estimated height and tilt', 'm / deg')
    fig.tight_layout()
    path = os.path.join(run_dir, 'report.png')
    fig.savefig(path, dpi=110)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('run_dir')
    parser.add_argument('--waypoints', help='waypoint の JSON (図に重ねる)')
    parser.add_argument('--bag', help='同じ走行の rosbag (loc_eval_record.sh). 停止中のスキャン同士を照合する')
    parser.add_argument('--cloud-topic', default='/hokuyo3d/hokuyo_cloud2')
    parser.add_argument('--pose-topic', default='/estimated_pose')
    parser.add_argument('--cmd-topic', default='/cmd_vel', help='停止の判定に使う速度指令 (cmdvel_stopper の出力)')
    args = parser.parse_args()

    reg = read_csv(os.path.join(args.run_dir, 'registration.csv'))
    if not reg:
        raise SystemExit('registration.csv が空です. /loc_registered_cloud を受信できていません')
    events_path = os.path.join(args.run_dir, 'events.csv')
    events = read_csv(events_path) if os.path.exists(events_path) else {}
    waypoints = []
    if args.waypoints:
        with open(args.waypoints) as f:
            waypoints = [(w[0][0], w[0][1]) for w in json.load(f)]

    laps = lap_summary(reg, events)
    write_rows(os.path.join(args.run_dir, 'per_lap.csv'), laps)
    scans, scan_plot = scan_check(args.bag, waypoints, args.cloud_topic, args.pose_topic, args.cmd_topic) if args.bag else ([], {})
    write_rows(os.path.join(args.run_dir, 'scan_check.csv'), scans)

    lines = [f'run: {args.run_dir}',
             f'registrations: {len(reg["lap"])}, distance: {reg["dist"][-1]:.1f} m, laps: {len(laps)}',
             '', '[per lap]']
    keys = list(laps[0].keys())
    lines.append(' '.join(f'{k:>16}' for k in keys))
    for r in laps:
        lines.append(' '.join(f'{r[k]:>16}' for k in keys))
    if scans:
        lines += ['', '[scan check: estimated pose error relative to the first lap (scan-to-scan ICP at stops)]',
                  f'{"wp":>4} {"lap":>4} {"est_dx":>7} {"est_dy":>7} {"scan_dx":>7} {"scan_dy":>7} '
                  f'{"err_x":>7} {"err_y":>7} {"err_xy":>7} {"err_yaw":>7} {"fitness":>7} {"rmse":>6}']
        for r in scans:
            lines.append(f'{r["wp"]:>4} {r["lap"]:>4} {r["est_dx"]:7.3f} {r["est_dy"]:7.3f} {r["scan_dx"]:7.3f} '
                         f'{r["scan_dy"]:7.3f} {r["err_x"]:+7.3f} {r["err_y"]:+7.3f} {r["err_xy"]:7.3f} '
                         f'{r["err_yaw_deg"]:+7.2f} {r["icp_fitness"]:7.3f} {r["icp_rmse"]:6.3f}')
        later = [r for r in scans if r['lap'] != r['ref_lap'] and r['icp_fitness'] >= 0.5]
        if later:
            e = np.array([r['err_xy'] for r in later])
            y = np.abs([r['err_yaw_deg'] for r in later])
            lines.append(f'2 周目以降: err_xy mean={e.mean():.3f} max={e.max():.3f} m, '
                         f'|err_yaw| mean={y.mean():.2f} max={y.max():.2f} deg (n={len(later)}, fitness>=0.5)')
        lines.append('fitness: 停止中スキャン同士の 0.1 m 以内の割合. 0.5 未満は照合が怪しいので誤差の値は参考扱い')
        lines.append(f'figure: {plot_scan_check(args.run_dir, scans, scan_plot)}')
    elif not args.bag:
        lines += ['', '--bag に同じ走行の rosbag を指定すると、停止中のスキャン同士の照合で周回間の誤差を求めます']
    lines += ['', f'figure: {plot(args.run_dir, reg, events, waypoints)}']

    text = '\n'.join(lines)
    with open(os.path.join(args.run_dir, 'summary.txt'), 'w') as f:
        f.write(text + '\n')
    print(text)


if __name__ == '__main__':
    main()
