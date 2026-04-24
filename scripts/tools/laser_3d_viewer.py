#!/usr/bin/env python3
"""
Laser 3D Surface Viewer — Ray-tracing 모델 기반 3D 형상 시각화

Usage:
    python scripts/tools/laser_3d_viewer.py [csv_path]

기본값: data/laser_scan/calibration/vertical/scan_raw_20260321_195848.csv

모델: d = fy*K / (cy + fy*tan(α) - y_px), depth = d0 - d
파라미터: config/laser/vertical/laser_vertical_triangulation_calib.json
Updated: 2026-04-24
"""

import sys
import os
import json
import numpy as np
import pandas as pd


def load_calib(project_root):
    """캘리브레이션 config 로드"""
    calib_path = os.path.join(
        project_root, 'config/laser/vertical/laser_vertical_triangulation_calib.json'
    )
    with open(calib_path, 'r') as f:
        calib = json.load(f)

    p = calib['parameters']
    alpha_rad = np.radians(p['alpha_deg'])
    K = p['h_mm'] + p['By_mm'] * np.tan(alpha_rad)

    return {
        'fy': calib['camera']['fy_px'],
        'cy': calib['camera']['cy_px'],
        'alpha_rad': alpha_rad,
        'K': K,
        'd0': p['d0_mm'],
        'C': calib['camera']['cy_px'] + calib['camera']['fy_px'] * np.tan(alpha_rad),
    }


def load_and_compute(csv_path, cal):
    """CSV 로드 → ray-tracing 역모델로 3D 좌표 계산"""
    df = pd.read_csv(csv_path)
    laser = df[df['roi_name'] == 'roi_laser'].copy()

    # 베이스라인 (기준면 y_px)
    base = df[df['roi_name'] == 'roi_laser_base(R)']
    base_mean = base.groupby('capture_idx')['center_y_px'].mean()
    laser['y_floor'] = laser['capture_idx'].map(base_mean)

    fy, cy = cal['fy'], cal['cy']
    K, C, d0 = cal['K'], cal['C'], cal['d0']

    # Ray-tracing inverse: y_px → surface distance d
    denom = C - laser['center_y_px']
    denom = denom.clip(lower=0.1)  # 특이점 보호
    laser['d_surface'] = fy * K / denom

    # 3D 좌표
    cx = 960.0
    laser['X'] = (laser['col_px'] - cx) * laser['d_surface'] / fy
    laser['Y'] = laser['z_tcp_mm']
    laser['Z'] = d0 - laser['d_surface']  # depth = protrusion from wall

    # 이상치 제거
    laser = laser[(laser['Z'] > -10) & (laser['Z'] < 80)].copy()

    return laser


def show_3d(laser):
    """PyVista 인터랙티브 3D 뷰어"""
    try:
        import pyvista as pv
    except ImportError:
        print("pyvista 미설치 → matplotlib fallback")
        show_3d_matplotlib(laser)
        return

    points = laser[['X', 'Y', 'Z']].values
    cloud = pv.PolyData(points)
    cloud['depth_mm'] = laser['Z'].values

    surf = cloud.delaunay_2d(alpha=5.0)

    pl = pv.Plotter(window_size=(1400, 900))
    pl.set_background('white')
    pl.add_mesh(surf, scalars='depth_mm', cmap='turbo',
                show_edges=False, opacity=0.95,
                scalar_bar_args={'title': 'Depth (mm)', 'color': 'black'})
    pl.add_axes(color='black')
    pl.add_title("Laser 3D — Ray-tracing Model (v3)", color='black', font_size=14)
    pl.camera_position = 'iso'
    pl.show()


def show_3d_matplotlib(laser):
    """Matplotlib 인터랙티브 3D 뷰어"""
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from scipy.interpolate import griddata

    X = laser['X'].values
    Y = laser['Y'].values
    Z = laser['Z'].values

    xi = np.linspace(X.min(), X.max(), 250)
    yi = np.linspace(Y.min(), Y.max(), 120)
    Xi, Yi = np.meshgrid(xi, yi)
    Zi = griddata((X, Y), Z, (Xi, Yi), method='linear')

    fig = plt.figure(figsize=(14, 9))
    ax = fig.add_subplot(111, projection='3d')

    surf = ax.plot_surface(Xi, Yi, Zi, cmap='turbo', alpha=0.9,
                           rstride=1, cstride=1, linewidth=0,
                           antialiased=True)

    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Z_tcp (mm)')
    ax.set_zlabel('Depth (mm)')
    ax.set_title('3D Surface — Ray-tracing Model (v3)')
    ax.view_init(elev=35, azim=-50)

    fig.colorbar(surf, ax=ax, label='Depth (mm)', shrink=0.55, pad=0.08)

    plt.tight_layout()
    plt.show()


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(base_dir, '..', '..'))

    default_csv = os.path.join(
        project_root,
        'data/laser_scan/calibration/vertical/scan_raw_20260321_195848.csv'
    )

    csv_path = sys.argv[1] if len(sys.argv) > 1 else default_csv

    if not os.path.exists(csv_path):
        print(f"File not found: {csv_path}")
        sys.exit(1)

    cal = load_calib(project_root)
    print(f"Calibration: d0={cal['d0']:.1f}mm, K={cal['K']:.2f}, "
          f"alpha={np.degrees(cal['alpha_rad']):.3f}deg")

    print(f"Loading: {csv_path}")
    laser = load_and_compute(csv_path, cal)
    print(f"  {len(laser)} points, "
          f"X: {laser['X'].min():.1f}~{laser['X'].max():.1f}mm, "
          f"Y: {laser['Y'].min():.1f}~{laser['Y'].max():.1f}mm, "
          f"Z: {laser['Z'].min():.1f}~{laser['Z'].max():.1f}mm")

    # Step 높이 검증
    from scipy.signal import find_peaks
    Z = laser['Z'].values
    hist, edges = np.histogram(Z, bins=200, range=(-10, 70))
    ctrs = (edges[:-1] + edges[1:]) / 2
    pks, _ = find_peaks(hist, height=50, distance=8, prominence=20)
    if len(pks) > 0:
        floor_z = Z[np.abs(Z - ctrs[pks[0]]) < 3].mean()
        print("\n  Step Heights:")
        for i, p in enumerate(pks):
            mask = np.abs(Z - ctrs[p]) < 3
            z_mean = Z[mask].mean()
            dz = z_mean - floor_z
            err = dz - i * 10
            print(f"    Step {i}: {dz:.2f}mm (expected {i*10}mm, err={err:+.2f}mm)")

    show_3d(laser)


if __name__ == '__main__':
    main()
