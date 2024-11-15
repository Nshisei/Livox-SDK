import open3d as o3d
import pandas as pd
import os
import glob
import numpy as np

def load_point_clouds_from_csvs():
    # カレントディレクトリにあるすべてのCSVファイルを検索
    csv_files = glob.glob("*.csv")
    all_points = []

    for csv_file in csv_files:
        # CSVファイルを読み込んでx, y, z列を取得
        df = pd.read_csv(csv_file, usecols=["x", "y", "z"])
        points = df[["x", "y", "z"]].values
        all_points.append(points)
        print(f"Loaded {len(points)} points from {csv_file}")

    # 全ての点群データを1つの配列に結合
    if all_points:
        all_points = np.vstack(all_points)
        print(f"Total points loaded: {len(all_points)}")
    else:
        print("No CSV files found in the current directory.")
        return None

    return all_points

def visualize_point_cloud(points):
    # 点群データをOpen3DのPointCloudオブジェクトに変換して可視化
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    o3d.visualization.draw_geometries([point_cloud])

if __name__ == "__main__":
    points = load_point_clouds_from_csvs()
    if points is not None:
        visualize_point_cloud(points)

