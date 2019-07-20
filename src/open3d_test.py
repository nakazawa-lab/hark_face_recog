#!/usr/bin/env python
## coding: UTF-8

## https://qiita.com/tttamaki/items/2be476cc15fa39601243

# import sys
# sys.path.append("./bunny/data") # ビルドしたディレクトリ Open3D/build/lib/ へのパス
import numpy as np
import open3d as py3d

print("read ply points#############################")
pcd1 = py3d.read_point_cloud("./bunny/data/bun000.ply") # メッシュなしply
print("pcd1:", pcd1)
print("has points?", pcd1.has_points())
point_array = np.asarray(pcd1.points)
print(point_array.shape, "points:\n", point_array)
print("has color?", pcd1.has_colors())
print("colors:", np.asarray(pcd1.colors))
print("has normals?", pcd1.has_normals())
py3d.draw_geometries([pcd1], window_name="pcd1 without normals", width=640, height=480)


print("estimate normal#############################")
py3d.estimate_normals(pcd1, search_param = py3d.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
print("has normals?", pcd1.has_normals())
normal_array = np.asarray(pcd1.normals)
print(normal_array.shape, "normals:\n", normal_array)
py3d.draw_geometries([pcd1], "pcd1 with normals", 640, 480)


# print("read ply mesh#############################")
# pcd2 = py3d.read_triangle_mesh("./bunny/data/bun000mesh.ply") # メッシュありply
# print("has triangle normals?", pcd2.has_triangle_normals())
# print("triangle normals:\n", np.asarray(pcd2.triangle_normals))
# print("has triangles?", pcd2.has_triangles())
# print("triangles:", np.asarray(pcd2.triangles))
# print("has vertices?", pcd2.has_vertices())
# print("has vertex colors?", pcd2.has_vertex_colors())
# print("vertex colors:", np.asarray(pcd2.vertex_colors))
# print("has vertex normals?", pcd2.has_vertex_normals())
# print("vertex normals:\n", np.asarray(pcd2.vertex_normals))
# py3d.draw_geometries([pcd2], "pcd2 with mesh but no normals", 640, 480)

# print("estimate normal#############################")
# pcd2.compute_vertex_normals()
# print("has triangle normals?", pcd2.has_triangle_normals())
# print("triangle normals:\n", np.asarray(pcd2.triangle_normals))
# print("has triangles?", pcd2.has_triangles())
# print("triangles:", np.asarray(pcd2.triangles))
# print("has vertices?", pcd2.has_vertices())
# print("has vertex colors?", pcd2.has_vertex_colors())
# print("vertex colors:", np.asarray(pcd2.vertex_colors))
# print("has vertex normals?", pcd2.has_vertex_normals())
# print("vertex normals:\n", np.asarray(pcd2.vertex_normals))
# py3d.draw_geometries([pcd2], "pcd2 with mesh and normals", 640, 480)