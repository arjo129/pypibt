from shapely.geometry import LineString, Point

path1_coords = [(0, 0), (2, 2), (0, 4)]
path2_coords = [(0, 2), (2, 0), (2, 4)]

line1 = LineString(path1_coords)
line2 = LineString(path2_coords)

intersection = line1.intersection(line2)

if intersection.is_empty:
    print("No intersection found.")
elif intersection.geom_type == 'Point':
    print(f"Intersection point: {intersection.coords[0]}")
elif intersection.geom_type == 'MultiPoint':
    print(f"Intersection points: {[p.coords[0] for p in intersection.geoms]}")
elif intersection.geom_type == 'LineString':
    print(f"Intersection is a line segment: {list(intersection.coords)}")
elif intersection.geom_type == 'MultiLineString':
    print(f"Intersection is multiple line segments: {[list(ls.coords) for ls in intersection.geoms]}")