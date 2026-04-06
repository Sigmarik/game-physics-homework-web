extends MeshInstance3D

# Assign a PackedScene (e.g., a .tscn file) in the Inspector
@export var packed_scene: PackedScene

# Tolerance for comparing vertex positions (avoids floating-point mismatches)
@export var tolerance: float = 0.0001

# Member variables
var edges: Array
var unique_verts: PackedVector3Array
var instances: Array = []

# ------------------------------------------------------------------------------
# Get world‑space vertices from the mesh
# ------------------------------------------------------------------------------
func get_world_space_vertices() -> PackedVector3Array:
	var world_vertices := PackedVector3Array()
	if mesh == null:
		return world_vertices
	for surface_idx in mesh.get_surface_count():
		var surface_array: Array = mesh.surface_get_arrays(surface_idx)
		var vertex_array: PackedVector3Array = surface_array[Mesh.ARRAY_VERTEX]
		for local_vertex in vertex_array:
			world_vertices.append(global_transform * local_vertex)
	return world_vertices

# ------------------------------------------------------------------------------
# Deduplicate vertices within tolerance
# ------------------------------------------------------------------------------
func deduplicate_vertices(vertices: PackedVector3Array, eps: float) -> PackedVector3Array:
	var unique := PackedVector3Array()
	for v in vertices:
		var is_dup := false
		for u in unique:
			if u.distance_to(v) <= eps:
				is_dup = true
				break
		if not is_dup:
			unique.append(v)
	return unique

# ------------------------------------------------------------------------------
# 3D Delaunay triangulation (Bowyer‑Watson) – returns Array of Vector2 edges
# ------------------------------------------------------------------------------
func delaunay_3d(vertices: PackedVector3Array) -> Array:
	var indices = Geometry3D.tetrahedralize_delaunay(vertices)
	edges = []
	for i in range(0, indices.size(), 4):
		var v0 = indices[i]
		var v1 = indices[i + 1]
		var v2 = indices[i + 2]
		var v3 = indices[i + 3]
		# Add edges of the tetrahedron (6 unique edges)
		var edge_set = [
			Vector2(v0, v1), Vector2(v0, v2), Vector2(v0, v3),
			Vector2(v1, v2), Vector2(v1, v3),
			Vector2(v2, v3)
		]
		for edge in edge_set:
			if not edges.has(edge) and not edges.has(Vector2(edge.y, edge.x)):
				edges.append(edge)
	return edges

# ------------------------------------------------------------------------------
# Draw edges using DebugDraw3D addon
# ------------------------------------------------------------------------------
func draw_edges(edges_array: Array, vertices_array: PackedVector3Array, color: Color) -> void:
	if not Engine.has_singleton("DebugDraw3D"):
		print("DebugDraw3D singleton not found. Please install the addon.")
		return
	var dd = Engine.get_singleton("DebugDraw3D")
	for e in edges_array:
		var p1 = vertices_array[int(e.x)]
		var p2 = vertices_array[int(e.y)]
		dd.draw_line(p1, p2, color)

func set_world_pos(node, position):
	node.global_position = position

# ------------------------------------------------------------------------------
# _ready – main entry point
# ------------------------------------------------------------------------------
func _ready() -> void:
	var world_verts = get_world_space_vertices()
	unique_verts = deduplicate_vertices(world_verts, tolerance)

	# Spawn objects at each unique vertex (original functionality)
	if packed_scene != null:
		for v in unique_verts:
			var instance = packed_scene.instantiate()
			get_tree().root.add_child.call_deferred(instance)
			set_world_pos.call_deferred(instance, v)
			instances.append(instance)
		print("Spawned %d objects (original vertex count: %d)" % [unique_verts.size(), world_verts.size()])
	else:
		print("Warning: No packed_scene assigned. Object spawning skipped.")

	# Perform Delaunay triangulation and draw edges
	if unique_verts.size() >= 4:
		edges = delaunay_3d(unique_verts)
		print("Generated %d Delaunay edges." % edges.size())
		draw_edges(edges, unique_verts, Color.CYAN)
	else:
		print("Not enough unique vertices (need ≥4) for Delaunay triangulation.")

	hide()

func _process(delta: float) -> void:
	# If you want to draw edges at the current positions of the spawned objects,
	# you can use that. Otherwise, use the original unique_verts.
	if instances.size() > 0 and edges.size() > 0:
		var current_positions = PackedVector3Array()
		for inst in instances:
			current_positions.append(inst.global_position)
		draw_edges(edges, current_positions, Color.RED)