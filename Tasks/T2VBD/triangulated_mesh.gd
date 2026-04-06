extends MeshInstance3D

@export var packed_scene: PackedScene
@export var tolerance: float = 0.0001

var edges: Array
var unique_verts: PackedVector3Array
var instances: Array = []
var original_surface_arrays: Array
var vertex_to_instance_idx: Array

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
func delaunay_3d(vertices: PackedVector3Array) -> Array:
	var indices = Geometry3D.tetrahedralize_delaunay(vertices)
	var tets = []
	for i in range(0, indices.size(), 4):
		tets.append([
			indices[i],   indices[i+1],
			indices[i+2], indices[i+3]
		])
	
	var face_to_tets = {}
	for tet_idx in range(tets.size()):
		var tet = tets[tet_idx]
		var faces = [
			[tet[0], tet[1], tet[2]],
			[tet[0], tet[1], tet[3]],
			[tet[0], tet[2], tet[3]],
			[tet[1], tet[2], tet[3]]
		]
		for face in faces:
			face.sort()
			var key = "%d,%d,%d" % [face[0], face[1], face[2]]
			if not face_to_tets.has(key):
				face_to_tets[key] = []
			face_to_tets[key].append(tet_idx)
	
	var edge_keys = {}
	for tet in tets:
		var v0 = tet[0]; var v1 = tet[1]; var v2 = tet[2]; var v3 = tet[3]
		var tet_edges = [
			[v0, v1], [v0, v2], [v0, v3],
			[v1, v2], [v1, v3], [v2, v3]
		]
		for e in tet_edges:
			var key = "%d,%d" % [min(e[0], e[1]), max(e[0], e[1])]
			edge_keys[key] = true
	
	for face_key in face_to_tets:
		var tet_indices = face_to_tets[face_key]
		if tet_indices.size() != 2:
			continue
		var tet_a = tets[tet_indices[0]]
		var tet_b = tets[tet_indices[1]]
		var parts = face_key.split(",")
		var face_verts = [int(parts[0]), int(parts[1]), int(parts[2])]
		var tip_a = -1; var tip_b = -1
		for v in tet_a:
			if not v in face_verts:
				tip_a = v; break
		for v in tet_b:
			if not v in face_verts:
				tip_b = v; break
		if tip_a == -1 or tip_b == -1:
			continue
		var beam_key = "%d,%d" % [min(tip_a, tip_b), max(tip_a, tip_b)]
		edge_keys[beam_key] = true
	
	var edges_out = []
	for key in edge_keys:
		var parts = key.split(",")
		edges_out.append(Vector2(int(parts[0]), int(parts[1])))
	return edges_out

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

# ------------------------------------------------------------------------------
func build_vertex_to_instance_mapping() -> void:
	var original_world_verts = get_world_space_vertices()
	var original_vert_count = original_world_verts.size()
	vertex_to_instance_idx.resize(original_vert_count)
	
	for i in range(original_vert_count):
		var v_world = original_world_verts[i]
		var found_idx = -1
		for u_idx in range(unique_verts.size()):
			if v_world.distance_to(unique_verts[u_idx]) <= tolerance:
				found_idx = u_idx
				break
		if found_idx != -1:
			vertex_to_instance_idx[i] = found_idx
		else:
			vertex_to_instance_idx[i] = 0
			push_warning("Vertex mapping failed – using first instance as fallback")

# ------------------------------------------------------------------------------
func update_mesh_from_instances() -> void:
	# Guard against node not being in tree (prevents the error)
	if not is_inside_tree():
		return
	if instances.is_empty() or original_surface_arrays.is_empty():
		return
	
	var new_mesh = ArrayMesh.new()
	
	for surf_idx in range(original_surface_arrays.size()):
		var old_surf = original_surface_arrays[surf_idx]
		var old_vertices: PackedVector3Array = old_surf[Mesh.ARRAY_VERTEX]
		
		var new_vertices = PackedVector3Array()
		new_vertices.resize(old_vertices.size())
		
		for vert_idx in range(old_vertices.size()):
			var inst_idx = vertex_to_instance_idx[vert_idx]
			var inst_world_pos = instances[inst_idx].global_position
			var local_pos = global_transform.affine_inverse() * inst_world_pos
			new_vertices[vert_idx] = local_pos
		
		var new_surf = old_surf.duplicate()
		new_surf[Mesh.ARRAY_VERTEX] = new_vertices
		new_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, new_surf)
	
	mesh = new_mesh

# ------------------------------------------------------------------------------
func _ready() -> void:
	if mesh == null:
		print("Error: No mesh assigned to MeshInstance3D.")
		return
	
	# Store original surface arrays
	original_surface_arrays = []
	for surf_idx in mesh.get_surface_count():
		original_surface_arrays.append(mesh.surface_get_arrays(surf_idx))
	
	var world_verts = get_world_space_vertices()
	unique_verts = deduplicate_vertices(world_verts, tolerance)
	
	if packed_scene != null:
		for v in unique_verts:
			var instance = packed_scene.instantiate()
			get_tree().root.add_child.call_deferred(instance)
			set_world_pos.call_deferred(instance, v)
			instances.append(instance)
		print("Spawned %d objects (original vertex count: %d)" % [unique_verts.size(), world_verts.size()])
	else:
		print("Warning: No packed_scene assigned. Object spawning skipped.")
	
	build_vertex_to_instance_mapping()
	
	if unique_verts.size() >= 4:
		edges = delaunay_3d(unique_verts)
		print("Generated %d Delaunay edges." % edges.size())
		draw_edges(edges, unique_verts, Color.CYAN)
	else:
		print("Not enough unique vertices (need ≥4) for Delaunay triangulation.")
	
	show()
	# Do NOT call update_mesh_from_instances() here – it will run in _process after the node is ready

# ------------------------------------------------------------------------------
func _process(delta: float) -> void:
	if instances.is_empty() or edges.is_empty():
		return
	
	update_mesh_from_instances()
	
	var current_positions = PackedVector3Array()
	for inst in instances:
		current_positions.append(inst.global_position)
	draw_edges(edges, current_positions, Color.RED)

# ------------------------------------------------------------------------------
func set_world_pos(node, position):
	node.global_position = position
