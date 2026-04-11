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

const CONNECTION_RADIUS = 5.5   # adjust based on your point cloud density

func delaunay_3d(vertices: PackedVector3Array) -> Array:
	var indices = Geometry3D.tetrahedralize_delaunay(vertices)
	var edge_set = {}
	
	# Process each tetrahedron (4 indices per tet)
	for i in range(0, indices.size(), 4):
		var tet = [
			indices[i],   indices[i+1],
			indices[i+2], indices[i+3]
		]
		
		# 6 edges of the tetrahedron
		var edges = [
			[tet[0], tet[1]], [tet[0], tet[2]], [tet[0], tet[3]],
			[tet[1], tet[2]], [tet[1], tet[3]], [tet[2], tet[3]]
		]
		
		for e in edges:
			var key = "%d,%d" % [min(e[0], e[1]), max(e[0], e[1])]
			edge_set[key] = true
	
	var edges_out = []
	for key in edge_set:
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
	if not is_inside_tree():
		return
	if instances.is_empty() or original_surface_arrays.is_empty():
		return
	
	var new_mesh = ArrayMesh.new()
	
	for surf_idx in range(original_surface_arrays.size()):
		var old_surf = original_surface_arrays[surf_idx]
		var old_vertices: PackedVector3Array = old_surf[Mesh.ARRAY_VERTEX]
		
		# --- Build new vertex positions ---
		var new_vertices = PackedVector3Array()
		new_vertices.resize(old_vertices.size())
		for vert_idx in range(old_vertices.size()):
			var inst_idx = vertex_to_instance_idx[vert_idx]
			var inst_world_pos = instances[inst_idx].global_position
			var local_pos = global_transform.affine_inverse() * inst_world_pos
			new_vertices[vert_idx] = local_pos
		
		# --- Recalculate normals ---
		# Retrieve index array (if present)
		var indices: PackedInt32Array
		if old_surf.size() > Mesh.ARRAY_INDEX and old_surf[Mesh.ARRAY_INDEX] != null:
			indices = old_surf[Mesh.ARRAY_INDEX]
		else:
			indices = PackedInt32Array()
		
		# If no index array, assume vertices are in triangle strips (consecutive triplets)
		if indices.is_empty():
			indices.resize(new_vertices.size())
			for i in range(new_vertices.size()):
				indices[i] = i
		
		var new_normals = PackedVector3Array()
		new_normals.resize(new_vertices.size())
		for i in range(new_normals.size()):
			new_normals[i] = Vector3.ZERO
		
		# Accumulate face normals
		for tri in range(0, indices.size(), 3):
			var i0 = indices[tri]
			var i1 = indices[tri + 1]
			var i2 = indices[tri + 2]
			
			var v0 = new_vertices[i0]
			var v1 = new_vertices[i1]
			var v2 = new_vertices[i2]
			
			var face_normal = (v1 - v0).cross(v2 - v0).normalized()
			new_normals[i0] += face_normal
			new_normals[i1] += face_normal
			new_normals[i2] += face_normal
		
		# Normalise accumulated normals
		for i in range(new_normals.size()):
			var n = new_normals[i]
			if n != Vector3.ZERO:
				new_normals[i] = -n.normalized()
		
		# --- Build new surface ---
		var new_surf = old_surf.duplicate()
		new_surf[Mesh.ARRAY_VERTEX] = new_vertices
		new_surf[Mesh.ARRAY_NORMAL] = new_normals
		new_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, new_surf)
	
	mesh = new_mesh

# ------------------------------------------------------------------------------
func setup_springs() -> void:
	if instances.is_empty() or edges.is_empty():
		return
	
	# Get the SoftBodySpring class from the first instance's script
	var first_script = instances[0].get_script()
	if not first_script:
		push_error("Instances have no script – cannot create springs.")
		return
	
	var SpringClass = first_script.get("SoftBodySpring")
	if not SpringClass:
		push_error("Script does not define SoftBodySpring inner class.")
		return
	
	for edge in edges:
		var idx_a = int(edge.x)
		var idx_b = int(edge.y)
		var node_a = instances[idx_a]
		var node_b = instances[idx_b]
		var pos_a = unique_verts[idx_a]
		var pos_b = unique_verts[idx_b]
		
		# Compute initial rest length (world distance)
		var rest_len = pos_a.distance_to(pos_b)
		var rest_vector = pos_b - pos_a
		
		# Create spring for node_a
		var spring_a = SpringClass.new()
		spring_a.other_node = node_b
		spring_a.rest_length = rest_len
		spring_a.rest_vector = rest_vector
		node_a.springs.append(spring_a)
		
		# Create spring for node_b
		var spring_b = SpringClass.new()
		spring_b.other_node = node_a
		spring_b.rest_length = rest_len
		spring_b.rest_vector = -rest_vector
		node_b.springs.append(spring_b)

	for instance in instances:
		instance.capture_volume_springs()

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

	var fixed_vertex_index = 0
	
	if packed_scene != null:
		for v in unique_verts:
			var instance = packed_scene.instantiate()
			get_tree().current_scene.add_child.call_deferred(instance)
			set_world_pos.call_deferred(instance, v)
			if v.y > unique_verts[fixed_vertex_index].y: fixed_vertex_index = instances.size()
			instance.last_frame_position = v
			instances.append(instance)
			instance.add_to_group("vbd_point")
		print("Spawned %d objects (original vertex count: %d)" % [unique_verts.size(), world_verts.size()])
	else:
		print("Warning: No packed_scene assigned. Object spawning skipped.")
	
	build_vertex_to_instance_mapping()

	instances[fixed_vertex_index].fixed_in_place = true
	
	if unique_verts.size() >= 4:
		edges = delaunay_3d(unique_verts)
		print("Generated %d Delaunay edges." % edges.size())
		draw_edges(edges, unique_verts, Color.CYAN)
		
		# Add springs for each edge
		setup_springs.call_deferred()
	else:
		print("Not enough unique vertices (need ≥4) for Delaunay triangulation.")
	
	show()

# ------------------------------------------------------------------------------
func _process(delta: float) -> void:
	if instances.is_empty() or edges.is_empty():
		return
	
	update_mesh_from_instances()
	
	# var current_positions = PackedVector3Array()
	# for inst in instances:
	# 	current_positions.append(inst.global_position)
	# draw_edges(edges, current_positions, Color.RED)

# ------------------------------------------------------------------------------
func set_world_pos(node, position):
	node.global_position = position
