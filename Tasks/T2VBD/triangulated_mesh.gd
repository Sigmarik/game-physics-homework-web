extends MeshInstance3D

# Assign a PackedScene (e.g., a .tscn file) in the Inspector
@export var packed_scene: PackedScene

# Tolerance for comparing vertex positions (avoids floating-point mismatches)
@export var tolerance: float = 0.0001

func get_world_space_vertices() -> PackedVector3Array:
	var world_vertices := PackedVector3Array()
	
	if mesh == null:
		return world_vertices
	
	for surface_idx in mesh.get_surface_count():
		var surface_array: Array = mesh.surface_get_arrays(surface_idx)
		var vertex_array: PackedVector3Array = surface_array[Mesh.ARRAY_VERTEX]
		for local_vertex in vertex_array:
			var world_vertex := global_transform * local_vertex
			world_vertices.append(world_vertex)
	
	return world_vertices

func deduplicate_vertices(vertices: PackedVector3Array, eps: float) -> PackedVector3Array:
	var unique: PackedVector3Array
	for v in vertices:
		var is_duplicate := false
		for u in unique:
			if u.distance_to(v) <= eps:
				is_duplicate = true
				break
		if not is_duplicate:
			unique.append(v)
	return unique

func _ready() -> void:
	var vertices = get_world_space_vertices()
	
	if packed_scene == null:
		print("Warning: No packed_scene assigned. Cannot summon objects.")
		return
	
	# Remove duplicate vertices within tolerance
	var unique_vertices = deduplicate_vertices(vertices, tolerance)
	
	# Spawn an instance at each unique vertex
	for vertex in unique_vertices:
		var instance = packed_scene.instantiate()
		add_child(instance)
		instance.global_position = vertex
	
	print("Spawned %d objects (original vertex count: %d)" % [unique_vertices.size(), vertices.size()])
