@tool
extends MeshInstance3D

func _ready() -> void:
	_sync_mesh()


func _process(_delta: float) -> void:
	# Update continuously while in the editor so changes to the collision shape
	# (size, transform) are reflected immediately.
	if Engine.is_editor_hint():
		_sync_mesh()


func _sync_mesh() -> void:
	var sibling_shape := _find_sibling_box_shape()
	if not sibling_shape:
		return

	var box_shape: BoxShape3D = sibling_shape.shape
	# Create or reuse a BoxMesh
	if not mesh is BoxMesh:
		mesh = BoxMesh.new()
	var box_mesh := mesh as BoxMesh
	box_mesh.size = box_shape.extents * 2.0

	# Copy the sibling's transform so the mesh always matches its position/rotation/scale
	transform = sibling_shape.transform


func _find_sibling_box_shape() -> CollisionShape3D:
	var parent_node = get_parent()
	if not parent_node:
		return null

	for child in parent_node.get_children():
		if child == self:
			continue
		if child is CollisionShape3D and child.shape is BoxShape3D:
			return child
	return null
