extends AnimatableBody3D

# Dragging state
var dragging = false
var drag_plane = Plane.PLANE_XZ  # Plane to project mouse onto (XZ by default)
var drag_offset = Vector3.ZERO    # Offset from object to mouse projection

var fixed_in_place = false  # If true, object won't move (can be used for fixed points in a soft body)

const SPRING_STIFFNESS = 1000.0 # Spring stiffness for connected springs (if any)

class SoftBodySpring:
	var other_node: Node3D
	var rest_length: float

var last_frame_position
var mass = 1.0
var springs: Array[SoftBodySpring]

# The actual node to move (usually self, but can be another child if needed)
@export var movable_node: Node3D = null

func get_spring_force() -> Vector3:
	var total_force = Vector3.ZERO
	for spring in springs:
		var delta_position = movable_node.global_position - spring.other_node.movable_node.global_position
		var normalized_delta = delta_position.normalized()
		var delta_length = delta_position.length() - spring.rest_length
		total_force -= normalized_delta * delta_length * SPRING_STIFFNESS
	return total_force

# a.k.a. energy derivative times -1
func outer_product_from_vector3(v: Vector3) -> DenseMatrix:
	var packed_array = PackedFloat64Array([v.x, v.y, v.z])
	var vec_n = VectorN.from_packed_array(packed_array)
	return vec_n.column_vector().multiply_dense(vec_n.row_vector())

func get_spring_hessian() -> DenseMatrix:
	var hessian = DenseMatrix.zero(3)
	for spring in springs:
		var delta_position = movable_node.global_position - spring.other_node.movable_node.global_position
		var length = delta_position.length()
		var local_hessian : DenseMatrix = DenseMatrix.identity(3)
		local_hessian.multiply_scaler_in_place(1 - spring.rest_length / length)
		var outer_product = outer_product_from_vector3(delta_position)
		outer_product.multiply_scaler_in_place(spring.rest_length / (length * length * length))
		local_hessian.add_dense_in_place(outer_product)
		local_hessian.multiply_scaler_in_place(SPRING_STIFFNESS)
		hessian.add_dense_in_place(local_hessian)
	return hessian

func _ready():
	# Default to self if no movable node is assigned
	if not movable_node:
		movable_node = self

	# Connect input event from this AnimatableBody3D (it's a CollisionObject3D)
	if has_signal("input_event"):
		input_event.connect(_on_input_event)
	else:
		push_error("Draggable: AnimatableBody3D does not have 'input_event' signal. " +
			"Make sure this node is properly set up as a CollisionObject3D.")

func _on_input_event(camera: Node, event: InputEvent, click_position: Vector3, click_normal: Vector3, shape_idx: int):
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT:
		if event.pressed:
			# Start dragging
			dragging = true
			# Create a drag plane at the object's position, facing the camera
			var cam_transform = camera.global_transform
			var object_pos = movable_node.global_position
			var cam_forward = -cam_transform.basis.z
			drag_plane = Plane(cam_forward, object_pos)
			# Compute initial offset
			var mouse_pos_3d = _get_mouse_position_on_plane(camera, get_viewport().get_mouse_position(), drag_plane)
			if mouse_pos_3d:
				drag_offset = movable_node.global_position - mouse_pos_3d
			else:
				dragging = false
		else:
			# Stop dragging
			dragging = false

func _input(event: InputEvent):
	if dragging and event is InputEventMouseMotion:
		var camera = get_viewport().get_camera_3d()
		if not camera:
			return
		var mouse_pos_3d = _get_mouse_position_on_plane(camera, event.position, drag_plane)
		if mouse_pos_3d:
			var new_position = mouse_pos_3d + drag_offset
			last_frame_position = movable_node.global_position
			movable_node.global_position = new_position

func _get_mouse_position_on_plane(camera: Camera3D, mouse_pos: Vector2, plane: Plane) -> Variant:
	# Raycast from camera through mouse position onto the plane
	var from = camera.project_ray_origin(mouse_pos)
	var direction = camera.project_ray_normal(mouse_pos)
	var intersection = plane.intersects_ray(from, direction)
	return intersection  # Returns null if no intersection
