extends CollisionShape3D

# Dragging state
var dragging = false
var drag_plane = Plane.PLANE_XZ  # Plane to project mouse onto (XZ by default)
var drag_offset = Vector3.ZERO    # Offset from object to mouse projection

var last_frame_position = Vector3.ZERO
var mass = 1.0
var springs: Array[Node2D]

# The actual node to move (can be self or parent)
@export var movable_node: Node3D = null

func _ready():
	last_frame_position = movable_node.global_position if movable_node else global_position

	# If no movable node is set, try to use the parent (common setup)
	if not movable_node:
		movable_node = get_parent() as Node3D
		if not movable_node:
			push_error("Draggable: No movable node found! Assign 'movable_node' or ensure parent is a Node3D.")
			return
	
	# Connect input event from the parent (must be an Area3D or PhysicsBody)
	var parent = get_parent()
	if parent.has_signal("input_event"):
		parent.input_event.connect(_on_input_event)
	else:
		push_warning("Draggable: Parent does not have 'input_event' signal. " +
			"Make sure this CollisionShape3D is inside an Area3D or PhysicsBody.")

func _on_input_event(camera: Node, event: InputEvent, click_position: Vector3, click_normal: Vector3, shape_idx: int):
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT:
		if event.pressed:
			# Start dragging
			dragging = true
			# Create a drag plane at the object's position, facing the camera
			var cam_transform = camera.global_transform
			# var cam_pos = cam_transform.origin
			var object_pos = movable_node.global_position
			var cam_forward = -cam_transform.basis.z
			# var direction_to_object = (object_pos - cam_pos).normalized()
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
			movable_node.global_position = new_position

func _get_mouse_position_on_plane(camera: Camera3D, mouse_pos: Vector2, plane: Plane) -> Variant:
	# Raycast from camera through mouse position onto the plane
	var from = camera.project_ray_origin(mouse_pos)
	var direction = camera.project_ray_normal(mouse_pos)
	var intersection = plane.intersects_ray(from, direction)
	return intersection  # Returns null if no intersection
