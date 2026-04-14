extends AnimatableBody3D

# Dragging state
var dragging = false
var drag_plane = Plane.PLANE_XZ
var drag_offset = Vector3.ZERO

var fixed_in_place = false

const SPRING_STIFFNESS = 100.0
const EXPANSION_RESISTANCE = 100.0
const TORQUE_RESISTANCE = 0.0

class VolumeElement:
	var volume_axis_a = 0
	var volume_axis_b = 0
	var volume_axis_c = 0
	var default_volume = 0.0

var volume_elements: Array[VolumeElement] = []

class SoftBodySpring:
	var other_node: Node3D
	var rest_length: float
	var rest_vector: Vector3

var last_frame_position: Vector3
var mass = 1.0
var springs: Array[SoftBodySpring] = []
var planned_position = Vector3.ZERO

# ------------------------------------------------------------------
# Helper: 3×3 determinant (now uses Basis)
# ------------------------------------------------------------------
func determinant_3x3(m: Basis) -> float:
	return m.determinant()

# ------------------------------------------------------------------
# Helper: trace of 3×3 matrix
# ------------------------------------------------------------------
func trace_3x3(m: Basis) -> float:
	return m[0][0] + m[1][1] + m[2][2]

@export var movable_node: Node3D = null

func build_volume_element(i: int, j: int, h: int) -> void:
	var element = VolumeElement.new()
	var vec_a = springs[i].rest_vector
	var vec_b = springs[j].rest_vector
	var vec_c = springs[h].rest_vector
	var volume = vec_a.cross(vec_b).dot(vec_c)
	if abs(volume) <= 0.01: return
	if volume > 0:
		element.volume_axis_a = i
		element.volume_axis_b = j
		element.volume_axis_c = h
	else:
		element.volume_axis_a = i
		element.volume_axis_b = h
		element.volume_axis_c = j
	element.default_volume = abs(volume)
	volume_elements.append(element)

func append_if_no_match(arr: Array, a: int, b: int, c: int) -> void:
	if a == b or a == c or b == c: return
	for existing in arr:
		if (existing.x == a and existing.y == b and existing.z == c) or \
		   (existing.x == a and existing.y == c and existing.z == b) or \
		   (existing.x == b and existing.y == a and existing.z == c) or \
		   (existing.x == b and existing.y == c and existing.z == a) or \
		   (existing.x == c and existing.y == a and existing.z == b) or \
		   (existing.x == c and existing.y == b and existing.z == a):
			return
	arr.append(Vector3i(a, b, c))

func pseudo_convex_hull(points: Array) -> Array[Vector3i]:
	var triangles: Array[Vector3i] = []
	var axis_x = 0
	var axis_y = 0
	var axis_z = 0

	for p in range(0, points.size()):
		for q in range(p + 1, points.size()):
			if abs(points[axis_x].cross(points[p]).dot(points[q])) > abs(points[axis_x].cross(points[axis_y]).dot(points[axis_z])):
				axis_y = p
				axis_z = p

	var axis_nx = 0
	var axis_ny = 0
	var axis_nz = 0

	for p in range(0, points.size()):
		if points[p].dot(-points[axis_x]) > points[axis_nx].dot(-points[axis_x]):
			axis_nx = p
	for p in range(0, points.size()):
		if points[p].dot(-points[axis_y]) > points[axis_ny].dot(-points[axis_y]):
			axis_ny = p
	for p in range(0, points.size()):
		if points[p].dot(-points[axis_z]) > points[axis_nz].dot(-points[axis_z]):
			axis_nz = p

	append_if_no_match(triangles, axis_x, axis_y, axis_z)
	append_if_no_match(triangles, axis_x, axis_y, axis_nz)
	append_if_no_match(triangles, axis_x, axis_ny, axis_z)
	append_if_no_match(triangles, axis_x, axis_ny, axis_nz)
	append_if_no_match(triangles, axis_nx, axis_y, axis_z)
	append_if_no_match(triangles, axis_nx, axis_y, axis_nz)
	append_if_no_match(triangles, axis_nx, axis_ny, axis_z)
	append_if_no_match(triangles, axis_nx, axis_ny, axis_nz)

	return triangles

func triplets_from_springs() -> Array[Vector3i]:
	var axes = []
	for i in range(0, springs.size()):
		var vector = springs[i].rest_vector
		axes.append(vector.normalized())
	return pseudo_convex_hull(axes)

func capture_volume_springs() -> void:
	for triplet in triplets_from_springs():
		build_volume_element(triplet.x, triplet.y, triplet.z)

func get_neophookean_energy_from_volume_element_at(element: VolumeElement, point: Vector3) -> float:
	if element.default_volume == 0: return 0.0

	var spring_a: SoftBodySpring = springs[element.volume_axis_a]
	var spring_b: SoftBodySpring = springs[element.volume_axis_b]
	var spring_c: SoftBodySpring = springs[element.volume_axis_c]
	var vec_a = spring_a.other_node.planned_position - point
	var vec_b = spring_b.other_node.planned_position - point
	var vec_c = spring_c.other_node.planned_position - point

	var current_volume = vec_a.cross(vec_b).dot(vec_c)
	var torque_a = vec_a.length() / spring_a.rest_length
	var torque_b = vec_b.length() / spring_b.rest_length
	var torque_c = vec_c.length() / spring_c.rest_length
	
	var volume_penalty = current_volume / element.default_volume - 1.0
	volume_penalty *= volume_penalty
	var delta_torque = torque_a * torque_a + torque_b * torque_b + torque_c * torque_c - 3.0

	return EXPANSION_RESISTANCE * volume_penalty / 2.0 + TORQUE_RESISTANCE * delta_torque / 2.0

func get_neohookean_energy_at(point: Vector3) -> float:
	var energy = 0.0
	for element in volume_elements:
		energy += get_neophookean_energy_from_volume_element_at(element, point)
	return energy

func get_neohookean_energy() -> float:
	return get_neohookean_energy_at(movable_node.global_position)

const DERIVATIVE_EPS = 0.01

class DerivativeHessian:
	var derivative: Vector3
	var hessian: Basis

# ------------------------------------------------------------------
# Outer product of a Vector3 (v * vᵀ)
# ------------------------------------------------------------------
func _outer_product(v: Vector3) -> Basis:
	return Basis(
		Vector3(v.x * v.x, v.y * v.x, v.z * v.x),   # column 0
		Vector3(v.x * v.y, v.y * v.y, v.z * v.y),   # column 1
		Vector3(v.x * v.z, v.y * v.z, v.z * v.z)    # column 2
	)

func get_neohookean_info() -> DerivativeHessian:
	var en = []
	var origin = movable_node.global_position
	for i in range(-1, 2):
		en.append([])
		for j in range(-1, 2):
			en[i + 1].append([])
			for h in range(-1, 2):
				var shift = Vector3(i, j, h) * DERIVATIVE_EPS
				var energy = get_neohookean_energy_at(origin + shift)
				en[i + 1][j + 1].append(energy)

	var info = DerivativeHessian.new()

	var dx = en[2][1][1] - en[0][1][1]
	var dy = en[1][2][1] - en[1][0][1]
	var dz = en[1][1][2] - en[1][1][0]
	info.derivative = Vector3(dx, dy, dz) / DERIVATIVE_EPS * 0.5

	var dxx = en[2][1][1] - 2.0 * en[1][1][1] + en[0][1][1]
	var dyy = en[1][2][1] - 2.0 * en[1][1][1] + en[1][0][1]
	var dzz = en[1][1][2] - 2.0 * en[1][1][1] + en[1][1][0]
	var dxy = (en[2][2][1] - en[0][2][1]) - (en[2][0][1] - en[0][0][1])
	dxy *= 0.25
	var dxz = (en[2][1][2] - en[0][1][2]) - (en[2][1][0] - en[0][1][0])
	dxz *= 0.25
	var dyz = (en[1][2][2] - en[1][0][2]) - (en[1][2][0] - en[1][0][0])
	dyz *= 0.25

	info.hessian = Basis(
		Vector3(dxx, dxy, dxz),
		Vector3(dxy, dyy, dyz),
		Vector3(dxz, dyz, dzz)
	)
	# Multiply by 1/eps²
	var scale = 1.0 / (DERIVATIVE_EPS * DERIVATIVE_EPS)
	info.hessian = Basis(
		info.hessian[0] * scale,
		info.hessian[1] * scale,
		info.hessian[2] * scale
	)
	return info

func get_total_energy_at(point: Vector3) -> float:
	return get_neohookean_energy_at(point) + get_spring_energy_at(point)

func get_spring_energy_at(point: Vector3) -> float:
	var energy = 0.0
	for spring in springs:
		var delta_position = movable_node.global_position - spring.other_node.planned_position
		var delta_length = delta_position.length() - spring.rest_length
		energy += delta_length * delta_length * SPRING_STIFFNESS * 0.5
	return energy

func get_spring_force() -> Vector3:
	var total_force = Vector3.ZERO
	for spring in springs:
		var delta_position = movable_node.global_position - spring.other_node.planned_position
		var normalized_delta = delta_position.normalized()
		var delta_length = delta_position.length() - spring.rest_length
		total_force -= normalized_delta * delta_length * SPRING_STIFFNESS
	return total_force

func get_spring_hessian() -> Basis:
	var hessian = Basis()  # zero matrix
	for spring in springs:
		var delta_position = movable_node.global_position - spring.other_node.planned_position
		var length = delta_position.length()
		# local_hessian = I * (1 - rest/length) + outer(delta) * (rest / length³)
		var local_hessian = Basis.IDENTITY
		local_hessian = Basis(
			local_hessian[0] * (1.0 - spring.rest_length / length),
			local_hessian[1] * (1.0 - spring.rest_length / length),
			local_hessian[2] * (1.0 - spring.rest_length / length)
		)
		var outer = _outer_product(delta_position)
		var outer_scale = spring.rest_length / (length * length * length)
		outer = Basis(
			outer[0] * outer_scale,
			outer[1] * outer_scale,
			outer[2] * outer_scale
		)
		local_hessian = Basis(
			local_hessian[0] + outer[0],
			local_hessian[1] + outer[1],
			local_hessian[2] + outer[2]
		)
		# Multiply by stiffness
		local_hessian = Basis(
			local_hessian[0] * SPRING_STIFFNESS,
			local_hessian[1] * SPRING_STIFFNESS,
			local_hessian[2] * SPRING_STIFFNESS
		)
		hessian = Basis(
			hessian[0] + local_hessian[0],
			hessian[1] + local_hessian[1],
			hessian[2] + local_hessian[2]
		)
	return hessian

func _ready():
	if not movable_node:
		movable_node = self

func _unhandled_input(event):
	var camera = get_viewport().get_camera_3d()
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT:
		if event.pressed:
			var space_state = get_world_3d().direct_space_state
			var mouse_pos = get_viewport().get_mouse_position()
			var origin = camera.project_ray_origin(mouse_pos)
			var end = origin + camera.project_ray_normal(mouse_pos) * 1000.0
			var query = PhysicsRayQueryParameters3D.create(origin, end)
			query.collide_with_areas = true
			query.collide_with_bodies = true
			var result = space_state.intersect_ray(query)
			
			if result and result.collider == self:
				dragging = true
				var cam_transform = camera.global_transform
				var object_pos = movable_node.global_position
				var cam_forward = -cam_transform.basis.z
				drag_plane = Plane(cam_forward, object_pos)
				var mouse_pos_3d = _get_mouse_position_on_plane(camera, get_viewport().get_mouse_position(), drag_plane)
				if mouse_pos_3d:
					drag_offset = movable_node.global_position - mouse_pos_3d
				else:
					dragging = false
				get_viewport().set_input_as_handled()
		else:
			dragging = false
			if dragging: get_viewport().set_input_as_handled()

	if dragging and event is InputEventMouseMotion:
		var mouse_pos_3d = _get_mouse_position_on_plane(camera, event.position, drag_plane)
		if mouse_pos_3d:
			movable_node.global_position = mouse_pos_3d + drag_offset
		get_viewport().set_input_as_handled()

func _get_mouse_position_on_plane(camera: Camera3D, mouse_pos: Vector2, plane: Plane):
	var from = camera.project_ray_origin(mouse_pos)
	var direction = camera.project_ray_normal(mouse_pos)
	var intersection = plane.intersects_ray(from, direction)
	return intersection