extends Camera3D

## Mouse sensitivity for orbit (radians per pixel)
@export var orbit_sensitivity := 0.003
## How much the distance changes per scroll step
@export var zoom_step := 1.0
## Minimum distance to the origin
@export var min_distance := 2.0
## Maximum distance to the origin
@export var max_distance := 50.0
## Minimum pitch angle (degrees) to avoid flipping
@export var min_pitch := -89.0
## Maximum pitch angle (degrees)
@export var max_pitch := 89.0

var yaw := 0.0       # Horizontal angle
var pitch := 0.0     # Vertical angle
var distance := 10.0 # Distance from origin

func _ready() -> void:
	# Initialize spherical angles from the current camera position
	var offset := global_position - Vector3.ZERO
	distance = offset.length()
	# Yaw: angle from +Z axis (since forward is -Z, we use atan2(x, z))
	yaw = atan2(offset.x, offset.z)
	# Pitch: angle from the horizontal plane
	pitch = asin(clampf(offset.y / distance, -1.0, 1.0))
	_update_camera()

func _input(event: InputEvent) -> void:
	# Orbit when right mouse button is pressed (mouse motion)
	if event is InputEventMouseMotion and Input.is_mouse_button_pressed(MOUSE_BUTTON_RIGHT):
		yaw -= event.relative.x * orbit_sensitivity
		pitch += event.relative.y * orbit_sensitivity
		pitch = clampf(pitch, deg_to_rad(min_pitch), deg_to_rad(max_pitch))

	# Zoom with mouse wheel
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			distance = maxf(distance - zoom_step, min_distance)
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			distance = minf(distance + zoom_step, max_distance)

	# Toggle mouse capture when right button is pressed / released
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_RIGHT:
		if event.pressed:
			Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
		else:
			Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)

func _process(_delta: float) -> void:
	_update_camera()

func _update_camera() -> void:
	# Convert spherical to Cartesian coordinates
	var pos := Vector3(
		distance * cos(pitch) * sin(yaw),
		distance * sin(pitch),
		distance * cos(pitch) * cos(yaw)
	)
	global_position = pos
	look_at(Vector3.ZERO, Vector3.UP)
