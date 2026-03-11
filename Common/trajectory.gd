extends Line2D

# Maximum number of points in the trail
@export var max_points := 50

# Minimum distance the parent must move before a new point is added
@export var spacing_distance := 20.0

# Stores the global positions of the parent (most recent first)
var _history: Array[Vector2] = []

# The position of the last point we added
var _last_recorded_position: Vector2

func _ready():
	var parent = get_parent()
	if not parent is Node2D:
		push_error("TrailLine2D must be a child of a Node2D.")
		return

	# Initialize with the parent's current position
	_last_recorded_position = parent.global_position
	_history.append(_last_recorded_position)

func _process(_delta):
	var parent = get_parent()
	if not parent is Node2D:
		return

	var current_pos = parent.global_position

	# Add a new point only if the parent has moved far enough
	if _last_recorded_position.distance_to(current_pos) >= spacing_distance:
		_history.insert(0, current_pos)
		_last_recorded_position = current_pos

		# Keep the history within the size limit
		if _history.size() > max_points:
			_history.pop_back()

	# Convert global positions to local coordinates relative to this Line2D
	var local_points = PackedVector2Array()
	for global_pos in _history:
		local_points.append(global_pos - global_position)

	points = local_points