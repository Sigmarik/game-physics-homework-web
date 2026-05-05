@tool
extends Node3D

@export var first_object : PhysicalBox
@export var first_shift := Vector3.ZERO

@export var second_object : Node3D
@export var second_shift := Vector3.ZERO

func _ready() -> void:
	if Engine.is_editor_hint():
		# Editor: do nothing permanent, just let _process draw
		return

	# Runtime (unchanged)
	if first_object is PhysicalBox:
		var constraint := PivotConstraint.new()
		constraint.anchor_object = second_object
		constraint.anchor_position = second_shift
		constraint.relative_shift = first_shift
		constraint.compliance = 0.001
		constraint.capture_distance(first_object)
		first_object.constraints.append(constraint)
	if second_object is PhysicalBox:
		var constraint := PivotConstraint.new()
		constraint.anchor_object = first_object
		constraint.anchor_position = first_shift
		constraint.relative_shift = second_shift
		constraint.compliance = 0.001
		constraint.capture_distance(second_object)
		second_object.constraints.append(constraint)
	queue_free()

func _process(delta: float) -> void:
	if not Engine.is_editor_hint():
		return

	# Draw the constraints using the existing draw_constraint method
	if is_instance_valid(first_object):
		var temp_constraint := PivotConstraint.new()
		temp_constraint.anchor_object = second_object
		temp_constraint.anchor_position = second_shift
		temp_constraint.relative_shift = first_shift
		temp_constraint.draw_constraint(first_object)

	if is_instance_valid(second_object):
		var temp_constraint := PivotConstraint.new()
		temp_constraint.anchor_object = first_object
		temp_constraint.anchor_position = first_shift
		temp_constraint.relative_shift = second_shift
		temp_constraint.draw_constraint(second_object)