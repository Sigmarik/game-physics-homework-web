extends Area2D

var dragging = false
var drag_offset : Vector2

func _input_event(viewport, event, shape_idx):
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT:
		if event.pressed:
			dragging = true
			drag_offset = get_global_mouse_position() - global_position
		else:
			dragging = false

func _process(delta):
	if dragging:
		global_position = get_global_mouse_position() - drag_offset
		
