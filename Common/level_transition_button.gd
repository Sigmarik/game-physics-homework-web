extends Button

# Path to the scene you want to load (e.g., "res://Level2.tscn")
@export var next_level_scene: String = ""

func _ready():
	# Connect the button's pressed signal to our custom method
	pressed.connect(_on_button_pressed)

func _on_button_pressed():
	# Check if a scene path is provided
	if next_level_scene.is_empty():
		print("No scene path specified!")
		return
	
	# Change to the new scene
	get_tree().change_scene_to_file(next_level_scene)
	
