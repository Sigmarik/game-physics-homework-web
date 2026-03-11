extends OptionButton

# Optional: choose what part of the selected item to store
enum StoreMode { TEXT, ID, METADATA }
@export var store_mode: StoreMode = StoreMode.ID
# Optional: custom name for the variable on the root node
@export var variable_name: String = "spring_solver_mode"

func _ready():
	# Connect the item_selected signal to our handler
	item_selected.connect(_on_item_selected)
	_on_item_selected(0)

func _on_item_selected(index: int):
	# Get the root node of the current scene
	var root = get_tree().current_scene
	if not root:
		push_error("No scene root found!")
		return

	# Determine which value to store based on store_mode
	var value_to_store
	match store_mode:
		StoreMode.TEXT:
			value_to_store = get_item_text(index)
		StoreMode.ID:
			value_to_store = get_item_id(index)
		StoreMode.METADATA:
			value_to_store = get_item_metadata(index)

	# Set the value as a custom property on the root node
	root.set_meta(variable_name, value_to_store)

	
