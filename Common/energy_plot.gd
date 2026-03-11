extends Line2D

# Sampling settings
@export var sample_interval: float = 0.1          # seconds between samples
@export var max_samples: int = 100                # number of points to keep

# Y‑axis scaling
@export var auto_scale: bool = true                # if false, use fixed_min/max
@export var fixed_min: float = 0.0
@export var fixed_max: float = 1.0

# Plot dimensions in local coordinates
@export var plot_width: float = 500.0              # total width of the plot
@export var plot_height: float = 200.0             # total height
@export var x_offset: float = 0.0                   # left edge X
@export var y_offset: float = 0.0                   # bottom edge Y (energy rises upward)

# If true, point spacing is based on max_samples (fixed time window).
# If false, points always fill the whole plot width (scrolling adapts to current count).
@export var fixed_spacing: bool = true

var energy_history: Array[float] = []
var timer: Timer


func _ready():
    # Create a timer that triggers sampling
    timer = Timer.new()
    timer.wait_time = sample_interval
    timer.autostart = true
    timer.timeout.connect(_on_timer_timeout)
    add_child(timer)


func _on_timer_timeout():
    # Calculate total energy from the whole scene
    var total = _sum_energy(get_tree().current_scene)
    energy_history.append(total)

    # Keep only the last max_samples values
    if energy_history.size() > max_samples:
        energy_history.pop_front()

    update_plot()


# Recursively sum get_energy() from a node and all its children
func _sum_energy(node: Node) -> float:
    var total = 0.0
    if node.has_method("get_energy"):
        total += node.get_energy()
    for child in node.get_children():
        total += _sum_energy(child)
    return total


func update_plot():
    if energy_history.is_empty():
        clear_points()
        return

    # Determine Y range
    var min_val: float
    var max_val: float
    if auto_scale:
        min_val = energy_history.min()
        max_val = energy_history.max()
        # Avoid division by zero and give a little headroom
        if min_val == max_val:
            min_val -= 0.1
            max_val += 0.1
    else:
        min_val = fixed_min
        max_val = fixed_max

    # Build point array
    var points := PackedVector2Array()
    var count = energy_history.size()

    if fixed_spacing:
        # Fixed step based on max_samples – leftmost points may be missing
        var step = plot_width / (max_samples - 1) if max_samples > 1 else 0
        for i in range(count):
            var x = x_offset + i * step
            var t = (energy_history[i] - min_val) / (max_val - min_val) if max_val > min_val else 0.5
            var y = y_offset + plot_height * (1.0 - t)   # invert so larger values go upward
            points.append(Vector2(x, y))
    else:
        # Adaptive spacing – points always fill the whole width
        var step = plot_width / (count - 1) if count > 1 else 0
        for i in range(count):
            var x = x_offset + i * step
            var t = (energy_history[i] - min_val) / (max_val - min_val) if max_val > min_val else 0.5
            var y = y_offset + plot_height * (1.0 - t)
            points.append(Vector2(x, y))

    self.points = points
