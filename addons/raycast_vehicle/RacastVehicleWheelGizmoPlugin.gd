extends EditorSpatialGizmoPlugin
class_name RaycastVehicleWheelGizmoPlugin

var settings = EditorSettings.new()

func get_name():
	return "RaycastVehicleWheel"

func editor_def(var setting: String, var default):
	if settings.has_setting(setting):
		return settings.get(setting)
	else:
		return default

func has_gizmo(spatial):
	return spatial is RaycastVehicleWheel


func _init():
	var gizmo_color: Color = editor_def("editors/3d_gizmos/gizmo_colors/shape", Color(0.5, 0.7, 1))
	create_material("shape_material", gizmo_color)
	
	
func redraw(gizmo):
	var car_wheel: RaycastVehicleWheel = gizmo.get_spatial_node()
	
	gizmo.clear()
	
	var points := []
	
	var r := car_wheel.wheel_radius
	var rest := car_wheel.suspension_rest_length
	
	var skip := 10
	for i in range(0, 360, skip):
		var ra := deg2rad(i)
		var rb := deg2rad(i + skip)
		var a = Vector2(sin(ra), cos(ra)) * r
		var b = Vector2(sin(rb), cos(rb)) * r
		
		points.push_back(Vector3(0, a.x, a.y))
		points.push_back(Vector3(0, b.x, b.y))
		
		var spring_sec = 4
		
		for j in range(0, spring_sec):
			var t := rest * 5
			points.push_back(Vector3(a.x, i / 360.0 * t / spring_sec + j * (t / spring_sec), a.y) * 0.2)
			points.push_back(Vector3(b.x, (i + skip) / 360.0 * t / spring_sec + j * (t / spring_sec), b.y) * 0.2)
			
	
	#travel
	points.push_back(Vector3.ZERO)
	points.push_back(Vector3(0, rest, 0))
	
	#axis
	points.push_back(Vector3(r * 0.2, rest, 0))
	points.push_back(Vector3(r * -0.2, rest, 0))
	#axis
	points.push_back(Vector3(r * 0.2, 0, 0))
	points.push_back(Vector3(r * -0.2, 0, 0))
	
	#forward line
	points.push_back(Vector3(0, -r, 0))
	points.push_back(Vector3(0, -r, r * 2))
	points.push_back(Vector3(0, -r, r * 2))
	points.push_back(Vector3(r * 2 * 0.2, -r, r * 2 * 0.8))
	points.push_back(Vector3(0, -r, r * 2))
	points.push_back(Vector3(-r * 2 * 0.2, -r, r * 2 * 0.8))
	
	var material := get_material("shape_material", gizmo)
	
	gizmo.add_lines(points, material)
	gizmo.add_collision_segments(points)
	

# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.

# Called every frame. 'delta' is the elapsed time since the previous frame.
#func _process(delta):
#	pass
