tool
extends Spatial
class_name RaycastVehicleWheel, "res://addons/raycast_vehicle/icons/icon_vehicle_wheel.svg"

var engine_traction := false
var steers := false

var suspension_rest_length := 0.15
var max_suspension_travel := 0.5
var wheel_radius := 0.5

var suspension_stiffness := 5.88
var wheels_damping_compression := 0.83
var wheels_damping_relaxation := 0.88
var friction_slip := 10.5
var max_suspension_force := 6000.0

var steering := 0.0
var roll_influence := 0.1
var engine_force := 0.0
var brake := 0.0


var _world_transform: Transform
var _local_xform: Transform


var _chassis_connection_point_cs: Vector3		#const
var _wheel_direction_cs: Vector3				#const
var _wheel_axle_cs: Vector3					#const or modified by steering


var _is_front_wheel := false

var _body#: RaycastVehicle

var _rotation := 0.0
var _delta_rotation := 0.0
var _rpm: float

var _clipped_inv_contact_dot_suspension := 1.0
var _suspension_relative_velocity := 0.0
#calculated by suspension
var _wheels_suspension_force: float
var _skid_info: float

class RaycastInfo:
	var contact_normal_ws: Vector3		#contact normal
	var contact_point_ws: Vector3		#raycast hitpoint
	var suspension_length: float
	var hard_point_ws: Vector3			#raycast starting point
	var wheel_direction_ws: Vector3	#direction in worldspace
	var wheel_axle_ws: Vector3			#axle in worldspace
	var is_in_contact: bool
	var ground_object: PhysicsBody

var _raycast_info: RaycastInfo = RaycastInfo.new()


func _update(var s: PhysicsDirectBodyState) -> void:
	if _raycast_info.is_in_contact:
		var project := _raycast_info.contact_normal_ws.dot(_raycast_info.wheel_direction_ws)
		var relpos := _raycast_info.contact_point_ws - s.transform.origin
		
		var chassis_velocity_at_contact_point := s.linear_velocity + s.angular_velocity.cross(relpos)
		
		var proj_vel := _raycast_info.contact_normal_ws.dot(chassis_velocity_at_contact_point)
		if project >= -0.1:
			_suspension_relative_velocity = 0.0
			_clipped_inv_contact_dot_suspension = 1.0 / 0.1
		else:
			var inv := -1.0 / project
			_suspension_relative_velocity = proj_vel * inv
			_clipped_inv_contact_dot_suspension = inv
	
	else: # Not in contact : position wheel in a nice (rest length) position
		_raycast_info.suspension_length = suspension_rest_length
		_suspension_relative_velocity = 0.0
		_raycast_info.contact_normal_ws = -_raycast_info.wheel_direction_ws
		_clipped_inv_contact_dot_suspension = 1.0


func _enter_tree():
	var cb = get_parent()
	if !cb:
		return
	_body = cb
	_local_xform = get_transform();
	cb.wheels.append(self)
	
	_chassis_connection_point_cs = transform.origin
	_wheel_direction_cs = -transform.basis.y.normalized()
	_wheel_axle_cs = transform.basis.x.normalized()
	pass


func _exit_tree():
	var cb := get_parent()# as RaycastVehicle
	if !cb:
		return
	cb.wheels.erase(self)
	_body = null


func is_in_contact() -> bool:
	return _raycast_info.is_in_contact


func get_skidinfo() -> float:
	return _skid_info
	
	
func get_rpm() -> float:
	return _rpm


func _get_configuration_warning() -> String:
#	if not get_parent().is_class("RaycastVehicleBody"):
#		return "RaycastVehicleWheel serves to provide a wheel system to a RaycastVehicleBody. Please use it as a child of a RaycastVehicleBody, not a " + get_parent().get_
	return ""

func _get_property_list():
	return [
		{
			"name": "RaycastVehicleWheel",
			"type": TYPE_NIL,
			"usage": PROPERTY_USAGE_CATEGORY
		},
		{
			"name": "Per-Wheel Motion",
			"type": TYPE_NIL,
			"usage": PROPERTY_USAGE_GROUP
		},
		{
			"name": "engine_force",
			"type": TYPE_REAL
		},
		{
			"name": "brake",
			"type": TYPE_REAL
		},
		{
			"name": "steering",
			"type": TYPE_REAL
		},
		{
			"name": "Vehicle Body Motion",
			"type": TYPE_NIL,
			"usage": PROPERTY_USAGE_GROUP
		},
		{
			"name": "use_as_traction",
			"type": TYPE_BOOL
		},
		{
			"name": "use_as_steering",
			"type": TYPE_BOOL
		},
		{
			"name": "Wheel",
			"type": TYPE_NIL,
			"usage": PROPERTY_USAGE_GROUP
		},
		{
			"name": "roll_influence",
			"type": TYPE_REAL,
			"hint": PROPERTY_HINT_RANGE,
			"hint_string": "0.0,1.0,0.01,allow_greater",
		},
		{
			"name": "radius",
			"type": TYPE_REAL,
			"hint": PROPERTY_HINT_RANGE,
			"hint_string": "0.0,1.0,0.01,allow_greater",
		},
		{
			"name": "rest_length",
			"type": TYPE_REAL,
			"hint": PROPERTY_HINT_RANGE,
			"hint_string": "0.0,1.0,0.01,allow_greater",
		},
		{
			"name": "friction_slip",
			"type": TYPE_REAL,
			"hint": PROPERTY_HINT_RANGE,
			"hint_string": "0.0,1.0,0.01,allow_greater",
		},
		{
			"name": "Suspension",
			"type": TYPE_NIL,
			"usage": PROPERTY_USAGE_GROUP
		},
		{
			"name": "travel",
			"type": TYPE_REAL,
			"hint": PROPERTY_HINT_RANGE,
			"hint_string": "0.0,1.0,0.01,allow_greater",
		},
		{
			"name": "stiffness",
			"type": TYPE_REAL,
			"hint": PROPERTY_HINT_RANGE,
			"hint_string": "0.0,100.0,1.0,allow_greater",
		},
		{
			"name": "max_force",
			"type": TYPE_REAL,
			"hint": PROPERTY_HINT_RANGE,
			"hint_string": "0.0,10000.0,1.0,allow_greater",
		},
		{
			"name": "damping",
			"type": TYPE_NIL,
			"usage": PROPERTY_USAGE_GROUP
		},
		{
			"name": "compression",
			"type": TYPE_REAL,
			"hint": PROPERTY_HINT_RANGE,
			"hint_string": "0.0,1.0,0.01,allow_greater",
		},
		{
			"name": "relaxation",
			"type": TYPE_REAL,
			"hint": PROPERTY_HINT_RANGE,
			"hint_string": "0.0,1.0,0.01,allow_greater",
		},
	]

func _get(property: String):
	match property:
		"engine_force":
			return engine_force
		"brake":
			return brake
		"steering":
			return steering
		"use_as_traction":
			return engine_traction
		"use_as_steering":
			return steers
		"roll_influence":
			return roll_influence
		"radius":
			return wheel_radius
		"rest_length":
			return suspension_rest_length
		"friction_slip":
			return friction_slip
		"travel":
			return max_suspension_travel
		"stiffness":
			return suspension_stiffness
		"max_force":
			return max_suspension_force
		"compression":
			return wheels_damping_compression
		"relaxation":
			return wheels_damping_relaxation

func _set(property: String, value):
	match property:
		"engine_force":
			engine_force = value
		"brake":
			brake = value
		"steering":
			steering = value
		"use_as_traction":
			engine_traction = value
		"use_as_steering":
			steers = value
		"roll_influence":
			roll_influence = value
		"radius":
			wheel_radius = value
		"rest_length":
			suspension_rest_length = value
		"friction_slip":
			friction_slip = value
		"travel":
			max_suspension_travel = value
		"stiffness":
			suspension_stiffness = value
		"max_force":
			max_suspension_force = value
		"compression":
			wheels_damping_compression = value
		"relaxation":
			wheels_damping_relaxation = value

	
