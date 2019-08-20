tool
extends RigidBody
class_name RaycastVehicleBody, "res://addons/raycast_vehicle/icons/icon_vehicle_body.svg"

var _pitch_control: float
var _steering_value: float
var _current_vehicle_speed_kph: float

var _exclude := []
var _forward_ws := []
var _axle := []
var _forward_impulse := []
var _side_impulse := []

var wheels := []

var steering := 0.0 setget set_steering
var engine_force := 0.0 setget set_engine_force
var brake := 0.0 setget set_brake


const CMP_EPSILON := 0.00001

class RaycastVehicleJacobianEntry:
	var linear_joint_axis: Vector3
	var aJ: Vector3
	var bJ: Vector3
	var minvJt0: Vector3
	var minvJt1: Vector3
	var a_diag: float
	func get_diagonal() -> float:
		return a_diag
	

	#constraint between two different rigidbodies
	func _init(var world2A: Basis,
				var world2B: Basis,
				var rel_pos1: Vector3,
				var rel_pos2: Vector3,
				var joint_axis: Vector3,
				var inertia_invA: Vector3,
				var mass_invA: float,
				var inertia_invB: Vector3,
				var mass_invB):
		linear_joint_axis = joint_axis
		aJ = world2A.xform(rel_pos1.cross(linear_joint_axis))
		bJ = world2B.xform(rel_pos2.cross(linear_joint_axis))
		minvJt0 = inertia_invA * aJ
		minvJt1 = inertia_invB * bJ
		a_diag = mass_invA + minvJt0.dot(aJ) + mass_invB + minvJt1.dot(bJ)
	
	func get_relative_velocity(var linvelA: Vector3, var angvelA: Vector3, var linvelB: Vector3, var angvelB: Vector3) -> float:
		var linrel := linvelA - linvelB
		var angvela = angvelA * aJ
		var angvelb = angvelB * bJ
		linrel *= linear_joint_axis
		angvela += angvelb
		angvela += linrel
		return angvela[0] + angvela[1] + angvela[2] + CMP_EPSILON
	

class RaycastVehicleWheelContactPoint:
	var s: PhysicsDirectBodyState
	var body1: PhysicsBody
	var friction_position_world: Vector3
	var friction_direction_world: Vector3
	var jac_diag_ab_inv: float
	var max_impulse: float
	
	func get_inverse_inertia_tensor(var s: PhysicsDirectBodyState) -> Basis:
		return s.transform.basis.scaled(s.inverse_inertia) * s.transform.basis.transposed()
	
	func _init(var _s: PhysicsDirectBodyState, var _body1: PhysicsBody, var _friction_position_world: Vector3, var _friction_direction_world: Vector3, var _max_impulse: float):
		s = _s
		body1 = _body1
		friction_position_world = _friction_position_world
		friction_direction_world = _friction_direction_world
		max_impulse = _max_impulse
		
		var denom0 := 0.0
		var denom1 := 0.0
		
		var r0 := friction_position_world - s.transform.origin
		var c0 := r0.cross(friction_direction_world)
		var vec := get_inverse_inertia_tensor(s).xform_inv(c0).cross(r0);
		denom0 = s.inverse_mass + friction_direction_world.dot(vec)
		var relaxation := 1.0
		jac_diag_ab_inv = relaxation / (denom0 + denom1)

class RayResult:
	var result: bool
	var position: Vector3
	var normal: Vector3
	var collider: PhysicsBody
	func _init(var r: Dictionary):
		if r.size() > 0:
			result = true
			position = r["position"]
			normal = r["normal"]
			collider = r["collider"]
		else:
			result = false

func _resolve_single_bilateral(var s: PhysicsDirectBodyState, var pos1: Vector3, var body2: PhysicsBody, var pos2: Vector3, var normal: Vector3, var roll_influence: float) -> float:
	var impulse: float
	var normal_len_sqr = normal.length_squared()
	
	if normal_len_sqr > 1.1:
		return 0.0
		
	var rel_pos1: Vector3 = pos1 - s.transform.origin
	var rel_pos2: Vector3 = pos2 - body2.global_transform.origin if body2 else Vector3.ZERO
	
	var vel1: Vector3 = s.linear_velocity + s.angular_velocity.cross(rel_pos1)
	var vel2: Vector3 = body2.linear_velocity if body2 is RigidBody else Vector3.ZERO 
	
	var vel = vel1 - vel2
	
	var b2_trans: Basis
	var b2_inv_mass := 0.0
	var b2_lv: Vector3
	var b2_av: Vector3
	var b2_inv_inertia: Vector3 #TODO
	
	if body2 is RigidBody:
		b2_trans = body2.global_transform.basis.transposed()
		b2_inv_mass = 1.0 / body2.mass
		b2_lv = body2.linear_velocity
		b2_av = body2.angular_velocity
	
	#TODO: need inverse inertia tensor!
	#	btVehicleJacobianEntry jac(s->get_transform().basis.transposed(),
	#		b2trans,
	#		rel_pos1,
	#		rel_pos2,
	#		normal,
	#		s->get_inverse_inertia_tensor().get_main_diagonal(),
	#		1.0 / mass,
	#		b2invinertia,
	#		b2invmass);
	# FIXME: rel_vel assignment here is overwritten by the following assignment.
	# What seems to be intended in the next next assignment is: rel_vel = normal.dot(rel_vel);
	# Investigate why.
	#real_t rel_vel = jac.getRelativeVelocity(
	#		s->get_linear_velocity(),
	#		s->get_transform().basis.transposed().xform(s->get_angular_velocity()),
	#		b2lv,
	#		b2trans.xform(b2av));
	
	var rel_vel := normal.dot(vel)
	
	var contact_damping := 0.2
	
	if roll_influence > 0.0:
		# But seeing we apply this frame by frame, makes more sense to me to make this time based
		# keeping in mind our anti roll factor if it is set
		contact_damping = min(contact_damping, s.step / roll_influence)
	
	var mass_term = 1.0 / ((1.0 / mass) + b2_inv_mass)
	impulse = -contact_damping * rel_vel * mass_term
	
	#real_t velocityImpulse = -contactDamping * rel_vel * jacDiagABInv;
	#impulse = velocityImpulse;

	return impulse


func _calc_rolling_friction(var contact_point: RaycastVehicleWheelContactPoint) -> float:
	var j1 := 0.0
	
	var contact_pos_world := contact_point.friction_position_world
	
	var rel_pos1 := contact_pos_world - contact_point.s.transform.origin
	var rel_pos2: Vector3 = contact_pos_world - contact_point.body1.global_transform.origin if contact_point.body1 else Vector3.ZERO
	
	var max_impulse := contact_point.max_impulse
	
	var vel1 := contact_point.s.linear_velocity + contact_point.s.angular_velocity.cross(rel_pos1)
	var vel2: Vector3 = contact_point.body1.linear_velocity + contact_point.body1.angular_velocity.cross(rel_pos2) if contact_point.body1 is RigidBody else Vector3.ZERO
	
	var vel := vel1 - vel2
	
	var vrel := contact_point.friction_direction_world.dot(vel)
	
	# calculate j that moves us to zero relative velocity
	j1 = -vrel * contact_point.jac_diag_ab_inv
	
	return clamp(j1, -max_impulse, max_impulse)

const SIDE_FRICTION_STIFFNESS2 := 1.0
func _update_friction(var s: PhysicsDirectBodyState) -> void:
	
	#calculate the impulse, so that the wheels don't move sidewards
	var num_wheel = wheels.size()
	if num_wheel == 0:
		return
	
	_forward_ws.resize(num_wheel)
	_axle.resize(num_wheel)
	_forward_impulse.resize(num_wheel)
	_side_impulse.resize(num_wheel)
	
	for i in num_wheel:
		_side_impulse[i] = 0.0
		_forward_impulse[i] = 0.0
	
	for i in num_wheel:
		var wheel_info = wheels[i] #as RaycastVehicleWheel
		
		if wheel_info._raycast_info.is_in_contact:
			var wheel_basis0 = wheel_info._world_transform.basis
			
			_axle[i] = wheel_basis0.x
			
			var surf_normal_ws = wheel_info._raycast_info.contact_normal_ws
			var proj: Vector3 = _axle[i].dot(surf_normal_ws)
			_axle[i] -= surf_normal_ws * proj
			_axle[i] = _axle[i].normalized()
			
			_forward_ws[i] = surf_normal_ws.cross(_axle[i]).normalized()
			
			_side_impulse[i] = _resolve_single_bilateral(s, wheel_info._raycast_info.contact_point_ws,
				wheel_info._raycast_info.ground_object, wheel_info._raycast_info.contact_point_ws,
				_axle[i], wheel_info.roll_influence)
				
			_side_impulse[i] *= SIDE_FRICTION_STIFFNESS2
	
	var side_factor := 1.0
	var fwd_factor := 0.5
	
	var sliding := false
	for i in num_wheel:
		var wheel_info = wheels[i]# as RaycastVehicleWheel
		
		var rolling_friction := 0.0
		
		if wheel_info._raycast_info.is_in_contact:
			if wheel_info.engine_force != 0.0:
				rolling_friction = -wheel_info.engine_force * s.step
			else:
				var default_rolling_friction_impulse := 0.0
				var max_impulse = wheel_info.brake if wheel_info.brake else default_rolling_friction_impulse
				var contact_pt = RaycastVehicleWheelContactPoint.new(s, wheel_info._raycast_info.ground_object, wheel_info._raycast_info.contact_point_ws, _forward_ws[i], max_impulse)
				rolling_friction = _calc_rolling_friction(contact_pt)
	
		#switch between active rolling (throttle), braking and non-active rolling friction (no throttle/brake)
		
		_forward_impulse[i] = 0.0
		wheel_info._skid_info = 1.0
		
		if wheel_info._raycast_info.is_in_contact:
			wheel_info._skid_info = 1.0
			
			var max_imp = wheel_info._wheels_suspension_force * s.step * wheel_info.friction_slip
			var max_imp_side = max_imp
			
			var max_imp_squared = max_imp * max_imp_side
			
			_forward_impulse[i] = rolling_friction
			
			var x: float = _forward_impulse[i] * fwd_factor
			var y: float = _side_impulse[i] * side_factor
			
			var impulse_squared := x * x + y * y
			
			if impulse_squared > max_imp_squared:
				sliding = true
				var factor = max_imp / sqrt(impulse_squared)
				wheel_info._skid_info *= factor
	
	if sliding:
		for i in num_wheel:
			if _side_impulse[i] != 0.0:
				if wheels[i]._skid_info < 1.0:
					_forward_impulse[i] *= wheels[i]._skid_info
					_side_impulse[i] *= wheels[i]._skid_info
					
	# apply the impulses
	
	for i in num_wheel:
		var wheel_info = wheels[i] #as RaycastVehicleWheel
		
		var rel_pos = wheel_info._raycast_info.contact_point_ws - s.transform.origin
		
		if _forward_impulse[i] != 0.0:
			s.apply_impulse(rel_pos, _forward_ws[i] * _forward_impulse[i])
		if _side_impulse[i] != 0.0:
			var ground_object = wheel_info._raycast_info.ground_object
			var rel_pos2: Vector3 = wheel_info._raycast_info.contact_point_ws - ground_object.global_transform.origin if ground_object else Vector3.ZERO
			
			var side_imp = _axle[i] * _side_impulse[i]
			var chassis_world_up = s.transform.basis.transposed().y
			rel_pos -= chassis_world_up * chassis_world_up.dot(rel_pos) * (1.0 - wheel_info.roll_influence)
			
			s.apply_impulse(rel_pos, side_imp)
			
			#apply friction impulse on the ground
			#todo
			#groundObject->applyImpulse(-sideImp,rel_pos2);

	
	
func _update_suspension(var s: PhysicsDirectBodyState) -> void:
	var chassis_mass = mass
	for w_it in wheels:
		var wheel_info = w_it #as RaycastVehicleWheel
		
		if wheel_info._raycast_info.is_in_contact:
			var force: float
			#Spring
			var susp_length = wheel_info.suspension_rest_length
			var current_length = wheel_info._raycast_info.suspension_length
			var length_diff = susp_length - current_length
			force = wheel_info.suspension_stiffness * length_diff * wheel_info._clipped_inv_contact_dot_suspension
			
			#Damper
			var projected_rel_vel = wheel_info._suspension_relative_velocity
			var susp_damping = wheel_info.wheels_damping_compression if projected_rel_vel < 0.0 else wheel_info.wheels_damping_relaxation
			force -= susp_damping * projected_rel_vel
			
			#Result
			wheel_info._wheels_suspension_force = force * chassis_mass
			if wheel_info._wheels_suspension_force < 0.0:
				wheel_info._wheels_suspension_force = 0.0
		else:
			wheel_info._wheels_suspension_force = 0.0
	
	
func _ray_cast(var idx: int, var s: PhysicsDirectBodyState) -> float:
	var wheel = wheels[idx] #as RaycastVehicleWheel
	_update_wheel_transform(wheel, s)
	
	var depth := -1.0
	var ray_len = wheel.suspension_rest_length + wheel.wheel_radius
	
	var ray_vector = wheel._raycast_info.wheel_direction_ws * ray_len
	var source = wheel._raycast_info.hard_point_ws
	wheel._raycast_info.contact_point_ws = source + ray_vector
	var target = wheel._raycast_info.contact_point_ws
	source -= wheel.wheel_radius * wheel._raycast_info.wheel_direction_ws
	
	var param := 0.0
	
	var ss := s.get_space_state()
	var rr := RayResult.new(ss.intersect_ray(source, target, _exclude))
	
	wheel._raycast_info.ground_object = null
	
	if rr.result:
		param = source.distance_to(rr.position) / source.distance_to(target)
		depth = ray_len * param
		wheel._raycast_info.contact_normal_ws = rr.normal
		
		wheel._raycast_info.is_in_contact = true
		wheel._raycast_info.ground_object = rr.collider
		
		var hit_distance = param * ray_len
		wheel._raycast_info.suspension_length = hit_distance - wheel.wheel_radius
		
		#clamp on max suspension travel
		var min_suspension_length = wheel.suspension_rest_length - wheel.max_suspension_travel
		var max_suspension_length = wheel.suspension_rest_length + wheel.max_suspension_travel
		clamp(wheel._raycast_info.suspension_length, min_suspension_length, max_suspension_length)
		
		wheel._raycast_info.contact_point_ws = rr.position
		
		var denominator = wheel._raycast_info.contact_normal_ws.dot(wheel._raycast_info.wheel_direction_ws)
		
		var chassis_velocity_at_contact_point := s.linear_velocity + s.angular_velocity.cross(wheel._raycast_info.contact_point_ws - s.transform.origin)
		
		var proj_vel = wheel._raycast_info.contact_normal_ws.dot(chassis_velocity_at_contact_point)
		
		if denominator > -0.1:
			wheel._suspension_relative_velocity = 0.0
			wheel._clipped_inv_contact_dot_suspension = 1.0 / 0.1
		else:
			var inv = -1.0 / denominator
			wheel._suspension_relative_velocity = proj_vel * inv
			wheel._clipped_inv_contact_dot_suspension = inv
	
	else:
		wheel._raycast_info.is_in_contact = false
		#put wheel info as in rest position
		wheel._raycast_info.suspension_length = wheel.suspension_rest_length
		wheel._suspension_relative_velocity = 0.0
		wheel._raycast_info.contact_normal_ws = -wheel._raycast_info.wheel_direction_ws
		wheel._clipped_inv_contact_dot_suspension = 1.0
	
	return depth

	
func _update_wheel_transform(var wheel, var s: PhysicsDirectBodyState) -> void:
	wheel._raycast_info.is_in_contact = false
	var chassis_trans := s.transform
	wheel._raycast_info.hard_point_ws = chassis_trans.xform(wheel._chassis_connection_point_cs)
	wheel._raycast_info.wheel_direction_ws = chassis_trans.basis.xform(wheel._wheel_direction_cs).normalized()
	wheel._raycast_info.wheel_axle_ws = chassis_trans.basis.xform(wheel._wheel_axle_cs).normalized()
	
	
func _update_wheel(var idx: int, var s: PhysicsDirectBodyState) -> void:
	var wheel = wheels[idx] #as RaycastVehicleWheel
	_update_wheel_transform(wheel, s)
	
	var up: Vector3 = -wheel._raycast_info.wheel_direction_ws
	var right: Vector3 = wheel._raycast_info.wheel_axle_ws
	var fwd := up.cross(right).normalized()
	
	var steering_mat := Basis(up, wheel.steering)
	var rotating_mat := Basis(right, wheel._rotation)
	var basis2 := Basis(right, up, fwd)
	
	wheel._world_transform.basis = steering_mat * rotating_mat * basis2
	wheel._world_transform.origin = wheel._raycast_info.hard_point_ws + wheel._raycast_info.wheel_direction_ws * wheel._raycast_info.suspension_length
	

func _integrate_forces(var state: PhysicsDirectBodyState) -> void:
	var step := state.step
	
	for i in wheels.size():
		_update_wheel(i, state)
	
	for i in wheels.size():
		_ray_cast(i, state)
		wheels[i].transform = state.transform.inverse() * wheels[i]._world_transform
	
	_update_suspension(state)
	
	for i in wheels.size():
		#apply suspension force
		var wheel = wheels[i]# as RaycastVehicleWheel
		
		var suspension_force: float = wheel._wheels_suspension_force
		if suspension_force > wheel.max_suspension_force:
			suspension_force = wheel.max_suspension_force
		
		var impulse: Vector3 = wheel._raycast_info.contact_normal_ws * suspension_force * step
		var rel_pos: Vector3 = wheel._raycast_info.contact_point_ws - state.transform.origin
		
		state.apply_impulse(rel_pos, impulse)
	
	_update_friction(state)
	
	for i in wheels.size():
		var wheel = wheels[i]# as RaycastVehicleWheel
		var rel_pos: Vector3 = wheel._raycast_info.hard_point_ws - state.transform.origin
		var vel := state.linear_velocity + state.angular_velocity.cross(rel_pos)
		
		if wheel._raycast_info.is_in_contact:
			var chassis_world_transform := state.transform
			var fwd := Vector3(chassis_world_transform.basis.x.z, chassis_world_transform.basis.y.z, chassis_world_transform.basis.z.z)
			
			var proj := fwd.dot(wheel._raycast_info.contact_normal_ws)
			fwd -= wheel._raycast_info.contact_normal_ws * proj
			
			var proj2 := fwd.dot(vel)
			
			wheel._delta_rotation = proj2 * step / wheel.wheel_radius
		wheel._rotation += wheel._delta_rotation
		wheel._rpm = (wheel._delta_rotation / step) * 60 / TAU
		wheel._delta_rotation *= 0.99 #damping of rotation when not in contact

func _ready():
	_exclude.append(get_rid())



func set_steering(value : float) -> void:
	steering = value
	for w in wheels:
		if w.steers:
			w.steering = value

func set_engine_force(value : float) -> void:
	engine_force = value
	for w in wheels:
		if w.engine_traction:
			w.engine_force = value

func set_brake(value : float) -> void:
	brake = value
	for w in wheels:
		w.brake = value


func _get_property_list():
	return [
		{
			"name": "RaycastVehicleBody",
			"type": TYPE_NIL,
			"usage": PROPERTY_USAGE_CATEGORY
		},
		{
			"name": "Motion",
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
	]


func _get(property: String):
	match property:
		"engine_force":
			return engine_force
		"brake":
			return brake
		"steering":
			return steering
			
			
func _set(property: String, value):
	if !(value is float):
		return
	
	match property:
		"engine_force":
			set_engine_force(value)
		"steering":
			set_steering(value)
		"brake":
			set_brake(value)
