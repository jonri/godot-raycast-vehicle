tool
extends RaycastVehicleBody

# Declare member variables here. Examples:
# var a = 2
# var b = "text"

# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	if (Input.is_action_pressed("ui_left")):
		
		#Per-wheel steering
		#$WheelLF.steering = 0.3
		#$WheelRF.steering = 0.3
		#$WheelLB.steering = -0.3
		#$WheelRB.steering = -0.3
		
		#Simple steering
		self.steering = 0.3
		
	elif (Input.is_action_pressed("ui_right")):
		
		#Per-wheel steering
		#$WheelLF.steering = -0.3
		#$WheelRF.steering = -0.3
		#$WheelLB.steering = 0.3
		#$WheelRB.steering = 0.3
		
		#Simple steering
		self.steering = -0.3
		
	else:

		#Per-wheel steering
		#$WheelLF.steering = 0.0
		#$WheelRF.steering = 0.0
		#$WheelLB.steering = 0.0
		#$WheelRB.steering = 0.0
		
		#Simple steering
		self.steering = 0.0
	
	if (Input.is_action_pressed("ui_up")):
		self.engine_force = 1000.0
	else:
		self.engine_force = 0.0
	
	if (Input.is_action_pressed("ui_down")):
		$WheelLB.brake = 100.0
		$WheelRB.brake = 100.0
	else:
		$WheelLB.brake = 0.0
		$WheelRB.brake = 0.0


