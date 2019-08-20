tool
extends EditorPlugin

var raycast_vehicle_wheel_gizmo = RaycastVehicleWheelGizmoPlugin.new()

func _enter_tree():
	add_spatial_gizmo_plugin(raycast_vehicle_wheel_gizmo)

func _exit_tree():
	remove_spatial_gizmo_plugin(raycast_vehicle_wheel_gizmo)	
