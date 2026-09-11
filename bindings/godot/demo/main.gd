# poncelet Godot demo — fire a catalog round, draw the live path + the aim arc.
# SPDX-License-Identifier: MIT
#
# Scene: a Node3D `Main` with this script, a Camera3D, and (optional) some
# MeshInstance3D scenery. main.tscn is not committed — add the node + script and
# press play; every ~1.2 s it fires a .308 downrange and prints its state.
extends Node3D

var sim := PonceletSim.new()
var rifle := -1
var shots: Array[int] = []
var _t := 0.0

@onready var live_line: ImmediateMesh = ImmediateMesh.new()
@onready var arc_line: ImmediateMesh = ImmediateMesh.new()


func _ready() -> void:
	sim.set_atmosphere(0.0)                       # sea-level ISA
	rifle = sim.register_catalog("762x51_175gr_smk")
	if rifle < 0:
		push_error("catalog round not found — is poncelet's data baked in?")

	for m in [live_line, arc_line]:
		var mi := MeshInstance3D.new()
		mi.mesh = m
		var mat := StandardMaterial3D.new()
		mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		mat.albedo_color = Color.YELLOW if m == arc_line else Color.RED
		mi.material_override = mat
		add_child(mi)


func _physics_process(delta: float) -> void:
	sim.step(delta)

	_t += delta
	if _t >= 1.2 and rifle >= 0:
		_t = 0.0
		var muzzle := Vector3(0, 1.7, 0)
		var aim := Vector3(1, 0.02, 0)              # a hair above flat
		shots.append(sim.fire(rifle, muzzle, aim))
		_draw_arc(muzzle, aim)

	_draw_live()
	for id in shots:
		if id >= 0 and sim.is_alive(id) and int(_t * 60) % 30 == 0:
			print(sim.describe(id))


func _draw_live() -> void:
	live_line.clear_surfaces()
	live_line.surface_begin(Mesh.PRIMITIVE_LINES)
	for id in shots:
		if id >= 0 and sim.is_alive(id):
			var p: Vector3 = sim.get_position(id)
			live_line.surface_add_vertex(p)
			live_line.surface_add_vertex(p + sim.get_velocity(id).normalized() * 2.0)
	live_line.surface_end()


func _draw_arc(muzzle: Vector3, aim: Vector3) -> void:
	var pts: PackedVector3Array = sim.preview_arc(rifle, muzzle, aim, 0.05, 6.0, 0.0)
	arc_line.clear_surfaces()
	if pts.size() < 2:
		return
	arc_line.surface_begin(Mesh.PRIMITIVE_LINE_STRIP)
	for p in pts:
		arc_line.surface_add_vertex(p)
	arc_line.surface_end()
