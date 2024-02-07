extends Control
class_name DebugUI

var text_to_draw := {}
var lines_to_draw := {}
var circles_to_draw := {}

var default_font = ThemeDB.fallback_font
var default_font_size = ThemeDB.fallback_font_size

var dirty := true

func _process(delta):
	if dirty:
		queue_redraw()
		dirty = false

func _draw():
	var current_camera :  Camera3D = get_viewport().get_camera_3d()
	
	for text in text_to_draw.values():
		if text["screen_space"]:
			draw_string(default_font, text["position"], text["text"], HORIZONTAL_ALIGNMENT_LEFT, -1.0, default_font_size, text["color"])
		else:
			draw_string(default_font, current_camera.unproject_position(text["position"]) + text["screen_offset"], text["text"], HORIZONTAL_ALIGNMENT_LEFT, -1.0, default_font_size, text["color"])
	
	for line in lines_to_draw.values():
		if line["screen_space"]:
			draw_line(line["position"], line["position"] + line["vector"], line["color"], 2.0)
		else:
			draw_line(current_camera.unproject_position(line["position"]) + line["screen_offset"], current_camera.unproject_position(line["position"] + line["vector"]) + line["screen_offset"], line["color"], 2.0)
	
	for circle in circles_to_draw.values():
		var circle_position = circle["position"]
		if not circle["screen_space"]:
			circle_position = current_camera.unproject_position(circle_position) + circle["screen_offset"]
		
		if circle["empty"]:
			draw_arc(circle_position, circle["radius"], 0, 2*PI, 100, circle["color"], 2.0)
		else:
			draw_circle(circle_position, circle["radius"], circle["color"])



func draw_debug_line(name : String, position, vector, color : Color, screen_space := false, screen_offset := Vector2(0,0)):
	var properties := {}
	properties["position"] = position
	properties["vector"] = vector
	properties["color"] = color
	properties["screen_space"] = screen_space
	properties["screen_offset"] = screen_offset
	lines_to_draw[name] = properties
	dirty = true

func draw_debug_circle(name : String, position, radius : float, color : Color, empty := false, screen_space := false, screen_offset := Vector2(0,0)):
	var properties := {}
	properties["position"] = position
	properties["radius"] = radius
	properties["color"] = color
	properties["empty"] = empty
	properties["screen_space"] = screen_space
	properties["screen_offset"] = screen_offset
	circles_to_draw[name] = properties
	dirty = true

func draw_debug_text(name : String, text : String, position, color : Color, screen_space := false, screen_offset := Vector2(0,0)):
	var properties := {}
	properties["position"] = position
	properties["text"] = text
	properties["color"] = color
	properties["screen_space"] = screen_space
	properties["screen_offset"] = screen_offset
	text_to_draw[name] = properties
	dirty = true

func clear_debug():
	text_to_draw = {}
	circles_to_draw = {}
	lines_to_draw = {}
	dirty = true
