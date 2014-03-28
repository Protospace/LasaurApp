Lasersaur = (function($, undefined) {
	"use strict";

	var SERIAL_CONNECT = 1;
	var SERIAL_DISCONNECT = 0;
	var SERIAL_IS_CONNECTED = 2;

	var gcode_buffer = [];

	function ajax_serial(method) {
		
	}

	function gcode_push(cmd, flush) {
		gcode_buffer.push(cmd);

		if(flush) {
			return gcode_flush();
		}
	}
	function gcode_clear() {
		gcode_buffer = [];
	}
	function gcode_flush() {
		if(gcode_buffer.length == 0) {
			return;
		}
		var gcode = gcode_buffer.join('\n');
		gcode_clear();

		return gcode_send(gcode);
	}
	function gcode_send(gcode) {
		return $.ajax(Lasersaur.domain + "/gcode", {
			type: "POST",
			dataType: "json",
			data: {'gcode_program':gcode}
		});
	}
	
	function parse_move(prefix, context) {
		if('x' in context) {
			prefix += "X" + context.x;
		}
		if('y' in context) {
			prefix += "Y" + context.y;
		}
		if('z' in context) {
			prefix += "Z" + context.z;
		}
		if('rate' in context) {
			prefix += "F" + context.rate;
		}
		return prefix;
	}

	return {
		domain: "",
		status: function() {
			return $.ajax(Lasersaur.domain + "/status", {
				dataType: "json"
			});
		},
		serial: {
			connect: function() {
				return ajax_serial(SERIAL_CONNECT);
			},
			disconnect: function() {
				return ajax_serial(SERIAL_DISCONNECT);
			},
			is_connected: function() {
				return ajax_serial(SERIAL_IS_CONNECTED);
			}
		},
		gcode: {
			clear: gcode_clear,
			flush: gcode_flush,
			seek: function(context) {
				return gcode_push(parse_move("G0", context));
			},
			feed: function() {
				return gcode_push(parse_move("G1", context));
			},
			intensity: function(intensity) {
				if(intensity < 0) {
					intensity = 0;
				}
				if(intensity > 255) {
					intensity = 255;
				}
				gcode_push("S" + intensity);
			},
			pierce: function(duration) {
				gcode_push("G4P" + duration);
			},
			home: function() {
				gcode_clear();
				return gcode_push("G30", true);
			},
			absolute: function() {return gcode_push("G90");},
			relative: function() {return gcode_push("G91");},
			aux0: function(state) {return gcode_push(state ? "M80" : "M81");},
			aux1: function(state) {return gcode_push(state ? "M82" : "M83");},

			raw: function(g) {return gcode_send(g);}
		},
	};
})(jQuery);
