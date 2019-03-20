/**
 * redis-web-gui.js
 *
 * Author: Toki Migimatsu
 * Created: December 2017
 */

var KEY_PREFIX   = "cs223a::dh::";
var KEY_Q        = "q";
var KEY_T_EE     = "T_ee_to_0";
var KEY_DH_TABLE = "dh_table";
var KEY_ROBOT    = "robot";
var AXIS_WIDTH = 0.005;
var AXIS_SIZE  = 0.1;
var FRAME_OFFSET = 0.2;
var ROTATION_RADIUS = 0.03;

$(document).ready(function() {

	// Set up web socket
	var url_parser = document.createElement("a");
	url_parser.href = window.location.href;
	var ws_ip = url_parser.hostname;
	var ws = new WebSocket("ws://" + ws_ip + ":" + ws_port);

	ws.onopen = function() {
		console.log("Web socket connection established.");
	};

	var camera, scene, renderer;

	var updated = false;
	var animation_timer;
	var bodies = [];
	var robot = null, new_robot = null;

	ws.onmessage = function(e) {
		var msg = JSON.parse(e.data);

		// Update requested
		msg.forEach(function(m) {
			var matches = m[0].match("^" + KEY_PREFIX + "(.*)$");
			if (matches === null) return;
			var key = matches[1];
			var val = m[1];

			if (key == "set") {
				// Flag when all keys have been updated
				updated = (val == "");
				return;
			} else if (key == KEY_ROBOT) {
				new_robot = JSON.parse(val);
				var is_new_robot = false;
				if (robot != null) {
					var types = robot.links.map(link => link.type);
					var new_types = new_robot.links.map(link => link.type);
					is_new_robot = types.toString() != new_types.toString();
				}
				if (robot === null || is_new_robot) {
					delete_robot();
					robot = new_robot;
					new_robot = null;
					create_robot(robot);
				}
				return;
			}

			var $form = $("form[data-key='" + key + "']");

			// Create new redis key-value form
			if ($form.length == 0) {
				var form;
				if (key == KEY_DH_TABLE) {
					val = val.split(";").map(el => el.split(","));
					form = htmlForm(key, val);
					form = form.replace(/NaN/g,"");
				} else if (key == KEY_T_EE) {
					form = htmlForm(key, val, true);
				} else {
					form = htmlForm(key, val);
				}
				var $form = $(form).hide();
				var li = "<a href='#" + key + "' title='" + key + "'><li>" + key + "</li></a>";
				var $li = $(li).hide();

				// Find alphabetical ordering
				var keys = $("form").map(function() {
					return $(this).attr("data-key");
				}).get();
				var idx_key;
				for (idx_key = 0; idx_key < keys.length; idx_key++) {
					if (key < keys[idx_key]) break;
				}
				if (idx_key < keys.length) {
					$("form").eq(idx_key).before($form);
					$("#left-col a").eq(idx_key).before($li);
				} else {
					$("#sidebar-keys").append($form);
					$("#left-col ul").append($li)
				}
				$form.slideDown("normal");
				$li.slideDown("normal");

				return;
			}

			// Update html
			if (key == "dh_table") {
				val = val.split(";").map(el => el.split(","));
				var form = htmlForm(key, val);
				form = form.replace(/NaN/g,"");
				$form.html(form);
			} else {
				updateHtmlValues($form, val);
			}

		});

		if (updated && new_robot) {
			// Animate rotation
			var i = 1;
			if (animation_timer) {
				clearInterval(animation_timer);
			}
			var temp_robot = JSON.parse(JSON.stringify(robot));
			var new_quats = []
			for (var l = 0; l < new_robot.links.length; l++) {
				var new_quat = new_robot.links[l].quat;
				new_quats.push(new THREE.Quaternion(new_quat[0], new_quat[1], new_quat[2], new_quat[3]));
			}
			animation_timer = setInterval(function() {
				if (i > 100) {
					clearInterval(animation_timer);
					robot = new_robot;
					new_robot = null;
					return;
				}
				var j = i / 100;

				for (var l = 0; l < temp_robot.links.length; l++) {
					var temp_link = temp_robot.links[l];
					var link = robot.links[l];
					var new_link = new_robot.links[l];
					for (var k = 0; k < 3; k++) {
						temp_link.pos[k] = (1 - j) * link.pos[k] + j * new_link.pos[k];
					}
					var quat = new THREE.Quaternion(link.quat[0], link.quat[1], link.quat[2], link.quat[3]);
					quat.slerp(new_quats[l], j);
					temp_link.quat[0] = quat.x;
					temp_link.quat[1] = quat.y;
					temp_link.quat[2] = quat.z;
					temp_link.quat[3] = quat.w;
				}
				update_robot(temp_robot);

				renderer.render(scene, camera);

				i++;
			}, 10);
		}

	};

	// Copy values to clipboard
	$(document).on("click", "input.copy", function(e) {
		e.preventDefault();

		// Get val
		var $form = $(this).closest("form");
		var val = matrixToString(getMatrix($form));

		// Create temporary input to copy to clipboard
		var $temp = $("<input>");
		$("body").append($temp);
		$temp.val(val).select();
		document.execCommand("copy");
		$temp.remove();
	});

	// Send redis values on form submit
	$(document).on("submit", "form", function(e) {
		e.preventDefault();

		// Get keyval
		var $form = $(this);
		var key = KEY_PREFIX + $form.attr("data-key");
		var val;
		if ($form.attr("data-key") == "dh_table") {
			val = $form.find("div.val-row").map(function() {
				return $(this).find("input.val").map(function() {
					return $(this).val();
				}).get().join(",");
			}).get().join(";");
		} else {
			val = getMatrix($form);
		}

		key_vals = {};
		key_vals[key] = val;
		key_vals[KEY_PREFIX + "set"] =  key;
		ajaxSendRedis(key_vals);
	});

	function init_graphics() {

		var width = window.innerWidth - $("#sidebar").width();
		var height = window.innerHeight;

		camera = new THREE.PerspectiveCamera(50, width / height, 0.01, 100);
		camera.position.copy(new THREE.Vector3(0.75, -0.75, 0.5));
		camera.up.copy(new THREE.Vector3(0, 0, 1));

		scene = new THREE.Scene();

		renderer = new THREE.WebGLRenderer({ antialias: true });
		renderer.setSize(width, height);
		$("#content").html(renderer.domElement);

		var controls = new THREE.OrbitControls(camera, renderer.domElement);
		controls.addEventListener("change", function() {
			renderer.render(scene, camera);
		});

		var grid = new THREE.GridHelper(1, 10);
		grid.rotation.x = Math.PI / 2;
		scene.add(grid);
		scene.add(axes(AXIS_SIZE, AXIS_WIDTH));

		renderer.render(scene, camera);
	}

	function revolute_mesh() {
		var group = new THREE.Object3D();

		var mesh = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.05), new THREE.MeshNormalMaterial());
		group.add(mesh);

		mesh = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.004, 16), new THREE.MeshNormalMaterial());
		mesh.position.y += 0.025;
		group.add(mesh);

		mesh = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.004, 16), new THREE.MeshNormalMaterial());
		mesh.position.y -= 0.025;
		group.add(mesh);

		group.rotation.x += Math.PI / 2;

		return group;
	}

	function prismatic_mesh() {
		var group = new THREE.Object3D();

		var shape = new THREE.Shape();
		shape.moveTo(0, 0);
		shape.lineTo(0.03, 0.03);
		shape.lineTo(0.03, -0.03);
		shape.lineTo(0, 0);
		var mesh = new THREE.Mesh(new THREE.ExtrudeGeometry(shape, {amount: 0.01, bevelEnabled: false}), new THREE.MeshNormalMaterial());
		group.add(mesh);

		shape = new THREE.Shape();
		shape.moveTo(0, 0);
		shape.lineTo(-0.03, 0.03);
		shape.lineTo(-0.03, -0.03);
		shape.lineTo(0, 0);
		mesh = new THREE.Mesh(new THREE.ExtrudeGeometry(shape, {amount: 0.01, bevelEnabled: false}), new THREE.MeshNormalMaterial());
		group.add(mesh);

		group.rotation.x += Math.PI / 2;

		return group;
	}

	function create_robot(robot) {

		var prev_frame = scene;
		robot.links.forEach(function(link) {
			var body = new THREE.Object3D();

			var mesh = link.type == 0 ? revolute_mesh() : prismatic_mesh();
			body.add(mesh);

			var quat = new THREE.Quaternion(link.quat[0], link.quat[1], link.quat[2], link.quat[3]);
			var pos  = new THREE.Quaternion(link.pos[0], link.pos[1], link.pos[2], 0);
			body.quaternion.copy(quat);
			body.position.copy(pos);

			body.add(axes(AXIS_SIZE, AXIS_WIDTH));

			bodies.push(body);
			prev_frame.add(body);
			prev_frame = body;
		});

		renderer.render(scene, camera);

	}

	function update_robot(robot) {

		for (var i = 0; i < robot.links.length; i++) {
			var link = robot.links[i];
			bodies[i].quaternion.set(link.quat[0], link.quat[1], link.quat[2], link.quat[3]);
			bodies[i].position.set(link.pos[0], link.pos[1], link.pos[2]);
		}

		renderer.render(scene, camera);

	}

	function delete_robot() {

		scene.remove(bodies[0]);
		bodies = [];

	}

	init_graphics();

	$(window).resize(function() {
		var width = window.innerWidth - $("#sidebar").width();
		var height = window.innerHeight;
		camera.aspect = width / height;
		camera.updateProjectionMatrix();
		renderer.setSize(width, height);
		renderer.render(scene, camera);
	})

});
