/**
 * redis-web-gui.js
 *
 * Author: Toki Migimatsu
 * Created: December 2017
 */

var KEY_PREFIX = "cs223a::gimbal_lock::";
var KEY_SET    = "set";
var KEY_EULER  = "zyx_euler_angles";
var KEY_FIXED  = "xyz_fixed_angles";
var KEY_MATRIX = "matrix";
var KEY_AXIS   = "rotation_axis";
var KEY_ANGLE  = "rotation_angle";
var KEY_EULER_DES = "rotation_euler";
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
	var euler_angles, fixed_angles;
	var new_euler_angles, new_fixed_angles, des_euler_angles;
	var animation_timer;

	ws.onmessage = function(e) {
		var msg = JSON.parse(e.data);

		// Look for set key
		// for (var i = 0; i < msg.length; i++) {
		//     var m = msg[i];
		//     if (m[0] != "cs223a::rotations::set") continue;
		//     if (m[1] != "update") return;
		// }

		// Update requested
		msg.forEach(function(m) {
			var matches = m[0].match("^" + KEY_PREFIX + "(.*)$");
			if (matches === null) return;
			var key = matches[1];
			var val = m[1];

			if (key == KEY_SET) {
				// Flag when all keys have been updated
				updated = (val == "");
				return;
			} else if (key == KEY_EULER_DES) {
				des_euler_angles = val[0];
				return;
			}

			var $form = $("form[data-key='" + key + "']");

			// Create new redis key-value form
			if ($form.length == 0) {
				var form = htmlForm(key, val);
				var $form = $(form).hide();
				var li = "<a href='#" + key + "' title='" + key + "'><li>" + key + "</li></a>";
				var $li = $(li).hide();

				// Find alphabetical ordering
				var keys = $("form").map(function() {
					return $(this).attr("data-key");
				}).get();
				var idx_key;
				for (idx_key = 0; idx_key < keys.length; idx_key++) {
					if (key == KEY_AXIS || key == KEY_ANGLE) {
						if (keys[idx_key] != KEY_AXIS && keys[idx_key] != KEY_ANGLE) continue;
					} else {
						if (keys[idx_key] == KEY_AXIS || keys[idx_key] == KEY_ANGLE) break;
					}
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

				switch(key) {
					case KEY_EULER:
						euler_angles = val[0];
						new_euler_angles = val[0];
						create_euler(val[0][0], val[0][1], val[0][2]);
						break;
					case KEY_FIXED:
						fixed_angles = val[0];
						new_fixed_angles = val[0];
						break;
					case KEY_MATRIX:
						var $inputs = $form.find("input.val");
						$inputs.eq(0).val(0);
						$inputs.eq(1).val(0);
						$inputs.eq(2).val(1);
						$inputs.eq(3).val(0);
						$inputs.eq(4).val(1);
						$inputs.eq(5).val(0);
						$inputs.eq(6).val(-1);
						$inputs.eq(7).val(0);
						$inputs.eq(8).val(0);
						$form.submit();
						break;
					case KEY_AXIS:
						create_rotation_axis(val[0]);
						break;
				}
				return;
			}

			// Update html
			updateHtmlValues($form, val);

			switch(key) {
				case KEY_EULER:
					new_euler_angles = val[0];
					break;
				case KEY_FIXED:
					new_fixed_angles = val[0];
					break;
				case KEY_AXIS:
					update_rotation_axis(val[0]);
					break;
				case KEY_EULER_DES:
					des_euler_angles = val[0];
					break;
			}

		});

		if (updated) {
			// Animate rotation
			var i = 1;
			if (animation_timer) {
				clearInterval(animation_timer);
			}
			animation_timer = setInterval(function() {
				if (i > 100) {
					clearInterval(animation_timer);
					euler_angles = new_euler_angles;
					fixed_angles = new_fixed_angles;
					return;
				}
				var j = i / 100;

				var alpha = (1 - j) * euler_angles[0] + j * new_euler_angles[0];
				var beta  = (1 - j) * euler_angles[1] + j * new_euler_angles[1];
				var gamma = (1 - j) * euler_angles[2] + j * new_euler_angles[2];
				update_euler(alpha, beta, gamma);

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
		var val = getMatrix($form);

		key_vals = {};
		key_vals[key] = val;
		key_vals[KEY_PREFIX + KEY_SET] =  key;
		ajaxSendRedis(key_vals);
	});

	function init_graphics() {

		var width = window.innerWidth - $("#sidebar").width();
		var height = window.innerHeight;

		camera = new THREE.PerspectiveCamera(50, width / height, 0.01, 100);
		camera.position.copy(new THREE.Vector3(0.75, 0.75, 0.5));
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

		world_axes = axes(AXIS_SIZE, AXIS_WIDTH);
		world_axes.position.copy(new THREE.Vector3(0.3, -0.4, 0));
		scene.add(world_axes);

		renderer.render(scene, camera);
	}

	function gimbal(radius, thickness, spacing, hex_color) {

		var gimbal = new THREE.Object3D();

		var ring_geometry = new THREE.TorusGeometry(radius, thickness, 8, 64);
		var ring_material = new THREE.MeshBasicMaterial({ color: new THREE.Color(hex_color) });
		var ring = new THREE.Mesh(ring_geometry, ring_material);
		gimbal.add(ring);

		var axis_geometry_a = new THREE.CylinderGeometry(thickness, thickness, spacing, 8);
		var axis_material_a = new THREE.MeshBasicMaterial({ color: new THREE.Color(hex_color) });
		var axis_a = new THREE.Mesh(axis_geometry_a, axis_material_a);
		axis_a.position.y = radius + spacing / 2;
		gimbal.add(axis_a);

		var axis_geometry_b = new THREE.CylinderGeometry(thickness, thickness, spacing, 8);
		var axis_material_b = new THREE.MeshBasicMaterial({ color: new THREE.Color(hex_color) });
		var axis_b = new THREE.Mesh(axis_geometry_b, axis_material_b);
		axis_b.position.y = -radius - spacing / 2;
		gimbal.add(axis_b);

		return gimbal;

	}

	function rotation_arrow(hex_color, line_width, radius) {
		var arrow = new THREE.Object3D();

		var ring_material = new MeshLineMaterial({
			color: new THREE.Color(hex_color),
			lineWidth: line_width
		});
		var ring_geometry = new THREE.Geometry();
		for (var i = 0; i < 3/2 * Math.PI; i += 2 * Math.PI / 32) {
			ring_geometry.vertices.push(new THREE.Vector3(radius * Math.cos(i), radius * Math.sin(i), 0));
		}
		var ring_line = new MeshLine();
		ring_line.setGeometry(ring_geometry);
		var ring = new THREE.Mesh(ring_line.geometry, ring_material);

		var cone_material = new THREE.MeshBasicMaterial({ color: new THREE.Color(hex_color) });
		var cone_geometry = new THREE.ConeGeometry(0.01, 0.03);
		var cone = new THREE.Mesh(cone_geometry, cone_material);
		cone.position.y = -radius;
		cone.rotation.z = -Math.PI / 2 - 0.3;

		var axis_material = new MeshLineMaterial({
			color: new THREE.Color(hex_color),
			lineWidth: line_width
		});
		var axis_geometry = new THREE.Geometry();
		var axis_length = 3/4 * (FRAME_OFFSET - AXIS_SIZE);
		axis_geometry.vertices.push(new THREE.Vector3(0, 0, -axis_length / 2));
		axis_geometry.vertices.push(new THREE.Vector3(0, 0, axis_length / 2));
		var axis_line = new MeshLine();
		axis_line.setGeometry(axis_geometry);
		var axis = new THREE.Mesh(axis_line.geometry, axis_material);

		arrow.add(ring);
		arrow.add(cone);
		arrow.add(axis);
		return arrow;
	}

	var euler_frames = new THREE.Object3D();

	function create_euler(alpha, beta, gamma) {

		var frame_0 = new THREE.Object3D();
		var ring_geometry = new THREE.TorusGeometry(0.3, 0.002, 8, 64);
		var ring_material = new THREE.MeshNormalMaterial();
		var ring = new THREE.Mesh(ring_geometry, ring_material);
		ring.rotation.x = Math.PI / 2;
		frame_0.add(ring);

		var frame_a = new THREE.Object3D();
		var rot_a = new THREE.Euler(0, 0, alpha, "ZYX");
		frame_a.rotation.copy(rot_a);
		var gimbal_x = gimbal(0.25, 0.002, 0.05, 0x0000ff);
		gimbal_x.rotation.x = Math.PI / 2;
		gimbal_x.rotation.y = Math.PI / 2;
		frame_a.add(gimbal_x);

		var frame_b = new THREE.Object3D();
		var rot_b = new THREE.Euler(0, beta, alpha, "ZYX");
		frame_b.rotation.copy(rot_b);
		var gimbal_y = gimbal(0.2, 0.002, 0.05, 0x00ff00);
		frame_b.add(gimbal_y);

		var frame_c = new THREE.Object3D();
		var rot_c = new THREE.Euler(gamma, beta, alpha, "ZYX");
		frame_c.rotation.copy(rot_c);
		var gimbal_z = gimbal(0.15, 0.002, 0.05, 0xff0000);
		gimbal_z.rotation.x = Math.PI / 2;
		gimbal_z.rotation.z = Math.PI / 2;
		frame_c.add(gimbal_z);

		var axes_c = axes(AXIS_SIZE, AXIS_WIDTH);
		axes_c.rotation.copy(rot_c);

		var sphere_geometry = new THREE.SphereGeometry(AXIS_WIDTH, 32, 32);
		var sphere_material = new THREE.MeshBasicMaterial({ color: 0xffffff });
		var sphere_x = new THREE.Mesh(sphere_geometry, sphere_material);
		sphere_x.position.x = AXIS_SIZE;
		var sphere_y = new THREE.Mesh(sphere_geometry, sphere_material);
		sphere_y.position.y = AXIS_SIZE;
		var sphere_z = new THREE.Mesh(sphere_geometry, sphere_material);
		sphere_z.position.z = AXIS_SIZE;
		axes_c.add(sphere_x);
		axes_c.add(sphere_y);
		axes_c.add(sphere_z);
		frame_c.add(axes_c);

		euler_frames.add(frame_0);
		euler_frames.add(frame_a);
		euler_frames.add(frame_b);
		euler_frames.add(frame_c);
		scene.add(euler_frames);

	}

	var rotation_axis = new THREE.Object3D();

	function create_rotation_axis(direction) {

		var axis_material = new MeshLineMaterial({
			color: new THREE.Color(0xffff00),
			lineWidth: AXIS_WIDTH
		});
		var axis_geometry = new THREE.Geometry();
		axis_geometry.vertices.push(new THREE.Vector3(0, 0, 0));
		axis_geometry.vertices.push(new THREE.Vector3(0, 0, -0.1));
		var axis_line = new MeshLine();
		axis_line.setGeometry(axis_geometry);
		var axis = new THREE.Mesh(axis_line.geometry, axis_material)

		var arrow = rotation_arrow(0xffff00, AXIS_WIDTH, ROTATION_RADIUS);
		arrow.position.z = -0.1;
		arrow.rotation.x = Math.PI;
		
		rotation_axis.add(axis);
		rotation_axis.add(arrow);

		var R = new THREE.Matrix4();
		var to = new THREE.Vector3(direction[0], direction[1], direction[2]);
		R.lookAt(new THREE.Vector3(0, 0, 0), to, new THREE.Vector3(0, 0, 1));
		rotation_axis.setRotationFromMatrix(R);

		scene.add(rotation_axis);

	}

	function update_euler(alpha, beta, gamma) {

		var frame_a = euler_frames.children[1];
		frame_a.rotation.z = alpha;

		var frame_b = euler_frames.children[2];
		frame_b.rotation.y = beta;
		frame_b.rotation.z = alpha;

		var frame_c = euler_frames.children[3];
		frame_c.rotation.x = gamma;
		frame_c.rotation.y = beta;
		frame_c.rotation.z = alpha;

	}

	function update_rotation_axis(direction) {

		var R = new THREE.Matrix4();
		var to = new THREE.Vector3(direction[0], direction[1], direction[2]);
		R.lookAt(new THREE.Vector3(0, 0, 0), to, new THREE.Vector3(0, 0, 1));
		rotation_axis.setRotationFromMatrix(R);

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
