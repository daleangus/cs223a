/**
 * redis-web-gui.js
 *
 * Author: Toki Migimatsu
 * Created: December 2017
 */

var KEY_PREFIX   = "cs223a::jacobian::";
var KEY_Q        = "q";
var KEY_T_EE     = "T_ee_to_0";
var KEY_DH_TABLE = "dh_table";
var KEY_JV       = "J_v";
var KEY_JW       = "J_w";
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

	var camera, scene, renderer, raycaster;

	var updated = false;
	var animation_timer;
	var bodies = [];
	var idx_selected = -1;
	var F_selected = -1;
	var M_selected = -1;
	var ee_velocities;
	var collision_meshes = [];
	var selected_meshes = [];
	var robot = null, new_robot = null;
	var J_v = null, J_w = null;
	var new_J_v = null, new_J_w = null;

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
			} else if (key == KEY_JV) {
				if (J_v === null) {
					J_v = val;
				} else {
					new_J_v = val;
				}
			} else if (key == KEY_JW) {
				if (J_w === null) {
					J_w = val;
				} else {
					new_J_w = val;
				}
			}

			var $form = $("form[data-key='" + key + "']");

			// Create new redis key-value form
			if ($form.length == 0) {
				var form;
				if (key == KEY_DH_TABLE) {
					val = val.split(";").map(el => el.split(","));
					form = htmlForm(key, val);
					form = form.replace(/NaN/g,"");
				} else if (key == KEY_JV || key == KEY_JW) {
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

		if (updated && new_robot != null) {
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

			if (idx_selected >= 0) {
				// Find current velocity in selected frame
				var q_0_to_i = bodies[idx_selected].getWorldQuaternion().inverse();
				var v = new THREE.Vector3(J_v[0][idx_selected], J_v[1][idx_selected], J_v[2][idx_selected]);
				v.applyQuaternion(q_0_to_i);
				var w = new THREE.Vector3(J_w[0][idx_selected], J_w[1][idx_selected], J_w[2][idx_selected]);
				w.applyQuaternion(q_0_to_i);

				// Find new velocity in updated selected frame
				update_robot(new_robot);
				scene.updateMatrixWorld();

				q_0_to_i = bodies[idx_selected].getWorldQuaternion().inverse();
				if (new_J_v !== null) {
					var new_v = new THREE.Vector3(new_J_v[0][idx_selected], new_J_v[1][idx_selected], new_J_v[2][idx_selected]);
					new_v.applyQuaternion(q_0_to_i);
				}
				if (new_J_w !== null) {
					var new_w = new THREE.Vector3(new_J_w[0][idx_selected], new_J_w[1][idx_selected], new_J_w[2][idx_selected]);
					new_w.applyQuaternion(q_0_to_i);
				}

				// Revert selected frame
				update_robot(robot);
				scene.updateMatrixWorld();
			} else if (F_selected >= 0) {
				var tau_F = J_v[F_selected];
				if (new_J_v !== null) {
					var new_tau_F = new_J_v[F_selected];
				}
			} else if (M_selected >= 0) {
				var tau_M = J_w[M_selected];
				if (new_J_w !== null) {
					var new_tau_M = new_J_w[M_selected];
				}
			}

			animation_timer = setInterval(function() {
				if (i > 100) {
					clearInterval(animation_timer);
					robot = new_robot;
					new_robot = null;
					if (new_J_v !== null) {
						J_v = new_J_v;
					}
					if (new_J_w !== null) {
						J_w = new_J_w;
					}
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
				for (var k = 0; k < 3; k++) {
					temp_robot.ee.pos[k] = (1 - j) * robot.ee.pos[k] + j * new_robot.ee.pos[k];
				}
				update_robot(temp_robot);

				if (idx_selected >= 0) {
					var v_i = new THREE.Vector3(v.x, v.y, v.z);
					if (new_J_v !== null) {
						v_i.lerp(new_v, j);
					}

					var w_i = new THREE.Vector3(w.x, w.y, w.z);
					if (new_J_w !== null) {
						w_i.lerp(new_w, j);
					}

					var ee_pos = new THREE.Vector3(temp_robot.ee.pos[0], temp_robot.ee.pos[1], temp_robot.ee.pos[2]);
					update_velocities(v_i, w_i, ee_pos, idx_selected);
				} else if (F_selected >= 0) {
					var tau_F_i = tau_F.slice();
					if (new_J_v !== null) {
						for (var k = 0; k < tau_F.length; k++) {
							tau_F_i[k] = (1 - j) * tau_F[k] + j * new_tau_F[k];
						}
					}
					update_torques(tau_F_i, true);
				} else if (M_selected >= 0) {
					var tau_M_i = tau_M.slice();
					if (new_J_w !== null) {
						for (var k = 0; k < tau_M.length; k++) {
							tau_M_i[k] = (1 - j) * tau_M[k] + j * new_tau_M[k];
						}
					}
					update_torques(tau_M_i, false);
				}

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

	$(document).on("click", "#content", function(e) {
		var size = renderer.getSize();
		var mouse = new THREE.Vector2();
		mouse.x = e.clientX / size.width * 2 - 1;
		mouse.y = -e.clientY / size.height * 2 + 1;

		raycaster.setFromCamera(mouse, camera);
		var intersects = raycaster.intersectObjects(collision_meshes);
		if (intersects.length == 0) return;

		var intersected = intersects[0].object;
		var body = intersected.parent.parent;

		for (var i = 0; i < bodies.length; i++) {
			if (body != bodies[i]) continue;

			if (i == bodies.length - 1) {
				select_force(intersected.position);
			} else {
				select_joint(i);
			}
			return;
		}

		select_moment(intersected.parent.position);
	});

	function point_light(x, y, z) {
		var light = new THREE.PointLight(0xffffff, 2, 3, 1);
		light.position.set(x, y, z);
		return light;
	}

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

		raycaster = new THREE.Raycaster();

		var controls = new THREE.OrbitControls(camera, renderer.domElement);
		controls.addEventListener("change", function() {
			renderer.render(scene, camera);
		});

		var grid = new THREE.GridHelper(1, 10);
		grid.rotation.x = Math.PI / 2;
		scene.add(grid);
		scene.add(axes(AXIS_SIZE, AXIS_WIDTH));

		scene.add(point_light(0, 1, 2));
		scene.add(point_light(1, 1, 2));
		scene.add(point_light(1, 0, 2));
		scene.add(point_light(1, -1, 2));
		scene.add(point_light(0, -1, 2));
		scene.add(point_light(-1, -1, 2));
		scene.add(point_light(-1, 0, 2));
		scene.add(point_light(-1, 1, 2));

		renderer.render(scene, camera);
	}

	function revolute_mesh() {
		var group = new THREE.Object3D();

		var mesh = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.05), new THREE.MeshNormalMaterial());
		group.add(mesh);
		collision_meshes.push(mesh);

		mesh = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.004, 16), new THREE.MeshNormalMaterial());
		mesh.position.y += 0.025;
		group.add(mesh);
		collision_meshes.push(mesh);

		mesh = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.004, 16), new THREE.MeshNormalMaterial());
		mesh.position.y -= 0.025;
		group.add(mesh);
		collision_meshes.push(mesh);

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
		collision_meshes.push(mesh);

		shape = new THREE.Shape();
		shape.moveTo(0, 0);
		shape.lineTo(-0.03, 0.03);
		shape.lineTo(-0.03, -0.03);
		shape.lineTo(0, 0);
		mesh = new THREE.Mesh(new THREE.ExtrudeGeometry(shape, {amount: 0.01, bevelEnabled: false}), new THREE.MeshNormalMaterial());
		group.add(mesh);
		collision_meshes.push(mesh);

		group.rotation.x += Math.PI / 2;

		return group;
	}

	function axis_cones(radius, hex_color) {
		var cones = new THREE.Object3D();
		var cone_geometry = new THREE.ConeGeometry(0.01, 0.03);
		var cone_material;
		if (hex_color == null) {
			cone_material = new THREE.MeshNormalMaterial();
		} else {
			cone_material = new THREE.MeshBasicMaterial({ color: new THREE.Color(hex_color) });
		}
		var cone_x = new THREE.Mesh(cone_geometry, cone_material);
		cone_x.position.x = AXIS_SIZE;
		cone_x.rotation.z = -Math.PI / 2;
		var cone_y = new THREE.Mesh(cone_geometry, cone_material);
		cone_y.position.y = AXIS_SIZE;
		var cone_z = new THREE.Mesh(cone_geometry, cone_material);
		cone_z.position.z = AXIS_SIZE;
		cone_z.rotation.x = Math.PI / 2;
		cones.add(cone_x);
		cones.add(cone_y);
		cones.add(cone_z);
		return cones;
	}

	function axis_rings(radius) {
		var rings = new THREE.Object3D();

		var ring_x = rotation_ring(0xff0000, AXIS_WIDTH, 0.03, 0.01, 0.03);
		ring_x.position.x = AXIS_SIZE;
		ring_x.rotation.y = Math.PI / 2;
		ring_x.children[1].material = new THREE.MeshNormalMaterial();
		var ring_y = rotation_ring(0x00ff00, AXIS_WIDTH, 0.03, 0.01, 0.03);
		ring_y.position.y = AXIS_SIZE;
		ring_y.rotation.x = -Math.PI / 2;
		ring_y.children[1].material = new THREE.MeshNormalMaterial();
		var ring_z = rotation_ring(0x0000ff, AXIS_WIDTH, 0.03, 0.01, 0.03);
		ring_z.position.z = AXIS_SIZE;
		ring_z.children[1].material = new THREE.MeshNormalMaterial();

		rings.add(ring_x);
		rings.add(ring_y);
		rings.add(ring_z);
		return rings;
	}

	function create_robot(robot) {

		var prev_frame = scene;
		robot.links.forEach(function(link) {
			var body = new THREE.Object3D();

			var mesh = link.type == 0 ? revolute_mesh() : prismatic_mesh();

			var quat = new THREE.Quaternion(link.quat[0], link.quat[1], link.quat[2], link.quat[3]);
			var pos  = new THREE.Quaternion(link.pos[0], link.pos[1], link.pos[2], 0);
			body.quaternion.copy(quat);
			body.position.copy(pos);

			var force_arrow = link.type == 0 ? rotation_arrow(0xff00ff, AXIS_WIDTH, 1, 0.03) : arrow(0xff00ff, AXIS_WIDTH, 1);
			force_arrow.visible = false;

			var moment_arrow = link.type == 0 ? rotation_arrow(0x00ffff, AXIS_WIDTH, 1, 0.03) : arrow(0x00ffff, AXIS_WIDTH, 1);
			moment_arrow.visible = false;

			body.add(mesh);
			body.add(axes(AXIS_SIZE, AXIS_WIDTH));
			body.add(force_arrow);
			body.add(moment_arrow);

			bodies.push(body);
			prev_frame.add(body);
			prev_frame = body;
		});

		scene.updateMatrixWorld();

		// Create end-effector
		var ee_frame = new THREE.Object3D();
		var ee_geometry = new THREE.SphereGeometry(0.01, 32, 32);
		var ee_material = new THREE.MeshNormalMaterial();
		var ee_mesh = new THREE.Mesh(ee_geometry, ee_material);
		ee_frame.position.set(robot.ee.pos[0], robot.ee.pos[1], robot.ee.pos[2]);
		var q_0_to_n = prev_frame.getWorldQuaternion().inverse();
		ee_frame.quaternion.copy(q_0_to_n);

		var ee_axes = axes(AXIS_SIZE, AXIS_WIDTH);
		var ee_cones = axis_cones(0.01);
		var ee_rings = axis_rings(0.01);
		for (var i = 0; i < 3; i++) {
			collision_meshes.push(ee_cones.children[i]);
			collision_meshes.push(ee_rings.children[i].children[1]);
		}

		ee_frame.add(ee_cones);
		ee_frame.add(ee_rings);
		ee_frame.add(ee_mesh);
		ee_frame.add(ee_axes);
		bodies.push(ee_frame);
		prev_frame.add(ee_frame);

		// Add end-effector frame
		ee_velocities = new THREE.Object3D();
		ee_velocities.add(arrow(0xff00ff, AXIS_WIDTH, 1));
		ee_velocities.add(rotation_arrow(0x00ffff, AXIS_WIDTH, 0.075, 0.03));

		renderer.render(scene, camera);

	}

	function update_robot(robot) {

		for (var i = 0; i < robot.links.length; i++) {
			var link = robot.links[i];
			bodies[i].quaternion.set(link.quat[0], link.quat[1], link.quat[2], link.quat[3]);
			bodies[i].position.set(link.pos[0], link.pos[1], link.pos[2]);
		}

		var ee_frame = bodies[bodies.length - 1];
		ee_frame.position.set(robot.ee.pos[0], robot.ee.pos[1], robot.ee.pos[2]);
		var q_0_to_n = bodies[bodies.length - 2].getWorldQuaternion().inverse();
		ee_frame.quaternion.copy(q_0_to_n);

	}

	function delete_robot() {

		scene.remove(bodies[0]);
		bodies = [];

	}

	function arrow(hex_color, line_width, line_length) {
		var arrow = new THREE.Object3D();

		var axis_material = new MeshLineMaterial({
			color: new THREE.Color(hex_color),
			lineWidth: line_width
		});
		var axis_geometry = new THREE.Geometry();
		axis_geometry.vertices.push(new THREE.Vector3(0, 0, 0));
		axis_geometry.vertices.push(new THREE.Vector3(0, 0, line_length));
		var axis_line = new MeshLine();
		axis_line.setGeometry(axis_geometry);
		var axis = new THREE.Mesh(axis_line.geometry, axis_material);

		var cone_material = new THREE.MeshBasicMaterial({ color: new THREE.Color(hex_color) });
		var cone_geometry = new THREE.ConeGeometry(0.0075, 0.02);
		var cone = new THREE.Mesh(cone_geometry, cone_material);
		cone.position.z = line_length;
		cone.rotation.x = Math.PI / 2;

		arrow.add(axis);
		arrow.add(cone);
		return arrow;
	}

	function rotation_ring(hex_color, line_width, ring_radius, cone_radius, cone_length) {
		var ring_arrow = new THREE.Object3D();

		var ring_material = new MeshLineMaterial({
			color: new THREE.Color(hex_color),
			lineWidth: line_width
		});
		var ring_geometry = new THREE.Geometry();
		for (var i = 0; i < 3/2 * Math.PI; i += 2 * Math.PI / 32) {
			ring_geometry.vertices.push(new THREE.Vector3(ring_radius * Math.cos(i), ring_radius * Math.sin(i), 0));
		}
		var ring_line = new MeshLine();
		ring_line.setGeometry(ring_geometry);
		var ring = new THREE.Mesh(ring_line.geometry, ring_material);
		ring.position.z = -0.01;

		var cone_material = new THREE.MeshBasicMaterial({ color: new THREE.Color(hex_color) });
		var cone_geometry = new THREE.ConeGeometry(0.0075, 0.03);
		var cone = new THREE.Mesh(cone_geometry, cone_material);
		cone.position.set(0, -ring_radius, -0.01);
		cone.rotation.z = -Math.PI / 2 - 0.3;

		ring_arrow.add(ring);
		ring_arrow.add(cone);

		return ring_arrow;
	}

	function rotation_arrow(hex_color, line_width, line_length, radius) {
		var rotation_arrow = new THREE.Object3D();

		var arrow = rotation_ring(hex_color, line_width, radius, 0.0075, 0.03);
		arrow.position.z = line_length;

		var axis_material = new MeshLineMaterial({
			color: new THREE.Color(hex_color),
			lineWidth: line_width
		});
		var axis_geometry = new THREE.Geometry();
		axis_geometry.vertices.push(new THREE.Vector3(0, 0, 0));
		axis_geometry.vertices.push(new THREE.Vector3(0, 0, line_length));
		var axis_line = new MeshLine();
		axis_line.setGeometry(axis_geometry);
		var axis = new THREE.Mesh(axis_line.geometry, axis_material);

		rotation_arrow.add(axis);
		rotation_arrow.add(arrow);
		return rotation_arrow;
	}

	function update_torques(tau, forces) {

		idx_arrow = forces ? 2 : 3;

		// Update force arrows
		for (var i = 0; i < bodies.length - 1; i++) {
			var tau_i = tau[i];
			var force_arrow = bodies[i].children[idx_arrow];
			var axis = force_arrow.children[0];
			var arrow = force_arrow.children[1];

			force_arrow.rotation.x = 0;
			force_arrow.visible = true;

			if (tau_i < -0.01) {
				force_arrow.rotation.x = Math.PI;
				tau_i = -tau_i;
			} else if (tau_i < 0.01) {
				force_arrow.visible = false;
				continue;
			}

			axis.scale.z = tau_i;
			arrow.position.z = tau_i;

		}

	}

	function update_velocities(v_i, w_i, ee_pos, idx_body) {

		scene.updateMatrixWorld();

		// Attach end-effector frame to selected frame
		bodies[bodies.length - 2].localToWorld(ee_pos);
		bodies[idx_body].worldToLocal(ee_pos);
		ee_velocities.position.copy(ee_pos);
		bodies[idx_body].add(ee_velocities);

		// Set linear velocity
		var v_axis = ee_velocities.children[0];
		if (v_i.length() < 0.01) {
			v_axis.visible = false;
		} else {
			v_axis.visible = true;
			v_axis.lookAt(v_i);

			var axis = v_axis.children[0];
			var cone = v_axis.children[1];
			axis.scale.z = v_i.length();
			cone.position.z = v_i.length();
		}

		// Set angular velocity
		var w_axis = ee_velocities.children[1];
		if (w_i.length() < 0.1) {
		  	w_axis.visible = false;
		} else {
			w_axis.visible = true;
			w_axis.lookAt(w_i);
		}
	}

	function select_force(dir) {

		if (dir.x > 0) {
			dir = 0;
		} else if (dir.y > 0) {
			dir = 1;
		} else {
			dir = 2;
		}

		// Highlight direction sphere
		for (var j = 0; j < selected_meshes.length; j++) {
			selected_meshes[j].material = new THREE.MeshNormalMaterial();
		}
		selected_meshes = [bodies[bodies.length-1].children[0].children[dir]];
		selected_meshes[0].material = new THREE.MeshLambertMaterial({ emissive: 0x333333 });

		if (idx_selected >= 0) {
			bodies[idx_selected].remove(ee_velocities);
			idx_selected = -1;
		}
		if (M_selected >= 0) deselect_torque(M_selected, false);
		M_selected = -1;
		update_torques(J_v[dir], true);

		F_selected = dir;
		renderer.render(scene, camera);

		// Highlight Jacobian row
		var $J_v = $("form[data-key='J_v'] input.val");
		var $J_w = $("form[data-key='J_w'] input.val");
		$J_v.css("background-color", "white");
		$J_w.css("background-color", "white");
		var n = J_v[0].length;
		for (var j = 0; j < n; j++) {
			$J_v.eq(n*dir+j).css("background-color", "#fbf");
		}

	}

	function select_moment(dir) {

		if (dir.x > 0) {
			dir = 0;
		} else if (dir.y > 0) {
			dir = 1;
		} else {
			dir = 2;
		}

		// Highlight direction sphere
		for (var j = 0; j < selected_meshes.length; j++) {
			selected_meshes[j].material = new THREE.MeshNormalMaterial();
		}
		selected_meshes = [bodies[bodies.length-1].children[1].children[dir].children[1]];
		selected_meshes[0].material = new THREE.MeshLambertMaterial({ emissive: 0x333333 });

		if (idx_selected >= 0) {
			bodies[idx_selected].remove(ee_velocities);
			idx_selected = -1;
		}
		if (F_selected >= 0) deselect_torque(F_selected, true);
		F_selected = -1;
		update_torques(J_w[dir], false);

		M_selected = dir;
		renderer.render(scene, camera);

		// Highlight Jacobian row
		var $J_v = $("form[data-key='J_v'] input.val");
		var $J_w = $("form[data-key='J_w'] input.val");
		$J_v.css("background-color", "white");
		$J_w.css("background-color", "white");
		var n = J_w[0].length;
		for (var j = 0; j < n; j++) {
			$J_w.eq(n*dir+j).css("background-color", "#bff");
		}

	}

	function deselect_torque(i, forces) {

		idx_arrow = forces ? 2 : 3;

		if (i >= 0) {
			for (var j = 0; j < bodies.length - 1; j++) {
				bodies[j].children[idx_arrow].visible = false;
			}
			// var force_sphere = bodies[bodies.length - 1].children[0].children[i];
			// force_sphere.material = new THREE.MeshNormalMaterial();
			i = -1;
		}

		if (forces) {
			var $J_v = $("form[data-key='J_v'] input.val");
			$J_v.css("background-color", "white");
		} else {
			var $J_w = $("form[data-key='J_w'] input.val");
			$J_w.css("background-color", "white");
		}

	}

	function select_joint(i) {

		// Highlight joint
		for (var j = 0; j < selected_meshes.length; j++) {
			selected_meshes[j].material = new THREE.MeshNormalMaterial();
		}
		selected_meshes = bodies[i].children[0].children;
		for (var j = 0; j < selected_meshes.length; j++) {
			selected_meshes[j].material = new THREE.MeshLambertMaterial({ emissive: 0x333333 });
		}
		if (F_selected >= 0) deselect_torque(F_selected, true);
		if (M_selected >= 0) deselect_torque(M_selected, false);

		var q_0_to_i = bodies[i].getWorldQuaternion().inverse();
		var v = new THREE.Vector3(J_v[0][i], J_v[1][i], J_v[2][i]);
		v.applyQuaternion(q_0_to_i);
		var w = new THREE.Vector3(J_w[0][i], J_w[1][i], J_w[2][i]);
		w.applyQuaternion(q_0_to_i);
		var ee_pos = new THREE.Vector3(robot.ee.pos[0], robot.ee.pos[1], robot.ee.pos[2]);
		update_velocities(v, w, ee_pos, i);

		idx_selected = i;
		renderer.render(scene, camera);

		// Highlight Jacobian column
		var $J_v = $("form[data-key='J_v'] input.val");
		var $J_w = $("form[data-key='J_w'] input.val");
		$J_v.css("background-color", "white");
		$J_w.css("background-color", "white");
		var n = J_v[0].length;
		for (var j = 0; j < 3; j++) {
			$J_v.eq(n*j+i).css("background-color", "#fbf");
			$J_w.eq(n*j+i).css("background-color", "#bff");
		}

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
