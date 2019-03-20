/**
 * redis-web-gui.js
 *
 * Author: Toki Migimatsu
 * Created: December 2017
 */

var KEY_PREFIX = "cs223a::simulator::";
var KEY_ROBOT = "robot";
var AXIS_WIDTH = 0.005;
var AXIS_SIZE  = 0.1;

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
	var bodies = [];
	var robot = null, new_robot = null;

	ws.onmessage = function(e) {
		var msg = JSON.parse(e.data);
		msg.forEach(function(m) {
			var matches = m[0].match("^" + KEY_PREFIX + "(.*)$");
			if (matches === null) return;
			var key = matches[1];
			var val = m[1];

			if (key == KEY_ROBOT) {
				new_robot = JSON.parse(val);
				var is_new_robot = false;
				if (robot != null) {
					var types = robot.links.map(link => link.type);
					var new_types = new_robot.links.map(link => link.type);
					is_new_robot = types.toString() != new_types.toString();
				}
				if (robot === null || is_new_robot) {
					delete_robot();
					create_robot(new_robot);
				}
				robot = new_robot;
				new_robot = null;
				update_robot(robot);
				return;
			}

			var $form = $("form[data-key='" + key + "']");

			// Create new redis key-value form
			if ($form.length == 0) {
				var form = htmlForm(key, val, true);
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
			updateHtmlValues($form, val);
		});
	};

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

	init_graphics();

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
			scene.add(body);
		});

		// Create end-effector
		var ee_frame = new THREE.Object3D();
		var ee_geometry = new THREE.SphereGeometry(0.01, 32, 32);
		var ee_material = new THREE.MeshNormalMaterial();
		var ee_mesh = new THREE.Mesh(ee_geometry, ee_material);
		ee_frame.position.set(robot.ee.pos[0], robot.ee.pos[1], robot.ee.pos[2]);

		ee_frame.add(ee_mesh);
		ee_frame.add(axes(AXIS_SIZE, AXIS_WIDTH));
		bodies[bodies.length-1].add(ee_frame);

		renderer.render(scene, camera);

	}

	function delete_robot() {

		scene.remove(bodies[0]);
		bodies = [];

	}

	function update_robot(robot) {

		for (var i = 0; i < robot.links.length; i++) {
			var link = robot.links[i];
			var quat = new THREE.Quaternion(link.quat[0], link.quat[1], link.quat[2], link.quat[3]);
			var pos  = new THREE.Quaternion(link.pos[0], link.pos[1], link.pos[2], 0);
			bodies[i].quaternion.copy(quat);
			bodies[i].position.copy(pos);
		}

		renderer.render(scene, camera);

	}

	$(window).resize(function() {
		var width = window.innerWidth - $("#sidebar").width();
		var height = window.innerHeight;
		camera.aspect = width / height;
		camera.updateProjectionMatrix();
		renderer.setSize(width, height);
		renderer.render(scene, camera);
	})

});
