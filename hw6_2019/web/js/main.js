/**
 * redis-web-gui.js
 *
 * Author: Toki Migimatsu
 * Created: December 2017
 */

function htmlForm(key, val, hidden) {
	hidden = false || hidden;
	var form = "<a name='" + key + "'></a><form data-key='" + key + "'><div class='keyval-card'>\n";
	form += "\t<div class='key-header'>\n";
	form += "\t\t<label>" + key + "</label>\n";
	form += "\t\t<div class='buttons'>\n";
	form += "\t\t\t<input type='button' value='Copy' class='copy' title='Copy value to clipboard'>\n";
	if (!hidden) {
		form += "\t\t\t<input type='submit' value='Set' title='Set values in Redis: <enter>'>\n";
	}
	form += "\t\t</div>\n";
	form += "\t</div>\n";
	form += "\t<div class='val-body'>\n";
	if (typeof(val) === "string") {
		form += "\t\t<div class='val-row'>\n";
		form += "\t\t\t<div class='val-string'>\n";
		form += "\t\t\t\t<input class='val' type='text' value='" + val + "'>\n";
		form += "\t\t\t</div>\n";
		form += "\t\t</div>\n";
	} else { // val should be a 2D array
		val.forEach(function(row, idx_row) {
			form += "\t\t<div class='val-row'>\n";
			row.forEach(function(el, idx) {
				var f = (Math.round(parseFloat(el) * 10000) / 10000).toString()
				form += "\t\t\t<input class='val' type='text' value='" + f + "'>\n";
			});
			form += "\t\t</div>\n";
		});
	}
	form += "\t</div>\n";
	form += "</div></form>\n";
	return form;
}

function updateHtmlValues($form, val) {
	// Update string
	var $inputs = $form.find("input.val");
	if (typeof(val) === "string") {
		$inputs.eq(0).val(val);
		return;
	}

	// Replace matrix if size has changed
	if (val.length * val[0].length != $inputs.length) {
		var key = $form.attr("data-key");
		var html = htmlForm(key, val);
		$form.html(html);
		return;
	}

	// Update matrix
	var i = 0;
	val.forEach(function(row) {
		row.forEach(function(el) {
			var f = (Math.round(parseFloat(el) * 10000) / 10000).toString()
			$inputs.eq(i).val(f);
			i++;
		});
	});
}

function getMatrix($form) {
	if ($form.find("div.val-string").length > 0)
		return $form.find("input.val").val();
	return $form.find("div.val-row").map(function() {
		return [$(this).find("input.val").map(function() {
			return $(this).val();
		}).get().filter(el => el != "")];
	}).get();
}

function fillMatrix(matrix, num) {
	matrix.forEach(function(row) {
		row.forEach(function(el, idx) {
			row[idx] = num.toString();
		});
	});
}

function matrixToString(matrix) {
	return matrix.map(function(row) {
		return row.join(" ");
	}).join("; ");
}

function matrixDim(val) {
	if (typeof(val) === "string") return "";
	return [val.length, val[0].length].toString();
}

// Send updated key-val pair via POST
function ajaxSendRedis(key_vals) {
	var data = {};
	if (arguments.length == 2) {
		data[key] = JSON.stringify(val);
	} else {
		for (var key in key_vals) {
			var val = key_vals[key];
			data[key] = JSON.stringify(key_vals[key]);
		}
	}

	console.log(data);
	$.ajax({
		method: "POST",
		url: "/",
		data: data
	});
}

function axes(size, line_width, colors) {
	colors = colors || [0xff0000, 0x00ff00, 0x0000ff];

	var xyz = new THREE.Object3D();

	var x_material = new MeshLineMaterial({
		color: new THREE.Color(colors[0]),
		lineWidth: line_width
	});
	var x_geometry = new THREE.Geometry();
	x_geometry.vertices.push(new THREE.Vector3(0, 0, 0));
	x_geometry.vertices.push(new THREE.Vector3(size, 0, 0));
	var x_line = new MeshLine();
	x_line.setGeometry(x_geometry);
	var x = new THREE.Mesh(x_line.geometry, x_material)

	var y_material = new MeshLineMaterial({
		color: new THREE.Color(colors[1]),
		lineWidth: line_width
	});
	var y_geometry = new THREE.Geometry();
	y_geometry.vertices.push(new THREE.Vector3(0, 0, 0));
	y_geometry.vertices.push(new THREE.Vector3(0, size, 0));
	var y_line = new MeshLine();
	y_line.setGeometry(y_geometry);
	var y = new THREE.Mesh(y_line.geometry, y_material)

	var z_material = new MeshLineMaterial({
		color: new THREE.Color(colors[2]),
		lineWidth: line_width
	});
	var z_geometry = new THREE.Geometry();
	z_geometry.vertices.push(new THREE.Vector3(0, 0, 0));
	z_geometry.vertices.push(new THREE.Vector3(0, 0, size));
	var z_line = new MeshLine();
	z_line.setGeometry(z_geometry);
	var z = new THREE.Mesh(z_line.geometry, z_material)

	xyz.add(x);
	xyz.add(y);
	xyz.add(z);
	return xyz;
}

ws_port = %(ws_port)s;
