//////////////////////////////////////////////////
/////     FORWARD KINEMATICS
//////////////////////////////////////////////////

// CS148: compute and draw robot kinematics (.xform matrix for each link)
// CS148: compute and draw robot heading and lateral vectors for base movement in plane
// matrix_2Darray_to_threejs converts a 2D JavaScript array to a threejs matrix
//   for example: var tempmat = matrix_2Darray_to_threejs(link.xform);
// simpleApplyMatrix transforms a threejs object by a matrix
//   for example: simpleApplyMatrix(link.geom,tempmat);

/*
CS148: reference code has functions for:

robot_forward_kinematics
traverse_forward_kinematics_link
traverse_forward_kinematics_joint
compute_and_draw_heading
*/

// robot.origin.xform: global position and orientation of robot
// robot.links[xx].xform: transform of link in global coordinates
// robot.joints[xx].origin.xform: transform of joint origin in global coordinates
// robot.joints[xx].xform: transform resulting from joint rotation in global coordinates.

function robot_forward_kinematics (robot) {
	compute_and_draw_heading();
	var old_transform = matrix_multiply(matrix_multiply(matrix_multiply(generate_translation_matrix(robot.origin.xyz), generate_rotation_matrix_X(robot.origin.rpy[0])),
					generate_rotation_matrix_Y(robot.origin.rpy[1])), 
	 				generate_rotation_matrix_Z(robot.origin.rpy[2]));

	traverse_forward_kinematics_link(robot, old_transform, robot.base);
}

function traverse_forward_kinematics_link (robot, current_transform, link) {
    robot.links[link].xform = current_transform;
		var length = robot.links[link].children.length;
		for (var i = 0; i < length; i++) {
			traverse_forward_kinematics_joint(robot, current_transform, robot.links[link].children[i]);
		}

}

function traverse_forward_kinematics_joint (robot, current_transform, joint) {
	var new_transform = matrix_multiply(matrix_multiply(matrix_multiply(generate_translation_matrix(robot.joints[joint].origin.xyz), generate_rotation_matrix_X(robot.joints[joint].origin.rpy[0])),
						generate_rotation_matrix_Y(robot.joints[joint].origin.rpy[1])), 
						generate_rotation_matrix_Z(robot.joints[joint].origin.rpy[2]));

	var transform = matrix_multiply(current_transform, new_transform);

	robot.joints[joint].origin.xform = transform;
	var Q = quaternion_from_axisangle (robot.joints[joint].axis, robot.joints[joint].angle);
    robot.joints[joint].xform = matrix_multiply(robot.joints[joint].origin.xform, quaternion_to_rotation_matrix(Q));

	traverse_forward_kinematics_link(robot, robot.joints[joint].xform, robot.links[robot.joints[joint].child].name);
}

function compute_and_draw_heading () {
	
	var new_transform = matrix_multiply(matrix_multiply(matrix_multiply(generate_translation_matrix(robot.origin.xyz), generate_rotation_matrix_X(robot.origin.rpy[0])),
						generate_rotation_matrix_Y(robot.origin.rpy[1])), 
						generate_rotation_matrix_Z(robot.origin.rpy[2]));

	var head = matrix_multiply(new_transform, generate_translation_matrix([0,0,1]));
	var lat = matrix_multiply(new_transform, generate_translation_matrix([1,0,0]));
	//console.log("HEAD");
	//console.log(head);

	robot_heading = [[head[0][3]], [head[1][3]], [head[2][3]]];
	robot_lateral = [[lat[0][3]], [lat[1][3]], [lat[2][3]]];

	if (typeof heading_geom === 'undefined') {
            var temp_geom = new THREE.CubeGeometry(0.3, 0.3, 0.3);
            var temp_material = new THREE.MeshBasicMaterial( {color: 0x00ffff} )
            heading_geom = new THREE.Mesh(temp_geom, temp_material);
            scene.add(heading_geom);
    }
    if (typeof lateral_geom === 'undefined') {
        var temp_geom = new THREE.CubeGeometry(0.3, 0.3, 0.3);
        var temp_material = new THREE.MeshBasicMaterial( {color: 0x008888} )
        lateral_geom = new THREE.Mesh(temp_geom, temp_material);
        scene.add(lateral_geom);
    }

    simpleApplyMatrix(heading_geom, matrix_2Darray_to_threejs(head));
    simpleApplyMatrix(lateral_geom, matrix_2Darray_to_threejs(lat));


}