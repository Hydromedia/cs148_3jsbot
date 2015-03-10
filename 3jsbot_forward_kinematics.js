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
	if (robot.joints !== null && robot.joints[0] !== null){
		var joint = robot.joints[0];
		var old_transform = matrix_multiply(matrix_multiply(matrix_multiply(generate_translation_matrix(joint.xyz), generate_rotation_matrix_X(joint.rpy[0])),
						generate_rotation_matrix_Y(joint.rpy[1])), 
						generate_rotation_matrix_Z(joint.rpy[2]));

		robot.origin.xform = old_transform;

		traverse_forward_kinematics_joint(robot, robot.joints[0]);
	}
}

function traverse_forward_kinematics_link (robot, current_transform, link) {
    robot.links[link].xform = current_transform;
	for (x in robot.links[link].children) {
		if (x !== null && robot.joints[x] !== null){
			traverse_forward_kinematics_link(robot, current_transform, x);
		}
	}

}

function traverse_forward_kinematics_joint (robot, current_transform, joint) {
	
	var old_transform = matrix_multiply(matrix_multiply(matrix_multiply(generate_translation_matrix(joint.xyz), generate_rotation_matrix_X(joint.rpy[0])),
						generate_rotation_matrix_Y(joint.rpy[1])), 
						generate_rotation_matrix_Z(joint.rpy[2]));

	joint.transform = matrix_multiply(current_transform, old_transform);

	robot.joints[xx].origin.xform = matrix_multiply(current_transform, old_transform);
    robot.joints[xx].xform = matrix_multiply(current_transform, old_transform);

	if (joint.child !== null && robot.links[joint.child] !== null){
		traverse_forward_kinematics_link(robot, joint.transform, robot.links[joint.child]);
	}
}

function compute_and_draw_heading () {
	
}