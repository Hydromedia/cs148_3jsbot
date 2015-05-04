//////////////////////////////////////////////////
/////     INVERSE KINEMATICS 
/////     Resolved-rate IK with geometric jacobian
//////////////////////////////////////////////////

// CS148: generate joint controls to move robot to move robot endeffector to target location

/*
CS148: reference code has functions for:

robot_inverse_kinematics
iterate_inverse_kinematics
*/

function update_joint_angles (angles, joint, index) {
	robot.joints[joint].control += .1*(angles[index][0]);
	//console.log("Update Angles: " + angles[index][0]);
	//console.log("Update Angles: " + angles[index][0]);
	index+=1;
	if (index < angles.length){
		update_joint_angles(angles, robot.joints[robot.links[robot.joints[joint].parent].parent].name, index);
	}
}

function robot_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos) {
    // compute joint angle controls to move location on specified link to Cartesian location

    //Basic Flow:
    // call inverse kimematics; parse inputs and pass the correct things to the recursive IK call
    //that should make the jacobian
    //then back in inversekinematics do what you need to with the jacobian
    //to be able to pass the right stuff onto the thing that updates the angles
    //console.log("robot_inverse_kinematics1");
    
	//console.log("robot_inverse_kinematics3");

    if (update_ik) {
    	jacobian = [[],[],[],[],[],[]];
    	//console.log("iterate start");

    	var vec4_endeffector_local_pos = generate_empty(4,1);
		vec4_endeffector_local_pos[0][0] = endeffector_local_pos[0][0];
		vec4_endeffector_local_pos[1][0] = endeffector_local_pos[1][0];
		vec4_endeffector_local_pos[2][0] = endeffector_local_pos[2][0];
		vec4_endeffector_local_pos[3][0] = 1;

		//console.log("robot_inverse_kinematics2");
		//console.log(vec4_endeffector_local_pos);
		//console.log(robot.joints[endeffector_joint].xform);
		abcd = matrix_multiply(robot.joints[endeffector_joint].xform, vec4_endeffector_local_pos);
		//console.log("ABCD!@#!#@!");
		//console.log(abcd);
        iterate_inverse_kinematics(jacobian, target_pos, endeffector_joint, endeffector_local_pos);
        //console.log("iterate end");
    	//jacobian = jacobian.splice(0, 1);
        var transpose = false;
        if (transpose){
	        var forward_jacobian_Nx3 = generate_empty(jacobian[0].length,3);
	        for(var i=0; i< jacobian[0].length;i++){
		        var size=jacobian[0].length;
		        forward_jacobian_Nx3[i][0]=jacobian[0][i];
		        forward_jacobian_Nx3[i][1]=jacobian[1][i];
		        forward_jacobian_Nx3[i][2]=jacobian[2][i];
	    	}
	    	//console.log(forward_jacobian_3N);
	    	//console.log(forward_jacobian_3N.length);
	    	var delta = generate_empty(3, 1);
			delta[0][0] = target_pos[0][0] - abcd[0][0];
			delta[1][0] = target_pos[1][0] - abcd[1][0];
			delta[2][0] = target_pos[2][0] - abcd[2][0];
	    	var angles = matrix_multiply(forward_jacobian_Nx3, delta);
	    	//console.log("Final angles");
	    	//console.log(angles);
	    	console.log(angles.length);
	    	//console.log(jacobian);
	    	update_joint_angles(angles, endeffector_joint, 0);
    	} else {
    		var A = jacobian;
    		var At = matrix_transpose(jacobian);
    		var pseudo_inv = matrix_multiply(numeric.inv(matrix_multiply(At,A)), At);
			console.log("JJ!!");
    		//console.log(matrix_transpose(jacobian));
    		console.log(pseudo_inv);
    		var forward_jacobian_Nx3 = generate_empty(pseudo_inv.length,3);
	        for(var i=0; i< pseudo_inv.length;i++){
		        forward_jacobian_Nx3[i][0]=pseudo_inv[i][0];
		        forward_jacobian_Nx3[i][1]=pseudo_inv[i][1];
		        forward_jacobian_Nx3[i][2]=pseudo_inv[i][2];
	    	}
	    	//console.log(forward_jacobian_3N);
	    	//console.log(forward_jacobian_3N.length);
	    	var delta = generate_empty(3, 1);
			delta[0][0] = target_pos[0][0] - abcd[0][0];
			delta[1][0] = target_pos[1][0] - abcd[1][0];
			delta[2][0] = target_pos[2][0] - abcd[2][0];
	    	var angles = matrix_multiply(forward_jacobian_Nx3, delta);
	    	//console.log("Final angles");
	    	//console.log(angles);
	    	console.log(angles.length);
	    	//console.log(jacobian);
	    	update_joint_angles(angles, endeffector_joint, 0);
    	}

    	endeffector_geom.visible = true;
    	//console.log("Gen Tran");
    	//console.log(generate_translation_matrix([endeffector_world_coordinate[0][0], endeffector_world_coordinate[1][0], endeffector_world_coordinate[2][0]]));
    	simpleApplyMatrix(endeffector_geom, matrix_2Darray_to_threejs(
    		generate_translation_matrix([abcd[0][0], abcd[1][0], abcd[2][0]])));
        target_geom.visible = true;
        simpleApplyMatrix(target_geom, matrix_2Darray_to_threejs(generate_translation_matrix([target_pos[0][0], target_pos[1][0], target_pos[2][0]])));
        update_ik = false;
        //alert("done");
    }
    else {
        endeffector_geom.visible = false;
        target_geom.visible = false;
    }
    update_ik = false;

}

function iterate_inverse_kinematics(jacobian, target_pos, joint, endeffector_local_pos) {
	//console.log("iterate");
	//console.log(joint);
	/*
	 so like the idea is you want each of the N columns to contain this derivative stuff
	It takes tree to tango.: but what that really is
	It takes tree to tango.: is like axis cross (endeffector location- joint location)
	*/

	var axis = generate_empty(4, 1);
	axis[0][0] = robot.joints[joint].axis[0];
	axis[1][0] = robot.joints[joint].axis[1];
	axis[2][0] = robot.joints[joint].axis[2];
	axis[3][0] = 1;
	//console.log("axis");
	//console.log(axis);


	var rotation_matrix = robot.joints[joint].origin.xform;
	rotation_matrix[0][3] = 0;
	rotation_matrix[1][3] = 0;
	rotation_matrix[2][3] = 0;

	axis = matrix_multiply(rotation_matrix, axis);


	var vec4_endeffector_local_pos = generate_empty(4,1);
	vec4_endeffector_local_pos[0][0] = endeffector_local_pos[0][0];
	vec4_endeffector_local_pos[1][0] = endeffector_local_pos[1][0];
	vec4_endeffector_local_pos[2][0] = endeffector_local_pos[2][0];
	vec4_endeffector_local_pos[3][0] = 1;

	//console.log("angle");
	//console.log(robot.joints[joint].angle);

	// console.log("robot.joints[joint].xform");
	// console.log(robot.joints[joint].xform);
	// console.log("vec4_endeffector_local_pos");
	// console.log(vec4_endeffector_local_pos);
	var endeffector_world_coordinate = matrix_multiply(robot.joints[joint].xform, vec4_endeffector_local_pos);
	// console.log("endeffector_world_coordinate");
	// console.log(endeffector_world_coordinate);

	// endeffector_world_position = generate_empty(1,3);
	// endeffector_world_position[0][0] = endeffector_world_coordinate[0][0];
	// endeffector_world_position[0][1] = endeffector_world_coordinate[0][1];
	// endeffector_world_position[0][2] = endeffector_world_coordinate[0][2];
	//endeffector_world_position[0][3] = 1;

	var joint_world_position = generate_empty(1,3);
	joint_world_position[0][0] = robot.joints[joint].origin.xform[0][3];
	joint_world_position[0][1] = robot.joints[joint].origin.xform[1][3];
	joint_world_position[0][2] = robot.joints[joint].origin.xform[2][3];
	//joint_world_position[0][3] = 1;

	var delta = [endeffector_world_coordinate[0][0] - joint_world_position[0][0],
				 endeffector_world_coordinate[1][0] - joint_world_position[0][1],
				 endeffector_world_coordinate[2][0] - joint_world_position[0][2]];

	//console.log("delta");
	//console.log(delta);
	//delta[0][0] = endeffector_world_position[0][0] - joint_world_position[0][0];
	//delta[1][0] = endeffector_world_position[0][1] - joint_world_position[0][1];
	//delta[2][0] = endeffector_world_position[0][2] - joint_world_position[0][2];
	//delta[0][3] = 1;

	var vector_axis = [axis[0][0], axis[1][0], axis[2][0]];
	var result1 = vector_cross(axis, delta);


	//var result2 = generate_empty(1,6);
	jacobian[0].push(result1[0]);
	jacobian[1].push(result1[1]);
	jacobian[2].push(result1[2]);
	jacobian[3].push(vector_axis[0]);
	jacobian[4].push(vector_axis[1]);
	jacobian[5].push(vector_axis[2]);
	// result2[0][0] = result1[0];
	// result2[0][1] = result1[1];
	// result2[0][2] = result1[2];
	// result2[0][3] = vector_axis[0];
	// result2[0][4] = vector_axis[1];
	// result2[0][5] = vector_axis[2];

	var forward_jacobian_3N = generate_empty(1,3);
	//for(var i=0; i< jacobian.length;i++){
        //var size=jacobian.length;
    forward_jacobian_3N[0][0]=result1[0];
    forward_jacobian_3N[0][1]=result1[1];
    forward_jacobian_3N[0][2]=result1[2];
	//}
	//console.log(forward_jacobian_3N);
	//console.log(forward_jacobian_3N.length);
	var delta2 = generate_empty(3, 1);
	delta2[0][0] = target_pos[0][0] - abcd[0][0];
	delta2[1][0] = target_pos[1][0] - abcd[1][0];
	delta2[2][0] = target_pos[2][0] - abcd[2][0];
	var angles = matrix_multiply(forward_jacobian_3N, delta2);
	//console.log("ANG!");
	//console.log(angles[0]);
	//update_joint_angles(angles, endeffector_joint, 0);


	//jacobian.push(result2);
	//console.log(jacobian);
	//alert("stop");
	if (robot.links[robot.joints[joint].parent].parent !== undefined) {
		iterate_inverse_kinematics(jacobian, target_pos, robot.joints[robot.links[robot.joints[joint].parent].parent].name, endeffector_local_pos);
	}
	
}
