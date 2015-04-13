//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

/*
CS148: reference code has functions for:

quaternion_from_axisangle
quaternion_normalize
quaternion_multiply
quaternion_to_rotation_matrix
*/


function quaternion_from_axisangle (axis, angle) {
	var resultQuaternion = [];

  	resultQuaternion[1] = axis[0] * Math.sin(angle/2);
  	resultQuaternion[2] = axis[1] * Math.sin(angle/2);
  	resultQuaternion[3] = axis[2] * Math.sin(angle/2);
  	resultQuaternion[0] = Math.cos(angle/2);
  	return resultQuaternion;


}

function quaternion_normalize (Q) {
	var mag = Math.sqrt(Q[0] + Q[1] + Q[2] + Q[3]);
	Q[0] = Q[0] / mag;
	Q[1] = Q[1] /  mag;
	Q[2] = Q[2] / mag;
	Q[3] = Q[3] / mag;
	return Q;
}

function quaternion_multiply (A, B) {
	var Q = [];
	var a = A[0];
	var b = A[1];
	var c = A[2];
	var d = A[3];
	var e = B[0];
	var f = B[1];
	var g = B[2];
	var h = B[3];
	Q[0] = a*e-b*f-c*g-a*h;
	Q[1] = a*f + b*e + c*h + d*g;
	Q[2] = a*g - b*h + c*e+d*f;
	Q[3] = a*h + b*g - c*f + d*e;
	return Q[0]+Q[1]+Q[2]+Q[3];
	
}

function quaternion_to_rotation_matrix (Q) {
	var I = generate_identity(4);
	I[0][0] = 1-2*(Q[2]*Q[2] + Q[3]*Q[3]);
	I[1][0] = 2*(Q[1]*Q[2] - Q[0]*Q[3]);
	I[2][0] = 2*(Q[0]*Q[2] + Q[1]*Q[3]);

	I[0][1] = 2*(Q[1]*Q[2] + Q[0]*Q[3]);
	I[1][1] = 1-2*(Q[1]*Q[1] + Q[3]*Q[3]);
	I[2][1] = 2*(Q[2]*Q[3] - Q[0]*Q[1]);

	I[0][2] = 2*(Q[1]*Q[3] - Q[0]*Q[2]);
	I[1][2] = 2*(Q[0]*Q[1] + Q[2]*Q[3]);
	I[2][2] = 1-2*(Q[1]*Q[1] + Q[2]*Q[2]);

	return I;
}