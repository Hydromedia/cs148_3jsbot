//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////


//CS148: reference code has functions for:

function matrix_multiply (A, B) {
	var resultMatrix = [],
		resultWidth = B[0].length;
		resultHeight = A.length;
	for (var i = 0; i < resultHeight; i++) {
		resultMatrix[i] = [];
		for (j = 0; j < resultWidth; j++){
			var total = 0;
			for (var p = 0; p < A[0].length; p++) {
                total += A[i][p] * B[p][j];
            }
            resultMatrix[i][j] = total;
		}
	}
	return resultMatrix;
}

var A = [[1, 2, 3, 4],
		 [1, 2, 3, 4]];

var B = [[1, 2, 3],
		 [4, 5, 6],
		 [7, 8, 9],
		 [10, 11, 12]];

var C = matrix_transpose(B);
function print_array (C) {
	for (i = 0; i < A.length; i++) {
		for (j = 0; j < A[0].length; j++) {
			console.log(A[i][j]);
		}
	}
}


function generate_empty(height, width){
	var resultMatrix = new Array(height);
	for (i = 0; i < height; i++) {
		resultMatrix[i] = new Array(width);
		for (j = 0; j < width; j++) {
			resultMatrix[i][j] = 0;
		}
	}
	return resultMatrix;
}

function matrix_transpose (A) {
	var resultMatrix = [];
		width = A[0].length;
		height = A.length;
	resultMatrix = generate_empty(height, width);
	for (i = 0; i < height; i++) {
		for (j = 0; j < width; j++) {
			resultMatrix[j][i] = A[i][j];
		}
	}
	return resultMatrix;
}

function vector_normalize (A) {

	for (i = 0; i < A.length; i++) {

	}
	var x = A[0];
	var y = A[1];
	var z = A[2];

	var length_squared = x*x + y*y + z*z;
	var length = Math.sqrt(length_square);
	var B = [x/length, y/length, z/length];
	return B;
}

function vector_cross (A, B) {
	A
	y * vec.z - z * vec.y, z * vec.x - x * vec.z, x * vec.y - y * vec.x;
}

function generate_identity(size) {
	var resultMatrix = new Array(size);
	for (i = 0; i < height; i++) {
		resultMatrix[i] = new Array(size);
		for (j = 0; j < width; j++) {
			if (i == j){
				resultMatrix[i][j] = 1;
			} else {
				resultMatrix[i][j] = 0;
			}
		}
	}
	return resultMatrix;
}

function generate_translation_matrix () {
	
}

function generate_rotation_matrix_X () {
	
}

function generate_rotation_matrix_Y () {
	
}

function generate_rotation_matrix_Z () {
	
}