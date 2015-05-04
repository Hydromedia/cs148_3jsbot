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
		[5, 6, 7, 8],
		[9, 9, 11, 12],
		[24, 25, 46, 75],
		[10,9,8,7]];

var B = [[1, 2, 3, 4],
		[5, 6, 7, 8],
		[9, 9, 11, 12],
		[24, 25, 46, 75]];
//console.log(matrix_multiply(A, B));
var C = matrix_multiply(A, B);
console.log(C);
function print_array (C) {
	for (i = 0; i < A.length; i++) {
		for (j = 0; j < A[0].length; j++) {
			console.log(A[i][j]);
		}
	}
}

console.log("Empty Test");
console.log(generate_empty(4, 6));
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

var D = [[1, 2, 3, 4],
		[5, 6, 7, 8],
		[9, 9, 11, 12],
		[24, 25, 46, 75],
		[10,9,8,7]];
console.log("Transpose Test:");
console.log(matrix_transpose(D));
function matrix_transpose (A) {
	var resultMatrix = [];
		oldCols = A[0].length;
		oldRows = A.length;
	resultMatrix = generate_empty(oldCols, oldRows);
	for (i = 0; i < oldRows; i++) {
		for (j = 0; j < oldCols; j++) {
			resultMatrix[j][i] = A[i][j];
		}
	}
	return resultMatrix;
}

function vector_normalize (A) {

	var B = Array(A.length);
	var total = 0;
	for (i = 0; i < A.length; i++) {
		total += A[i]*A[i];
	}
	var length_squared = total;
	var length = Math.sqrt(length_squared);
	for (i = 0; i < B.length; i++) {
		B[i]= A[i]/length;
	}
	return B;
}

function vector_cross (A, B) {
	var C = [A[1] * B[2] - A[2] * B[1], A[2] * B[0] - A[0] * B[2], A[0] * B[1] - A[1] * B[0]];
	return C;
}

function generate_identity(size) {
	var resultMatrix = new Array(size);
	for (i = 0; i < size; i++) {
		resultMatrix[i] = new Array(size);
		for (j = 0; j < size; j++) {
			if (i == j){
				resultMatrix[i][j] = 1;
			} else {
				resultMatrix[i][j] = 0;
			}
		}
	}
	return resultMatrix;
}

function generate_translation_matrix (A) {
	var I = generate_identity(4);
	//Math.
	I[0][3] = A[0];
	I[1][3] = A[1];
	I[2][3] = A[2];
	return I;
}

var A = [5, 6, 7];
console.log("test:");
console.log(generate_translation_matrix(A));

function generate_rotation_matrix_X (theta) {
	var I = generate_identity(4);
	I[1][1] = Math.cos(theta);
	I[1][2] = 0 - Math.sin(theta);
	I[2][1] = Math.sin(theta);
	I[2][2] = Math.cos(theta);
	return I;
}

function generate_rotation_matrix_Y (theta) {
	var I = generate_identity(4);
	I[0][0] = Math.cos(theta);
	I[0][2] = Math.sin(theta);
	I[2][0] = 0 - Math.sin(theta);
	I[2][2] = Math.cos(theta);
	return I;
}

function generate_rotation_matrix_Z (theta) {
	var I = generate_identity(4);
	I[0][0] = Math.cos(theta);
	I[0][1] = 0 - Math.sin(theta);
	I[1][0] = Math.sin(theta);
	I[1][1] = Math.cos(theta);
	return I;
}