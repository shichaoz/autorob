//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}

    // STENCIL: reference matrix code has the following functions:
    //   matrix_multiply
    //   matrix_transpose
    //   matrix_pseudoinverse
    //   matrix_invert_affine
    //   vector_normalize
    //   vector_cross
    //   generate_identity
    //   generate_translation_matrix
    //   generate_rotation_matrix_X
    //   generate_rotation_matrix_Y
    //   generate_rotation_matrix_Z

function matrix_multiply(m1,m2) { //m[0].length is the number of columns, m.length is the number of rows 
    if (m1[0].length !== m2.length){
        return 0;
    }
    var mat = [];
    var i,j;
    for (i=0;i<m1.length;i++) { 
        mat[i] = [];
        for (j=0;j<m2[0].length;j++) { 
            mat[i][j]=0;
            for (k=0;k<m2.length;k++){
            mat[i][j] += m1[i][k]*m2[k][j];
            }
        }
    }
    return mat;
}

function matrix_transpose(m1) {    
    var mat = [];
    var i,j;
    for (i=0;i<m1[0].length;i++) { 
        mat[i] = [];
        for (j=0;j<m1.length;j++) { 
            mat[i][j] = m1[j][i];
        }
    }
    return mat;
}

function matrix_pseudoinverse(m1) {
    if (m1.length >= m1[0].length ){ //tall matrix
        var mTm = matrix_multiply(matrix_transpose(m1),m1);
        var inv_mTm = numeric.inv(mTm);
        var mat = matrix_multiply(inv_mTm,matrix_transpose(m1))
    }
    if (m1.length < m1[0].length ){ //wide matrix
        var mmT = matrix_multiply(m1,matrix_transpose(m1));
        var inv_mmT = numeric.inv(mmT);
        var mat = matrix_multiply(matrix_transpose(m1),inv_mmT);
    }

    return mat;
}

function matrix_invert_affine(m1) {
    var M = [];
    var b = [];
    var i,j;
    for (i=0;i<3;i++) { // for each row of m1
        M[i] = [];
        b[i] = [];
        for (j=0;j<4;j++) { // for each column of m1
            if (j<3){
                M[i][j] = m1[i][j];
            }else{
                b[i][0] = m1[i][j];
            }
        }
    }
    var inv_M = numeric.inv(M)
    // var inv_M = matrix_inverse(M);
    var right_negative = matrix_multiply(inv_M,b);
    var mat = [];
    for (i=0;i<3;i++) { // for each row of mat
        mat[i] = [];
        for (j=0;j<4;j++) { // for each column of mat
            if (j<3){
                mat[i][j] = inv_M[i][j];
            }else{
                mat[i][j] = - right_negative[i][0];
            }
        }
    }
    mat[3] = [];
    for (j=0;j<4;j++) { // for each column of mat
        if (j<3){
            mat[3][j] = 0;
        }else{
            mat[3][j] = 1;
        }
    }
    return mat;
}

function vector_normalize(v1) {
    var i;
    var sum = 0;
    for (i=0;i<v1.length;i++) { // sum square of each element
        sum += v1[i]*v1[i];
    }
    var length_vector=Math.sqrt(sum);
    var vec=[];
    for (j=0;j<v1.length;j++) { 
        vec[j]=v1[j]/length_vector;
    }
    return vec;
}
function vector_cross(v1,v2){
    var v = new Array(v1.length);
    v[0] = v1[1]*v2[2] - v1[2]*v2[1];
    v[1] = - ( v1[0]*v2[2] - v1[2]*v2[0] );
    v[2] = v1[0]*v2[1] - v1[1]*v2[0];
    return v;
}

function generate_identity() {//generate a 4-by-4 identity matrix
    var mat = [];
    var i,j;
    for (i=0;i<4;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<4;j++) { // for each column of m1
            if (i===j){
                mat[i][j] = 1;
            }else{
                mat[i][j] = 0;
            }
        }
    }
    return mat;
}


function generate_translation_matrix(a1,a2,a3) {//generate a 4-by-4 identity matrix
    var mat = generate_identity();
    var translation=[a1,a2,a3];
    var i;
    for (i=0;i<3;i++) { // for each column of m1
        mat[i][3] += translation[i];
    }
    return mat;
}
function generate_rotation_matrix_X(theta) {//generate a 4-by-4 identity matrix
    var mat = generate_identity();
    mat[1][1]=Math.cos(theta);
    mat[2][2]=Math.cos(theta);
    mat[1][2]=-Math.sin(theta);
    mat[2][1]=Math.sin(theta);
    return mat;
}
function generate_rotation_matrix_Y(theta) {//generate a 4-by-4 identity matrix
    var mat = generate_identity();
    mat[0][0]=Math.cos(theta);
    mat[2][2]=Math.cos(theta);
    mat[0][2]=Math.sin(theta);
    mat[2][0]=-Math.sin(theta);
    return mat;
}
function generate_rotation_matrix_Z(theta) {//generate a 4-by-4 identity matrix
    var mat = generate_identity();
    mat[0][0]=Math.cos(theta);
    mat[1][1]=Math.cos(theta);
    mat[0][1]=-Math.sin(theta);
    mat[1][0]=Math.sin(theta);
    return mat;
}




// edit on 11.9 assignment 5

function matrix_vector_multiply(m1,v1){
    var v = [];
    var i;
    var j;
    v1 = [v1[0],v1[1],v1[2],0];
    for (i=0;i<m1.length;++i){
        v[i] = 0;
        for (j=0;j<m1[0].length;++j){
            v[i] += m1[i][j]*v1[j];
        }
    }
    v = [v[0],v[1],v[2]];
    return v;
}

function xform_to_theta(xform){
    
    var theta1=[Math.atan2( xform[2][1], xform[2][2])];
    var theta3=[Math.atan2( xform[1][0], xform[0][0])];
    var temp = Math.pow(xform[2][1], 2)+ Math.pow(xform[2][2], 2);
    temp = Math.pow(temp,0.5);
    var theta2=[Math.atan2(-xform[2][0],temp)];
    
    var theta = [theta1,theta2,theta3];
    return theta;
}

function vector_minus(v1,v2) {
    var v = [];
    for (i = 0; i<v1.length; ++i) {
        v[i] = v1[i]-v2[i];
    }
    return v;
}



