//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    //   quaternion_normalize
    //   quaternion_to_rotation_matrix
    //   quaternion_multiply

function quaternion_from_axisangle(theta,u) {
	return [ Math.cos(theta/2), u[0]*Math.sin(theta/2), 
	u[1]*Math.sin(theta/2), u[2]*Math.sin(theta/2)];
}

function quaternion_normalize (q) { 
    var norm = Math.sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    var q_normalized= [];
    q_normalized[0]=q[0]/norm;
    q_normalized[1]=q[1]/norm;
    q_normalized[2]=q[2]/norm;
    q_normalized[3]=q[3]/norm;
    return q_normalized;
}

function   quaternion_to_rotation_matrix(q){
    var temp = generate_identy(4);
    temp[0][0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
    temp[0][1] = 2*( q[1]*q[2] - q[0]*q[3] );
    temp[0][2] = 2*( q[0]*q[2] + q[1]*q[3] );
    temp[1][0] = 2*( q[1]*q[2] + q[0]*q[3] );
    temp[1][1] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
    temp[1][2] = 2*( q[3]*q[2] - q[0]*q[1] );
    temp[2][0] = 2*( q[1]*q[3] - q[0]*q[2] );
    temp[2][1] = 2*( q[1]*q[0] + q[2]*q[3] );
    temp[2][2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    return temp;

}

function quaternion_multiply (q1,q2) { 
	var q = new Array(q1.length);
    var a = q1[0];
    var b = q1[1];
    var c = q1[2];
    var d = q1[3];
    var e = q2[0];
    var f = q2[1];
    var g = q2[2];
    var h = q2[3];
    q = [(a*e-b*f-c*g-d*h),(a*f+b*e+c*h-d*g),(a*g-b*h+c*e+d*f),(a*h+b*g-c*f+d*e)];
    return q;
}