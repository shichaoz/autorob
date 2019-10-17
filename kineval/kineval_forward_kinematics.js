
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }
    kineval.buildFKTransforms();
    // STENCIL: implement kineval.buildFKTransforms();

}

kineval.buildFKTransforms = function buildFKTransforms(){
    traverseFKBase();
    for (var  i = 0; i < robot.links[robot.base].children.length; i++){
        traverseFKJoint(robot.links[robot.base].children[i]);
    }

}

function traverseFKBase(){
    /*robot.links[robot.base].xform=matrix_multiply(generate_translation_matrix(robot.origin.xyz[0],robot.origin.xyz[1],robot.origin.xyz[2]),
        matrix_multiply(generate_rotation_matrix_Z(robot.origin.rpy[2]),
            matrix_multiply(generate_rotation_matrix_Y(robot.origin.rpy[1]),generate_rotation_matrix_X(robot.origin.rpy[0]))));*/
    var mR_middle=matrix_multiply(generate_rotation_matrix_Y(robot.origin.rpy[1]),generate_rotation_matrix_X(robot.origin.rpy[0]));        
    var mR = matrix_multiply(generate_rotation_matrix_Z(robot.origin.rpy[2]),mR_middle);
    var mT = generate_translation_matrix(robot.origin.xyz[0], robot.origin.xyz[1], robot.origin.xyz[2]);
    robot.links[robot.base].xform = matrix_multiply(mT,mR);



    if (robot.links_geom_imported){
        robot.links[robot.base].xform = matrix_multiply(robot.links[robot.base].xform,
            matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2)));
    }
}

function traverseFKJoint(curjoint){
    /*var robot.joints[curjoint].xform = matrix_multiply(generate_translation_matrix(robot.joints[curjoint].origin.xyz[0],robot.joints[curjoint].origin.xyz[1],robot.joints[curjoint].origin.xyz[2]),
        matrix_multiply(generate_rotation_matrix_Z(robot.joints[curjoint].origin.rpy[2]),
            matrix_multiply(generate_rotation_matrix_Y(robot.joints[curjoint].origin.rpy[1]),generate_rotation_matrix_X(robot.joints[curjoint].origin.rpy[0]))));*/
    var mR_middle=matrix_multiply(generate_rotation_matrix_Y(robot.joints[curjoint].origin.rpy[1]),generate_rotation_matrix_X(robot.joints[curjoint].origin.rpy[0]));        
    var mR = matrix_multiply(generate_rotation_matrix_Z(robot.joints[curjoint].origin.rpy[2]),mR_middle);
    var mT = generate_translation_matrix(robot.joints[curjoint].origin.xyz[0],robot.joints[curjoint].origin.xyz[1],robot.joints[curjoint].origin.xyz[2]);
    robot.joints[curjoint].xform = matrix_multiply(robot.links[robot.joints[curjoint].parent].xform,matrix_multiply(mT,mR));

    traverseFKLink(robot.joints[curjoint].child);
}

function traverseFKLink(curlink){
    robot.links[curlink].xform = robot.joints[robot.links[curlink].parent].xform;
    if (typeof robot.links[curlink].children !== 'undefined'){
        for (var i = 0; i < robot.links[curlink].children.length; i++){
            traverseFKJoint(robot.links[curlink].children[i]);
        }
    }
}


    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //

