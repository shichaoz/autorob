
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();
    // get endeffector Cartesian position in the world
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);
    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
          Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
          + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
          + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );
    // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
        kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
        kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
        kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
        kineval.params.trial_ik_random.targets += 1;
        textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }


}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration


    var endeXYZ_World = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_position_local);
    var endeRPY_World = xform_to_theta(robot.joints[endeffector_joint].xform);
    var ende_para = []; // save parameters for endeffector 
    var i;
    for (i = 0; i< 3; i++){
        ende_para[i] = [endeXYZ_World[i][0]];
        ende_para[i+3] = [endeRPY_World[i][0]];
    }
    var diff_para = [];
    for (i = 0; i< 3; i++){
        diff_para[i] = [endeffector_target_world.position[i] - ende_para[i][0]];
    }
    for (i = 3; i< 6; i++){
        if (kineval.params.ik_orientation_included){
            diff_para[i] = [endeffector_target_world.orientation[i-3] - ende_para[i][0]];
        }
        else{
            diff_para[i] = [0];
        }
    }
    

    var jacob = [];
    var indexJoint = 0;
    var curJoint = endeffector_joint;
    var condition = true;  

    while (condition){
        var temp_axis = matrix_vector_multiply(robot.joints[curJoint].xform,robot.joints[curJoint].axis);
         temp_axis = vector_normalize(temp_axis);
         if (robot.joints[curJoint].type === "prismatic"){
             jacob[indexJoint] = [temp_axis[0], temp_axis[1], temp_axis[2], 0, 0, 0];
         }
         else{
             var jointXYZ_World = matrix_multiply(robot.joints[curJoint].xform, [[0],[0],[0],[1]]);
             var diff_JointXYZ = vector_minus(endeXYZ_World,jointXYZ_World);
             var temp_cross = vector_cross(temp_axis, diff_JointXYZ);
             jacob[indexJoint] = [temp_cross[0], temp_cross[1], temp_cross[2], temp_axis[0], temp_axis[1], temp_axis[2]];
         }
         if (robot.joints[curJoint].parent === robot.base){
             condition = false;
         }
         indexJoint++;
         curJoint = robot.links[robot.joints[curJoint].parent].parent;
     }
     var robot_cont = []; // for control 
     if (!kineval.params.ik_pseudoinverse){
         robot_cont = matrix_multiply(jacob, diff_para);
     }
     else{
         var jacobT = matrix_transpose(jacob);
         robot_cont = matrix_multiply(matrix_pseudoinverse(jacobT), diff_para);
     
     }


     condition = true;
     indexJoint = 0;
     curJoint = endeffector_joint;
     while (condition){
         robot.joints[curJoint].control += kineval.params.ik_steplength*robot_cont[indexJoint][0];
         indexJoint++;
         if (robot.joints[curJoint].parent === robot.base){
             condition = false;
         }
         curJoint = robot.links[robot.joints[curJoint].parent].parent;
     }



}

