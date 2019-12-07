
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
   - Add cubic spline interpolation
   - Add hook for random configuration
   - Display planning iteration number in UI
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }else if (kineval.params.update_motion_plan_traversal||        kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;//a number
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else//kineval.params.update_motion_plan_traversal == true
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }
    }
}


    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration


    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;
    kineval.motion_plan_traversal_index = 0;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();

    // init tree
    T_a = tree_init(q_start_config);
    T_b = tree_init(q_goal_config);
    ver_step = 0.4; // init ver_step , the step distance
}


function robot_rrt_planner_iterate() {

    // var i;
    // rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED), 2: rrt_star
     rrt_alg = 1;
    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

        // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
        //   is used instead of a for loop to avoid blocking and non-responsiveness 
        //   in the browser.
        //
        //   once plan is found, highlight vertices of found path by:
        //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
        //
        //   provided support functions:
        //
        //   kineval.poseIsCollision - returns if a configuration is in collision
        //   tree_init - creates a tree of configurations
        //   tree_add_vertex - adds and displays new configuration vertex for a tree
        //   tree_add_edge - adds and displays new tree edge between configurations
        if (typeof rrt_alg === "undefined"){
            rrt_alg = 1;
        }


        
        if (rrt_alg === 1 ){

            var q_rand = random_config();
            var q_near_id = nearest_neighbor(q_rand, T_a);
            var q_new_rand = new_config(q_rand, T_a,q_near_id);
            var g_extend = rrt_extend(T_a, q_rand, q_new_rand, q_near_id);
            if (g_extend !== "Trapped"){
                var temp_connect = rrt_connect(T_a, T_b);
                if( temp_connect === "Reached" ){
                    console.log("reached");
                    rrt_iterate = false;
                    kineval.motion_plan = path_dfs(T_a,T_b);
                    return "reached";
                }
            }
            var T_temp = T_a;
            T_a = T_b;
            T_b = T_temp;
            rrt_iter_count += 1;
            return "extended";
        }else if (rrt_alg === 2){
            q_rand = random_config();
            var q_near_id = nearest_neighbor(q_rand, T_a);
            var q_new_rand = new_config(q_rand, T_a,q_near_id);
            g_extend = rrt_star_extend(T_a, q_rand, q_new_rand, q_near_id);
            if (q_new_rand){
                var distance_goal = 0;
                for (let j = 0; j< q_goal_config.length; j++){
                    distance_goal = distance_goal + Math.pow(q_new_rand[j] - q_goal_config[j],2);
                }
                distance_goal = Math.sqrt(distance_goal);
                if (Math.floor(distance_goal/ver_step) == 0 ){
                    rrt_iterate = false;
                    kineval.motion_plan = path_dfs_star(T_a);
                    return "reached";
                }
            }
            rrt_iter_count += 1;
            return "extended";
        }

    }
}



//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree,q) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = 0;//suppose to be 0，vertex.vertex[1]
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////


    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
    //   rrt_connect
    //   random_config
    //   new_config
    //   nearest_neighbor
    //   normalize_joint_state
    //   find_path
    ///   path_dfs


function random_config(){
    var i;
    var q_rand=[];
    for (i=0;i<18;i++){
        q_rand[i]=0;
    }
    q_rand[0] = robot_boundary[0][0]+Math.random()*(robot_boundary[1][0] - robot_boundary[0][0]);
    q_rand[1] = robot.origin.xyz[1];
    q_rand[2] = robot_boundary[0][2]+Math.random()*(robot_boundary[1][2] - robot_boundary[0][2]);
    q_rand[4] = 2*(Math.random()-0.5)*Math.PI;
    for (var x in robot.joints){
        if (robot.joints[x].limit === undefined){
            q_rand[q_names[x]] = 2*(Math.random()-0.5)*Math.PI;
        }else if (robot.joints[x].limit !== undefined){
            q_rand[q_names[x]] = robot.joints[x].limit.lower+Math.random()*(robot.joints[x].limit.upper-robot.joints[x].limit.lower);
        }        
        if (robot.joints[x].type === "fixed"){
            q_rand[q_names[x]] = 0;
        }
    }
    return q_rand;
}    

function new_config(q_head, T_temp, q_near_index){
    var q_near = T_temp.vertices[q_near_index].vertex;
    var q_new = [];
    var distance_head = 0;
    for (let j = 0; j< q_head.length; j++){
        distance_head = distance_head + Math.pow(q_head[j] - q_near[j],2);
    }
    distance_head = Math.sqrt(distance_head);
    for (let j = 0; j< q_head.length; j++){
        q_new[j] = (q_head[j]-q_near[j])/distance_head * ver_step + q_near[j];
    }
    var temp_bul = true;
    if (robot.links_geom_imported){
        for (x in robot.joints) {
            if ((robot.joints[x].type === "revolute") || (robot.joints[x].type === "prismatic")){
                if ( q_new[q_names[x]] > robot.joints[x].limit.upper  || q_new[q_names[x]] < robot.joints[x].limit.lower){
                    temp_bul = false;
                    break;
                }
            }
        }
    }
    var collision = kineval.poseIsCollision(q_new);
    if ( collision !== false || temp_bul === false){
        return false;
    }else{
        return q_new;
    }
}


function nearest_neighbor(q, T){
    var nearest_distance = 1e5;
    for(var i = 0; i< T.vertices.length; i++){
        var distance_temp = 0;
        for (let j = 0; j< q.length; j++){
            distance_temp = distance_temp + Math.pow(T.vertices[i].vertex[j] - q[j],2);
        }
        if (distance_temp < nearest_distance){
            var q_near_index = i;
            nearest_distance = distance_temp;
        }
    }
    return q_near_index;
}




function path_dfs(T_1,T_2){
    var A_path = [];
    var Ttemp = T_1.vertices[T_1.newest];
    A_path.push(Ttemp);
    while (T_1.vertices.indexOf(Ttemp) !== 0){
        Ttemp = Ttemp.edges[0]; //children 
        A_path.push(Ttemp);
    }

    var B_path = [];
    var Ttemp_b = T_2.vertices[T_2.newest];
    B_path.push(Ttemp_b);
    while (T_2.vertices.indexOf(Ttemp_b) !== 0){
        Ttemp_b = Ttemp_b.edges[0];
        B_path.push(Ttemp_b);
    }

    var temp_bul = true;
    for (j=0; j< q_start_config.length; j++){
        temp_bul = temp_bul &&  (A_path[A_path.length-1].vertex[j] === q_start_config[j]);
    }
    var path_result = [];
    if (temp_bul === true){
        for (k=0; k< A_path.length; k++){
            path_result.unshift(A_path[k]);
        }        
        for (j=0; j< B_path.length; j++){
            path_result.push(B_path[j])
        }
    }else{
        for (k=0; k< B_path.length; k++){
            path_result.unshift(B_path[k]);
        }        
        for (j=0; j< A_path.length; j++){
            path_result.push(A_path[j])
        }
    }
    for (let i=0; i< path_result.length; i++){
        path_result[i].geom.material.color = {r:1,g:0,b:0};
    }
    return path_result;
}

function rrt_extend(T_extend, q_head, q_new, q_near_index){
    if (q_new !== false){
        tree_add_vertex(T_extend,q_new);
        tree_add_edge(T_extend,q_near_index,T_extend.newest);
        add_config_origin_indicator_geom(T_extend.vertices[T_extend.newest]);
        var distance_head = 0;
        for (let j = 0; j< q_head.length; j++){
            distance_head = distance_head + Math.pow(q_new[j] - q_head[j],2);
        }
        distance_head = Math.sqrt(distance_head);
        if (Math.floor(distance_head /ver_step) === 0 ){
            return "Reached"; 
        }else{
            return "Advanced"; 
        }
    }else{
        return "Trapped";
    }
}

function rrt_connect(T_1, T_2){
    var q_connect = T_1.vertices[T_1.newest].vertex;
    var g_connect = "Advanced";
    while (g_connect === "Advanced") {
        var q_near_2_id = nearest_neighbor(q_connect, T_2);
        var q_new2 = new_config(q_connect, T_2,q_near_2_id);
        g_connect = rrt_extend(T_2, q_connect, q_new2, q_near_2_id);
    }
    return g_connect;
}




function nearest_star_neighbor(T,q){
    var neighborList = []
    for (var i=0; i<T.vertices.length - 1; i++){
        if (1.1*step_inter>path_dfs(q, T.vertices[i].vertex)){
            neighborList.push([T.vertices[i], i])
        }
    }
    return neighborList;
}


function rrt_star_extend(T_extend, star_head, star_new, star_nearest_index){
    if (star_new){
        var star_near_index = get_near_index(T_extend, star_new, T_extend.vertices.length);
        var star_min_index = get_parent(T_extend, star_near_index, star_nearest_index, star_new);
        tree_add_vertex(T_extend,star_new);


        tree_add_edge(T_extend,star_min_index,T_extend.newest);
        add_config_origin_indicator_geom(T_extend.vertices[T_extend.newest]);

        var distance_head = 0;
        for (let j = 0; j< q_goal_config.length; j++){
            distance_head = distance_head + Math.pow(star_new[j] - star_head[j],2);
        }
        distance_head = Math.sqrt(distance_head);
        if (Math.floor(distance_head /ver_step) == 0 ){
            return "Reached";
        }else{
            return "Advanced";
        }
    }
    return "Trapped";
}

function path_dfs_star(T){
    var A_path = [];
    var Ttemp = T.vertices[T.newest];
    A_path.unshift(Ttemp);
    while (T.vertices.indexOf(Ttemp) !== 0){
        Ttemp = Ttemp.edges[0];
        A_path.unshift(Ttemp);
    }
    for (let i=0; i< A_path.length; i++){
        A_path[i].geom.material.color = {r:1,g:0,b:0};
    }
    return A_path;
}


// func by myself for convience

function get_parent(tree, star_near_index, star_nearest_index, star_new){
    var cross_min = tree.vertices[star_nearest_index].cost + ver_step;// 
    var star_min_index = star_nearest_index;
    for (let i = 0; i< star_near_index.length; i++){
        if (star_nearest_index == star_near_index[i]){
            break;
        }
        var temp_ste = true;
        if (temp_ste){
            var distance_new = 0;
            for (let j = 0; j< star_new.length; j++){
                distance_new = distance_new + Math.pow(tree.vertices[star_near_index[i]].vertex[j] - star_new[j],2);
            }
            distance_new = Math.sqrt(distance_new);
            var cross_get = tree.vertices[star_near_index[i]].cost + distance_new;
            if (cross_get < cross_min){
                cross_min = cross_get;
                star_min_index =  star_near_index[i];
            }
        }
    }
    
    return star_min_index;
}


function get_near_index(T_extend,q_new,q_range){
    var temp = q_new.length - 3;
    var gamm = 1.1/Math.pow(0.3662,temp) *ver_step;
    var ran_temp = gamm * Math.pow(Math.log(q_range)/q_range, temp);
    if (ran_temp < ver_step){
        ran_temp = ver_step;
    }
    var star_near_index = [];
    for (let i = 0; i< T_extend.vertices.length; i++){
        var dist = 0;
        for (let j = 0; j< q_new.length; j++){
            dist = dist + Math.pow(T_extend.vertices[i].vertex[j] - q_new[j],2);
        }
        dist = Math.sqrt(dist);
        if (dist < ran_temp){
            star_near_index.push(i);
        }
    }
    return star_near_index;
}




