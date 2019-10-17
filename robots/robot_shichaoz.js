//   CREATE ROBOT STRUCTURE

// KE 

links_geom_imported = false;

//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

// create robot data object
robot = new Object(); // or just {} will create new object

// give the robot a name
robot.name = "shichaoz";

// initialize start pose of robot in the world
robot.origin = {xyz: [0,0,3], rpy:[0,0,0]};  

// specify base link of the robot; robot.origin is transform of world to the robot base
robot.base = "base";  

        
// specify and create data objects for the links of the robot
robot.links = {
    "base": {},
    "palm": {},  
    "finger1_upper": {}, 
    "finger1_middle": {}, 
    "finger1_lower": {}, 
    "finger2_upper": {}, 
    "finger2_middle": {}, 
    "finger2_lower": {}, 
    "finger3_upper": {}, 
    "finger3_middle": {}, 
    "finger3_lower": {}, 
    "finger4_upper": {}, 
    "finger4_middle": {}, 
    "finger4_lower": {}, 
    "finger5_upper": {}, 
    "finger5_middle": {}, 
    "finger5_lower": {}
};


robot.joints = {};

robot.joints.finger1_joint1 = {parent:"palm", child:"finger1_upper"};
robot.joints.finger1_joint1.origin = {xyz: [-0.4,0.0,0.8], rpy:[0,Math.PI/4,0]};
robot.joints.finger1_joint1.axis = [0.0,1.0,0.0]; 

robot.joints.finger1_joint2 = {parent:"finger1_upper", child:"finger1_middle"};
robot.joints.finger1_joint2.origin = {xyz: [0.0,0.0,0.4], rpy:[-Math.PI/4,0,0]};
robot.joints.finger1_joint2.axis = [1.0,0.0,0.0]; 

robot.joints.finger1_joint3 = {parent:"finger1_middle", child:"finger1_lower"};
robot.joints.finger1_joint3.origin = {xyz: [0.0,0.0,0.5], rpy:[Math.PI/2,0,0,]};
robot.joints.finger1_joint3.axis = [1.0,0.0,1.0]; 

robot.joints.finger2_joint1 = {parent:"palm", child:"finger2_upper"};
robot.joints.finger2_joint1.origin = {xyz: [0.3,0.0,0.8], rpy:[0,Math.PI/2,0]};
robot.joints.finger2_joint1.axis = [0.0,1.0,0.0]; 

robot.joints.finger2_joint2 = {parent:"finger2_upper", child:"finger2_middle"};
robot.joints.finger2_joint2.origin = {xyz: [0.0,0.0,0.4], rpy:[-Math.PI/4,0,0]};
robot.joints.finger2_joint2.axis = [1.0,0.0,0.0]; 

robot.joints.finger2_joint3 = {parent:"finger2_middle", child:"finger2_lower"};
robot.joints.finger2_joint3.origin = {xyz: [0.0,0.0,0.55], rpy:[Math.PI/2,0,0]};
robot.joints.finger2_joint3.axis = [1.0,0.0,0.0]; 

robot.joints.finger3_joint1 = {parent:"palm", child:"finger3_upper"};
robot.joints.finger3_joint1.origin = {xyz: [0.3,0.0,0.3], rpy:[0,Math.PI/2,0]};
robot.joints.finger3_joint1.axis = [0.0,1.0,0.0]; 

robot.joints.finger3_joint2 = {parent:"finger3_upper", child:"finger3_middle"};
robot.joints.finger3_joint2.origin = {xyz: [0.0,0.0,0.4], rpy:[-Math.PI/4,0,0]};
robot.joints.finger3_joint2.axis = [1.0,0.0,0.0]; 

robot.joints.finger3_joint3 = {parent:"finger3_middle", child:"finger3_lower"};
robot.joints.finger3_joint3.origin = {xyz: [0.0,0.0,0.6], rpy:[Math.PI/2,0,0]};
robot.joints.finger3_joint3.axis = [1.0,0.0,0.0]; 

robot.joints.finger4_joint1 = {parent:"palm", child:"finger4_upper"};
robot.joints.finger4_joint1.origin = {xyz: [0.3,0.0,-0.3], rpy:[0,Math.PI/2,0]};
robot.joints.finger4_joint1.axis = [0.0,1.0,0.0]; 

robot.joints.finger4_joint2 = {parent:"finger4_upper", child:"finger4_middle"};
robot.joints.finger4_joint2.origin = {xyz: [0.0,0.0,0.4], rpy:[-Math.PI/4,0,0]};
robot.joints.finger4_joint2.axis = [1.0,0.0,0.0]; 

robot.joints.finger4_joint3 = {parent:"finger4_middle", child:"finger4_lower"};
robot.joints.finger4_joint3.origin = {xyz: [0.0,0.0,0.55], rpy:[Math.PI/2,0,0]};
robot.joints.finger4_joint3.axis = [1.0,0.0,0.0]; 

robot.joints.finger5_joint1 = {parent:"palm", child:"finger5_upper"};
robot.joints.finger5_joint1.origin = {xyz: [0.3,0.0,-0.8], rpy:[0,Math.PI/2,0]};
robot.joints.finger5_joint1.axis = [0.0,1.0,0.0]; 

robot.joints.finger5_joint2 = {parent:"finger5_upper", child:"finger5_middle"};
robot.joints.finger5_joint2.origin = {xyz: [0.0,0.0,0.4], rpy:[-Math.PI/4,0,0]};
robot.joints.finger5_joint2.axis = [1.0,0.0,0.0]; 

robot.joints.finger5_joint3 = {parent:"finger5_middle", child:"finger5_lower"};
robot.joints.finger5_joint3.origin = {xyz: [0.0,0.0,0.6], rpy:[Math.PI/2,0,0]};
robot.joints.finger5_joint3.axis = [1.0,0.0,0.0]; 



robot.joints.hand = {parent:"base", child:"palm"};
robot.joints.hand.origin = {xyz: [0.5,0.0,0], rpy:[0,0,0]};
robot.joints.hand.axis = [1.0,0.0,0.0]; 


// specify name of endeffector frame
robot.endeffector = {};
robot.endeffector.frame = "finger3_joint3";
robot.endeffector.position = [[0.2],[0],[0.2],[1]]

//////////////////////////////////////////////////
/////     DEFINE LINK threejs GEOMETRIES
//////////////////////////////////////////////////


links_geom = {};

links_geom["base"] = new THREE.CubeGeometry( 1, 0.4, 3.2 );
links_geom["base"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );


links_geom["palm"] = new THREE.CubeGeometry( 1, 0.4, 2.3 );
links_geom["palm"].applyMatrix( new THREE.Matrix4().makeTranslation(0.5, 0, 0) );

links_geom["finger1_upper"] = new THREE.CubeGeometry( 0.3, 0.3, 0.3 );
links_geom["finger1_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.15) );

links_geom["finger1_middle"] = new THREE.CubeGeometry( 0.3, 0.3, 0.6 );
links_geom["finger1_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.3) );

links_geom["finger1_lower"] = new THREE.CubeGeometry( 0.3, 0.3, 1.0 );
links_geom["finger1_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["finger2_upper"] = new THREE.CubeGeometry( 0.3, 0.3, 0.3 );
links_geom["finger2_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.15) );

links_geom["finger2_middle"] = new THREE.CubeGeometry( 0.3, 0.3, 0.6 );
links_geom["finger2_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.3) );

links_geom["finger2_lower"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["finger2_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["finger3_upper"] = new THREE.CubeGeometry( 0.3, 0.3, 0.3 );
links_geom["finger3_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.15) );

links_geom["finger3_middle"] = new THREE.CubeGeometry( 0.3, 0.3, 0.6 );
links_geom["finger3_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.3) );

links_geom["finger3_lower"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["finger3_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["finger4_upper"] = new THREE.CubeGeometry( 0.3, 0.3, 0.3 );
links_geom["finger4_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.15) );

links_geom["finger4_middle"] = new THREE.CubeGeometry( 0.3, 0.3, 0.6 );
links_geom["finger4_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.3) );

links_geom["finger4_lower"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["finger4_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["finger5_upper"] = new THREE.CubeGeometry( 0.3, 0.3, 0.3 );
links_geom["finger5_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.15) );

links_geom["finger5_middle"] = new THREE.CubeGeometry( 0.3, 0.3, 0.6 );
links_geom["finger5_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.3) );

links_geom["finger5_lower"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["finger5_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );


