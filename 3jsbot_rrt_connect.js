//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// CS148: 
// implement RRT-Connect by Kuffner and LaValle (2000)
//    paper link: http://msl.cs.uiuc.edu/~lavalle/papers/KufLav00.pdf

// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by robot_collision_test()

/*
CS148: reference code has functions for:

    tree_add_vertex
    tree_add_edge
    random_config
    new_config
    nearest_neighbor
    rrt_extend
    rrt_connect
    find_path
    path_dfs
*/


function robot_rrt_planner_init() {

    robot_path_traverse_idx = 0;

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];
    //console.log(q_start_config);

    q_names = {};  // store mapping between joint names and q DOFs


    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    //console.log(q_start_config);
    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // CS148: add necessary RRT initialization here

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();

    EPSILON = .5;
    if (typeof Ta !== 'undefined'){
        for (var i = 0; i <= Ta.newest; i++){
            scene.remove(Ta.vertices[i].geom);
        }
    }
    if (typeof Tb !== 'undefined'){
        for (var i = 0; i <= Tb.newest; i++){
            scene.remove(Tb.vertices[i].geom);
        }
    }
    Ta = tree_init(q_start_config);
    Tb = tree_init(q_goal_config);
    Ta.vertices[Ta.newest].geom.material.color = {r:0,g:0,b:1};
    Tb.vertices[Tb.newest].geom.material.color = {r:0,g:1,b:0};
    q_init = q_start_config;
    q_goal = q_goal_config;

    rrt_iterate = true;
    total_vertices = 0;

    //console.log("planner initialized");
}

// function draw_highlighted_path(robot_path) {
//     for (i=0;i<robot_path.length;i++) {
//         robot_path[i].geom.material.color = {r:1,g:0,b:0};
//     }
// }

function robot_rrt_planner_iterate() {


    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        robot_path = robot_rrt_planner_iterate_loop(q_init, q_goal);
        if (robot_path.length !== 0) {

            rrt_iterate = 0;
            cur_time = Date.now();
            return "Reached";


        }
        cur_time = Date.now();
   }


    // return path not currently found
    return "Advanced";
}

function tree_add_vertex (T, q) {
    T.newest = T.newest + 1;
    T.vertices[T.newest] = {};
    T.vertices[T.newest].vertex = q;
    T.vertices[T.newest].edges = [];
    add_config_origin_indicator_geom(T.vertices[T.newest]);
    total_vertices += 1;
}


function tree_add_edge (T, child_index, parent_index) {
    T.vertices[child_index].parent = T.vertices[parent_index];
}

function random_config () {
    var q_rand = [Math.random()*(robot_boundary[1][0]-(robot_boundary[0][0]))+(robot_boundary[0][0]),
                  0, 
                  Math.random()*(robot_boundary[1][2]-(robot_boundary[0][2]))+(robot_boundary[0][2]), 
                  0,
                  Math.random()*(6.28318531),
                  0];
    for (var i = 6; i < q_init.length; i++) {
        q_rand.push(0);
    }
    return q_rand;
}

function nearest_neighbor (q, T) {
    var curr_nearest = T.vertices[0].vertex;
    var curr_dist = Infinity;
    for (var i = 0; i <= T.newest; i++) {
        var dist = distance(T.vertices[i].vertex, q);
        if (dist < curr_dist){
            curr_nearest = T.vertices[i].vertex;
            curr_dist = dist;
        }
    }
    return curr_nearest;
}

function nearest_neighbor_index (q, T) {
    var curr_nearest = 0;
    var curr_dist = Infinity;
    for (var i = 0; i <= T.newest; i++) {
        var dist = distance(T.vertices[i].vertex, q);
        if (dist < curr_dist){
            curr_nearest = i;
            curr_dist = dist;
        }
    }
    return curr_nearest;
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

function find_path (Ta, Tb) {

    var path = [];

    //Ta
    var current_node = TaEnd;
    current_node.geom.material.color = {r:1,g:0,b:0};
    if (current_node !== undefined) {
        path.unshift(current_node);
    }
    while (current_node.parent !== undefined) {
        current_node = current_node.parent;
        current_node.geom.material.color = {r:1,g:0,b:0};
        path.unshift(current_node);
    }

    //Tb
    current_node = TbEnd;
    current_node.geom.material.color = {r:1,g:0,b:0};
    if (current_node !== undefined) {
        path.push(current_node);
    }
    while (current_node.parent !== undefined) {
        current_node = current_node.parent;
        current_node.geom.material.color = {r:1,g:0,b:0};
        path.push(current_node);
    }

    return path;
}

function new_config (q, q_near) {
    var d1 = [];
    for (var i = 0; i < q.length; i++){
        d1.push(q[i] - q_near[i]);
    }
    d1 = vector_normalize(d1);
    var q_new = [];
    for (var i = 0; i < q.length; i++){
        q_new.push(q_near[i] + EPSILON*(d1[i]));
    }
    return q_new;
}

function distance (A, B){
    var total = 0;
    for (var i = 0; i < A.length; i++) {
            total += Math.pow((A[i] - B[i]), 2);
    }
    total = Math.sqrt(total);
    return total;
}
function rrt_extend (T, q) {
    //something in Tb
    var neighbor_index = nearest_neighbor_index(q, T);
    var q_near = T.vertices[neighbor_index].vertex;

    //also in Tb, but if it == q then already in Ta
    var q_new = new_config(q, q_near);

    TbEnd = Tb.vertices[neighbor_index];
    TaEnd = Ta.vertices[Ta.newest];
    if (!robot_collision_test(q_new)){
        tree_add_vertex(T, q_new);
        tree_add_edge(T, T.newest, neighbor_index);
        if (distance(q_new, q) < EPSILON) {
            //alert("Added to tree and Reached");
            return "Reached";
        } else {
            //alert("Added to tree and Advanced");
            return "Advanced";
        }
    }
    return "Trapped";
}

function rrt_connect (T, q) {
    var ret = "Advanced";
    while (ret === "Advanced") {
        ret = rrt_extend(T, q);
    }
    return ret;
}

function robot_rrt_planner_iterate_loop(q_init, q_goal) {
    //alert("Loop");
    qrand = random_config();
    // console.log("QRand:");
    // console.log(qrand);
    if (rrt_extend(Ta, qrand) !== "Trapped"){
        //alert("Trying to connect!");
        if (rrt_connect(Tb, Ta.vertices[Ta.newest].vertex) === "Reached") {
            //alert("total_vertices: " + total_vertices + "\nTa verts: " + (Ta.newest+1) + "\nTb Verts: " + (Tb.newest+1));
            return find_path(Ta, Tb);
        }
        //alert("Done Connecting.");
    }
    temp = Ta;
    Ta = Tb;
    Tb = temp;
    //alert("Swapping.");
    return [];
}

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


function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);

    vertex.geom = temp_mesh;
}





