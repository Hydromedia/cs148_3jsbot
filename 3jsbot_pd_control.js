//////////////////////////////////////////////////
/////     MOTION CONTROL ROUTINES 
//////////////////////////////////////////////////

// CS148: add PD controller here
function robot_pd_control () {
	for (x in robot.joints) {
		var curdate = new Date();
		robot.joints[x].desired = curdate.getSeconds()/60*2*Math.PI;
		robot.joints[x].control = .2*(robot.joints[x].desired-robot.joints[x].angle);
	}
}