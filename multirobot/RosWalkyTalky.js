
const ROSLIB = require('roslib');
const request = require("request");
const Robot = require('./segbot.js');
const SERVER = "http://nixons-head.csres.utexas.edu:7979/hostsalivejson";
//const SERVER = "http://localhost:3000/hostsalive";

let transmissionFunctions = {};
let robots = {};
commonTopics = [{
		name: '/move_base/status',
		messageType: 'actionlib_msgs/GoalStatusArray'
		
	}
	/*{	
		name: 'topic2',
		messageType: 'std_msgs/String'
	}*/
];

commonTopicsForRobot = {
		'bender': [{
				name: '/test_harel',
				messageType: 'std_msgs/String'
			}
			
		],
		'roberto': [{
				name: '/test_harel',
				messageType: 'std_msgs/String'
			}
			
		]

};


errHandler = (err) => {
	console.log("ERROR:" + err);
	throw err;
}

function createTransmissionFunction(topicName,messageType,receiverRobot,transmitterRobot) {
	let walkyTalkyTopic  = new ROSLIB.Topic({
			ros : receiverRobot.ros,
			name : '/'+ transmitterRobot.name +'_walkyTalky_'+topicName,
			messageType : messageType
	});
	transmissionFunctions[receiverRobot.name][transmitterRobot.name][topicName] = function (msg) {
			//console.log("walkyTalkyAdaptor:" + );
	   		walkyTalkyTopic.publish(msg);
	};
}

function transmitToWalkyTalky(newRobot) {
	console.log("Transmitting from robot:" + newRobot.name);
	for(let robotName in robots) {
		let receiverRobot = robots[robotName];
	//TODO: make sure that curr robot is not included in this.robots
		if (receiverRobot.name === newRobot.name) {
			continue;	
		} 
		if (!(receiverRobot.name in transmissionFunctions)) {
					console.log("initializing receiver" + receiverRobot.name);
					transmissionFunctions[receiverRobot.name] = {};
		}
			
		for(let j=0; j < newRobot.commonTopics.length; j++) {
				let commonTopic = newRobot.commonTopics[j];
				console.log("name: "+ commonTopic.name);
				if (!(newRobot.name in transmissionFunctions[receiverRobot.name])) {
					transmissionFunctions[receiverRobot.name][newRobot.name] = {};
				}
	
				if (!(commonTopic.name in transmissionFunctions[receiverRobot.name][newRobot.name])) {
					// If there is a topic on subscriber robot already called robot.name+"walkyTalky"+topicName - You are hosed!
					createTransmissionFunction(commonTopic.name,commonTopic.messageType,receiverRobot,newRobot);
					console.log("name: "+ commonTopic.name);
					let commonTopicImpl  = new ROSLIB.Topic({
							ros : newRobot.ros,
							name : '/'+commonTopic.name,
							messageType : commonTopic.messageType
					});
					commonTopicImpl.subscribe(transmissionFunctions[receiverRobot.name][newRobot.name][commonTopic.name]);
				}
		}
	}
}

function subscribeToWalkyTalky(newRobot) {
		console.log("Transmitting from robot:" + newRobot.name);
		if (!(newRobot.name in transmissionFunctions)) {
			transmissionFunctions[newRobot.name] = {};
		}
		
		for(let robotName in robots) {
			let transmitterRobot = robots[robotName];
			if (transmitterRobot.name === newRobot.name) {
				continue;	
			} 
			
			for(let j=0; j < transmitterRobot.commonTopics.length; j++) {
				let commonTopic = transmitterRobot.commonTopics[j];
				if (!(transmitterRobot.name in transmissionFunctions[newRobot.name])) {
					transmissionFunctions[newRobot.name][transmitterRobot.name] = {};
				}
				if (!(commonTopic.name in transmissionFunctions[newRobot.name][transmitterRobot.name])) {
					createTransmissionFunction(commonTopic.name,commonTopic.messageType,newRobot,transmitterRobot);
					let commonTopicImpl  = new ROSLIB.Topic({
							ros : transmitterRobot.ros,
							name : '/'+commonTopic.name,
							messageType : commonTopic.messageType
					});
					commonTopicImpl.subscribe(transmissionFunctions[newRobot.name][transmitterRobot.name][commonTopic.name]);
				}
			}
		}
}
	
function unsubscribeFromWalkyTalky(deadRobot) {
		for (let transmitterName in transmissionFunctions[deadRobot.name]) {
			let transmitterRobotTopics = transmissionFunctions[deadRobot.name][transmitterName];
			//TODO: implement
			let transmitterRobot = robots[transmitterName];
			for (let i=0; i < transmitterRobot.commonTopics.length; i++) {
				commonTopic = transmitterRobot.commonTopics[i];
				//TODO:: think about robot.commonTopics be map (object) of ROSLIB.Topics mapped by name
				let commonTopicImpl  = new ROSLIB.Topic({
							ros : transmitterRobot.ros,
							name : '/'+commonTopic.name,
							messageType : commonTopic.messageType
				});
				commonTopicImpl.unsubscribe(transmissionFunctions[deadRobot.name][transmitterName][commTopic.name]);
			}	
		}
		
		delete transmissionFunctions[deadRobot.name];
		
		for (let receiverName in transmissionFunctions) {
			delete transmissionFunctions[receiverName][deadRobot.name];
		}
}

function test() {
	request(SERVER, {json: true}, (error, response, body) => {
  		if (error) {
  			errHandler(error);
  		}

  		//let data = JSON.parse(body);
  		let data = body;
  		for (let name in data) {
			console.log("found robot '"+ name +"' online");
  			let ipaddr = data[name];
  		}
  		if ('R8081' in data) {
			console.log("good!");
		} else {
			console.log("bad!");
		}
	});

}

function refreshRobots () {
		console.log("Refreshing robots");
		if (SERVER === "") {
			error("Will not be able to dynamically load robot's IP addresses","Error: No DNS server set");
			return;
		}
		
		if (commonTopics.length === 0) {
			error("Communication topics for Walky Talky have not been set");
		}

		console.log("Pinging dns server...");
		request(SERVER, {json: true}, (error, response, body) => {
			if (error) {
  				errHandler(Error("Failed to ping DNS server on" + SERVER));
  			}

			let data = body;
			let robotNumber = Object.keys(data).length;
			for (let name in data) {	//this.availableRobots[name] = 1;
				let ipaddr = data[name]; 
				console.log("found robot '"+ name +"' online");
			    if (name in robots && robots[name].ipaddr !== ipaddr) {
					console.log("Robot '"+ name +"' changed ip!");
					unsubscribeFromWalkyTalky(this.robots.get(name));
					delete robotos[name];
					delete this.topics[name];
					connectRobot(name,ipaddr);
				} else if (!(name in robots)) {
					console.log("Refresher connecting")
					connectRobot(name,ipaddr);
				}
			};
			
			for(let robotName in robots) {
				let robot = robots[robotName];
				if (!(robotName in data)) {
					console.log("Robot '" + robotName + "'disconnected, stopping communication.")
				    unsubscribeFromWalkyTalky(robot);
					delete robots[robot.name];
				}
			}
			
			if (Object.keys(data).length === 0) {
				console.log("No robots available at this time");
			}
    	});
}	

function connectRobot(name,ipaddr) {
	let myCommonTopics;
	if (name in commonTopicsForRobot) {
		myCommonTopics = commonTopicsForRobot[name];
	} else {
		myCommonTopics = commonTopics;
	}
	let robot = new Robot(name,ipaddr,myCommonTopics,errHandler);
	if (name in robots) {
		errHandler(Error("robot '" + name + "' is already connected. Found a Duplicate robot on network"));
	}
	robots[name] = robot;
	robot.connect(transmitToWalkyTalky,subscribeToWalkyTalky);

}

setInterval(refreshRobots,2000);

