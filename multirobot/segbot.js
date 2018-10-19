//TODO: OlegDbg:: to make fancy with callbacks and utility class for logs
let ROSLIB = require('roslib');

//TODO: Enable this back when deploying in the lab
const ROSBRIDGE_PORT = 9090;


class Robot {
	constructor(name,ipaddr,commonTopics,errHandlerCallback) {
		console.log("Created segbot: " + name + "(" + ipaddr + ")");
		//console.log("Created segbot: " + name + "(" + ipaddr + ":" + rosbridgeport + ")");
		this.name = name;
		this.ipaddr = ipaddr;
		//this.rosbridgeport = ROSBRIDGE_PORT;
		//this.mjpegserverport = MJPEGSERVERPORT;
		this.ros = null;
		this.topicsClient = null;
		this.topicsSpecs = []; // TODO: you could probably remove this one, it is only needed for initialization and there is no point on storing it
		this.commonTopics = commonTopics;
		this.errHandler = errHandlerCallback;
		this.verifyCommonTopics = this.verifyCommonTopics.bind(this);
	}

	verifyCommonTopics(result) {
		let availableTopics = result.topics;
		for (let i = 0; i < this.commonTopics.length; i++) { 
			let topicSpec = this.commonTopics[i];
			if (!availableTopics.includes("/"+topicSpec.name)) {
				//callback(new Error("Trying to transmit a non-existing topic '"+topicSpec.name+ "' for robot '"+this.name+"'"));
				this.errHandler(Error("Trying to transmit a non-existing topic '"+topicSpec.name+ "' for robot '"+this.name+"'"));
			}
			//createCommTopic(callback,commTopicSpec,availableTopics);
		}
		
	}
   		
	connect(transmitToWalkyTalky,subscribeToWalkyTalky) {
		console.log("Trying to connect to " + this.ipaddr + " : " + ROSBRIDGE_PORT);
		//console.log("Connecting to '" + this.ipaddr + "'");
		this.ros = new ROSLIB.Ros({
			url : 'ws://' + this.ipaddr + ':' + ROSBRIDGE_PORT
			//url : 'ws://' + this.ipaddr
		});
		
		this.ros.on('connection',() => {
				console.log('Virtour is running on host.\''+this.name + '\'');
				this.topicsClient = new ROSLIB.Service({
					ros : this.ros,
					name : '/rosapi/topics',
					serviceType : 'rosapi/Topics'
				});
			
				//this.verifyCommonTopics = this.verifyCommonTopics.bind(this);
				var request = new ROSLIB.ServiceRequest();
				console.log("verifiy topics for:" + this.name);
				//this.topicsClient.callService(request,this.verifyCommonTopics); 
		
				transmitToWalkyTalky(this);
				subscribeToWalkyTalky(this);
				console.log('Connection made!');
		});
		
		this.ros.on('error', (err) => {
		  console.log('Virtour is not running on host.\''+this.name + '\'');
		  //this.errHandler(err);
		});
	}
	
}

module.exports = Robot;
