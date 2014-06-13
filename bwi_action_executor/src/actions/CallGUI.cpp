#include "CallGUI.h"

#include "ActionFactory.h"

#include <ros/ros.h>

#include <segbot_gui/QuestionDialog.h>

using namespace segbot_gui;

namespace bwi_actexec {

CallGUI::CallGUI ( const std::string &name, const TYPE type,  const std::string& message,
                   float timeOut,
                   const std::vector<std::string> &options ) :
	name ( name ),
	type ( type ),
	message ( message ),
	timeOut ( timeOut ),
	options ( options ),
	done ( false ) {}

void CallGUI::run() {

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<segbot_gui::QuestionDialog> ( "question_dialog" );

	segbot_gui::QuestionDialog req;

	req.request.type = type;
	req.request.message = message;
	req.request.options = options;
	req.request.timeout = timeOut;

	client.call ( req );

	done = true;

}

bool CallGUI::hasFinished() const {
	return done;
}


//static ActionFactory hello2 ( new CallGUI ( "gothrough",CallGUI::DISPLAY,"gothrough" ) );
//static ActionFactory hello3 ( new CallGUI ( "opendoor",CallGUI::DISPLAY,"opendoor" ) );
//static ActionFactory hello4 ( new CallGUI ( "getin",CallGUI::DISPLAY,"getin" ) );

}





