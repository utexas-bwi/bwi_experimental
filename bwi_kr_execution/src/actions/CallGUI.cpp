#include "CallGUI.h"

#include <ros/ros.h>

#include <stdexcept>

using namespace std;

namespace bwi_krexec {
  
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
  client.waitForExistence();

  req.request.type = type;
  req.request.message = message;
  req.request.options = options;
  req.request.timeout = timeOut;

  client.waitForExistence();
  client.call ( req );

  done = true;

}

string CallGUI::getResponse() {
  if ((done) && (req.response.index == segbot_gui::QuestionDialogRequest::TEXT_RESPONSE)) {
    return req.response.text;
  }
  else {
    return "";
  }
}

int CallGUI::getResponseIndex() {
  if (done) {
    return req.response.index;
  }
  else {
    return segbot_gui::QuestionDialogRequest::NO_RESPONSE;
  }
}

actasp::Action *CallGUI::cloneAndInit(const actasp::AspFluent & fluent) const {
  throw runtime_error("CallGUI: initilization from fluent not supported");
} 
  
}