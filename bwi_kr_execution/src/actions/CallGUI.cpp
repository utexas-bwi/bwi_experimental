#include "CallGUI.h"

#include "segbot_gui/QuestionDialog.h"

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

  segbot_gui::QuestionDialog req;

  req.request.type = type;
  req.request.message = message;
  req.request.options = options;
  req.request.timeout = timeOut;

  client.call ( req );

  done = true;

}

actasp::Action *CallGUI::cloneAndInit(const actasp::AspFluent & fluent) const {
  throw runtime_error("CallGUI: initilization from fluent not supported");
} 
  
}