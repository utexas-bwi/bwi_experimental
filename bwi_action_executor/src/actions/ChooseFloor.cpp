
#include "ChooseFloor.h"

#include "ActionFactory.h"

#include <bwi_planning_common/PlannerInterface.h>
#include <bwi_kr/ChangeFluent.h>
#include <map_mux/ChangeMap.h>
#include "CallGUI.h"

#include <ros/ros.h>

#include <sstream>
using namespace ros;
using namespace std;

namespace bwi_actexec{

ChooseFloor::ChooseFloor(const std::string& name) :
			name(name),
			parameters(),
			done(false){}

void ChooseFloor::init(const vector< string >& params) {
	this->parameters = params;
}

void ChooseFloor::run(){
	NodeHandle n;
    ros::ServiceClient service_change_map = n.serviceClient<map_mux::ChangeMap> ( "change_map" );
    service_change_map.waitForExistence();

	int floor = -1;

	if( parameters[0] == "f2"){
		floor = 2;
	}
	if( parameters[0] == "f3"){
		floor = 3;
	}
    map_mux::ChangeMap cm;
    cm.request.data = floor;
    service_change_map.call(cm);

	vector< string > options;
    ostringstream convert;
    convert << floor;

    std::string str1 =  "We have arrived on floor "+ convert.str() + ".";
    std::string str2 =  "Please let me know when we have reached floor "+ convert.str();

	options.push_back(str1);
	CallGUI loadGUI("LoadGUI", CallGUI::CHOICE_QUESTION, str2 ,0.0,  options );
	loadGUI.run();

    // DO NOT NEED TO DO AT THIS TIME Pass along "fake" observation to KR node
    //NodeHandle n;
    //std::vector<bwi_planning_common::PlannerAtom> observations;
    //bwi_planning_common::PlannerAtom observation1;
    //observation1.name = "";
    //observation1.value.push_back(parameters[0]);
    //observations.push_back(observation1);
    //ros::ServiceClient krClient = n.serviceClient<bwi_kr::ChangeFluent> ( "/bwi_kr/change_fluent" );
    //krClient.waitForExistence();
    //for(int i=0, size=observations.size() ; i < size; ++i) {
        //bwi_kr::ChangeFluent cf;
        //cf.request.fluent.name = observations[i].name;
        //cf.request.fluent.parameters = observations[i].value;
        //krClient.call(cf);
    //}


	done = true;
}

static ActionFactory chooseFloorFactory(new ChooseFloor("choosefloor"));

}


// Origininal Choosefloor action
//        if action.name == "choosefloor":
//            #call change map service
//            try:
//                service_change_map = rospy.ServiceProxy('change_map', ChangeMap)
//                if action.value.value == "f2":
//                    resp = service_change_map(2)
//                    print "Ran change_map with 2"
//                if action.value.value == "f3":
//                    resp = service_change_map(3)
//                    print "Ran change_map with 3"
//
//                response = self.gui(QuestionDialogRequest.CHOICE_QUESTION,
//                                "Please let me know once floor" + str(action.value.value[1]) + "has been reached",
//                                ["Done!"], 0.0)
//                if response.index == 0: # The Done! button was hit
//                    success = True
//                #time.sleep(15.0)
//            except rospy.ServiceException:
//                rospy.loginfo("serviced failed - change_map")
//            #observations.append(AtomCoffee("closeto",str(action.value),time=next_step))
//
//            success = True
//
//        rospy.loginfo("  Observations: " + str(observations))
//        self.clear_gui()
//        return success, observations
