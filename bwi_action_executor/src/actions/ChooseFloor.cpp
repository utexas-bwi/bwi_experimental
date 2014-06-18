
#include "ChooseFloor.h"

#include "ActionFactory.h"

#include <bwi_planning_common/PlannerInterface.h>
#include <bwi_kr/ChangeFluent.h>
#include <map_mux/ChangeMap.h>
#include "LogicalNavigation.h"
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
    std::string door ;

	if( parameters[0] == "f2"){
		floor = 2;
        door = "d2_ele1";
	}
	if( parameters[0] == "f3"){
		floor = 3;
        door = "d3_ele1";
	}
    std::cerr << "we are at the start" + floor << std::endl;
    map_mux::ChangeMap cm;
    cm.request.data = floor;
    service_change_map.call(cm);

    std::cerr << "map service has been called" << std::endl;
	vector< string > options;
    ostringstream convert;
    convert << floor;

    std::string str1 =  "We have arrived on floor "+ convert.str() + ".";
    std::string str2 =  "Please let me know when we have reached floor "+ convert.str();

	options.push_back(str1);
	CallGUI loadGUI("LoadGUI", CallGUI::DISPLAY, str2 , 0.0,  options );
	loadGUI.run();

    //ros::ServiceClient krClient = n.serviceClient<bwi_kr::ChangeFluent> ( "/bwi_kr/change_fluent" );
    //bwi_kr::ChangeFluent cf;
	//cf.request.fluent.name ="at";
	//cf.request.fluent.parameters.push_back("f2_ele1");
    //krClient.call(cf);
    //cf.request.fluent.parameters.clear();
	//cf.request.fluent.name ="beside";
	//cf.request.fluent.parameters.push_back(door);
    //krClient.call(cf);
    //cf.request.fluent.parameters.clear();
	//cf.request.fluent.name ="facing";
	//cf.request.fluent.parameters.push_back(door);
    //krClient.call(cf);
        
	LogicalNavigation setInitialState("noop");
	setInitialState.run();


//    std::cerr << "gui is called" << std::endl;
//    // DO NOT NEED TO DO AT THIS TIME Pass along "fake" observation to KR node
//    std::vector<bwi_planning_common::PlannerAtom> observations;
//    bwi_planning_common::PlannerAtom observation1;
//    bwi_planning_common::PlannerAtom observation2;
//    observation1.name = "at";
//    observation1.value.push_back("f2_ele1");
//    //observation1.name = "facing";
//    //observation1.value.push_back(door);
//    observation2.name = "facing";
//    observation2.value.push_back(door);
//    observations.push_back(observation1);
//    ros::ServiceClient krClient = n.serviceClient<bwi_kr::ChangeFluent> ( "/bwi_kr/change_fluent" );
//    krClient.waitForExistence();
//        bwi_kr::ChangeFluent cf;
//    for(int i=0, size=observations.size() ; i < size; ++i) {
//		cf.request.fluent.name =observations[i].name;
//		cf.request.fluent.parameters = observations[i].value;
//    }
//        krClient.call(cf);
//

    std::cerr << "done is set" << std::endl;
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
