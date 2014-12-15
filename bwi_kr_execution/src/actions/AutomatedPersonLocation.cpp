#include "AutomatedPersonLocation.h"

#include<ros/ros.h>
#include<fstream>

namespace bwi_krexec {

  void readAutomatedPersonLocation(std::map<std::string, std::string>& person_location_map) {

    person_location_map.clear();

    // Get the file from the ros parameter.
    ros::NodeHandle private_nh("~");
    std::string automated_person_locations_file;
    private_nh.getParam("automated_person_location_file", automated_person_locations_file);

    // Read in the person and their locations.
    std::ifstream fin(automated_person_locations_file.c_str());
    if (!fin) {
      // File was not opened correctly.
      ROS_ERROR_STREAM("Unable to read automated person location file: " << automated_person_locations_file);
      return;
    }
    
    std::string person, location;
    while (!fin.eof()) {
      fin >> person;
      if (fin.eof()) {
        break;
      }
      fin >> location;
      person_location_map[person] = location;
      ROS_INFO_STREAM("AutomatedPersonLocation: " << person << " is in " << location);
    }

    fin.close();

  }

}

