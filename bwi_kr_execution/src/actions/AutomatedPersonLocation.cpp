#include "AutomatedPersonLocation.h"

#include<ros/ros.h>
#include<fstream>

namespace bwi_krexec {

  void readAutomatedPersonLocation(std::map<std::string, std::string>& person_location_map) {

    person_location_map.clear();

    // Get the file from the ros parameter.
    ros::NodeHandle private_nh("~");
    std::string automated_person_locations_file;
    private_nh.getParam("automated_person_locations_file", automated_person_locations_file);

    // Read in the person and their locations.
    std::ifstream fin(automated_person_locations_file.c_str());
    std::string person, location;
    fin >> person;
    fin >> location;
    while (!fin.eof()) {
      person_location_map[person] = location;
      fin >> person;
      fin >> location;
    }

  }

}

