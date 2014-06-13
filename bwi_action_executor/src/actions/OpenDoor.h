#ifndef bwi_actexec_OpenDoor_h__guard
#define bwi_actexec_OpenDoor_h__guard

#include "Action.h"

namespace bwi_actexec {

	
class OpenDoor : public bwi_actexec::Action {
public:
	
	OpenDoor(const std::string &name);
	
	void init(const std::vector<std::string>& params);
	
	int paramNumber() const {return 1;}
	
	std::string getName() const {return name;}
	
	void run();
	
	bool hasFinished() const {return doorOpen;}
	
	Action *clone() const {return new OpenDoor(*this);}
	
private:
	
	virtual std::vector<std::string> getParameters() const;
	
	std::string name;
	std::string doorName;
	bool asked;
	bool doorOpen;

};	
}

#endif