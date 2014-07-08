
#ifndef test_SimpleAction_h__guard
#define test_SimpleAction_h__guard

#include <actasp/Action.h>

#include <iostream>

class SimpleAction : public actasp::Action {
public:

	SimpleAction(const std::string &name) : name(name), done(false) {}

	int paramNumber() const {
		return 0;
	}

	std::string getName() const {
		return name;
	}

	void run() {
		std::cout << "running " << name << std::endl;
		done = true;
	}

	bool hasFinished() const {
		return done;
	}

	virtual actasp::Action *cloneAndInit(const actasp::AspFluent &) const {
		return new SimpleAction(*this);
	}
	
	virtual actasp::Action *clone() const {
		return new SimpleAction(*this);
	}

private:
	std::string name;
	bool done;

	std::vector<std::string> getParameters() const {
		return std::vector<std::string>();
	}
};

#endif