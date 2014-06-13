#ifndef bwi_actexec_Load_h__guard
#define bwi_actexec_Load_h__guard

#include "Action.h"

namespace bwi_actexec {


class Load : public bwi_actexec::Action {
public:

	Load (const std::string &name);

	void init(const std::vector<std::string>& params);

	int paramNumber() const {return 1;}

	std::string getName() const {return name;}

	void run();

	bool hasFinished() const {return done;}

	Action *clone() const {return new Load(*this);}

private:

	virtual std::vector<std::string> getParameters() const {return parameters;}

	std::string name;
	std::vector<std::string> parameters;
	bool done;

};
}

#endif
