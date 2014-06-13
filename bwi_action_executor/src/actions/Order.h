#ifndef bwi_actexec_Order_h__guard
#define bwi_actexec_Order_h__guard

#include "Action.h"

namespace bwi_actexec {


class Order : public bwi_actexec::Action {
public:

	Order(const std::string &name);

	void init(const std::vector<std::string>& params);

	int paramNumber() const {return 1;}

	std::string getName() const {return name;}

	void run();

	bool hasFinished() const {return done;}

	Action *clone() const {return new Order(*this);}

private:
	virtual std::vector<std::string> getParameters() const {return parameters;}

	std::string name;
	std::vector<std::string> parameters;
	bool done;

};
}

#endif
