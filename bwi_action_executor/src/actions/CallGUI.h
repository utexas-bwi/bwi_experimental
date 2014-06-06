#ifndef bwi_actexec_CallGUI_h__guard
#define bwi_actexec_CallGUI_h__guard

#include "Action.h"

#include <segbot_gui/QuestionDialogRequest.h>

namespace bwi_actexec {

class CallGUI : public Action {
public:

	enum TYPE {
	    DISPLAY = segbot_gui::QuestionDialogRequest::DISPLAY,
	    CHOICE_QUESTION = segbot_gui::QuestionDialogRequest::CHOICE_QUESTION,
	    TEXT_QUESTION = segbot_gui::QuestionDialogRequest::TEXT_QUESTION
	};

	CallGUI ( const std::string &name, const TYPE type,  const std::string& message,
	          float timeOut = 0.0f,
	          const std::vector<std::string> &options = std::vector<std::string>() );

	std::string getName() const {
		return name;
	}

	int paramNumber() const {
		return 0;
	}

	void run();

	bool hasFinished() const;

	CallGUI *clone() const {
		return new CallGUI ( *this );
	};

private:
	
	virtual std::vector<std::string> getParameters() const { return std::vector<std::string>();}
	
	std::string name;
	TYPE type;
	std::string message;
	float timeOut;
	std::vector<std::string> options;
	bool done;
};

}

#endif