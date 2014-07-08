
#include "unittest++/UnitTest++.h"

#include <actasp/AspFluent.h>

using namespace std;
using namespace actasp;

TEST(AspFluentConstructor) {
	
	CHECK_THROW(AspFluent("Fluent(a,b,c"), invalid_argument);
	
	CHECK_THROW(AspFluent("Fluenta,b,c)"), invalid_argument);
	
	CHECK_THROW(AspFluent("Fluent"), invalid_argument);
	
	CHECK_THROW(AspFluent(""), invalid_argument);
	
}

TEST(AspFluentComparison) {
	
	CHECK(AspFluent("a(a,b,0)") < AspFluent("b(a,b,0)"));
	
	CHECK(AspFluent("a(a,b,0)") < AspFluent("a(a,b,1)"));
	
	CHECK(!(AspFluent("a(a,b,1)") < AspFluent("a(a,b,0)")));
	
	CHECK(!(AspFluent("a(a,b,0)") < AspFluent("a(a,b,0)")));
	
	CHECK(!(AspFluent("b(a,b,0)") < AspFluent("a(a,b,0)")));
	
	CHECK(AspFluent("-a(a,b,0)") < AspFluent("a(a,b,0)"));
	
}