/*
 * This software is open source. Please see COPYING and AUTHORS for further information.
 *
 * This file is auto-generated. DO NOT CHANGE AS YOUR CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef CHALMERSREVERE_SCALEDCARS_EXAMPLEMESSAGE_TESTSUITE_H
#define CHALMERSREVERE_SCALEDCARS_EXAMPLEMESSAGE_TESTSUITE_H

#include "cxxtest/TestSuite.h"

#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <opendavinci/odcore/opendavinci.h>
#include <opendavinci/odcore/strings/StringToolbox.h>

#include "odvdscaledcarsdatamodel/GeneratedHeaders_ODVDScaledCarsDataModel.h"
#include "odvdscaledcarsdatamodel/generated/chalmersrevere/scaledcars/ExampleMessage.h"


class chalmersrevere_scaledcars_ExampleMessage_TestSuite : public CxxTest::TestSuite {

	public:
		void testCreateObject() {
			using namespace chalmersrevere::scaledcars;
	
			ExampleMessage obj1;
			TS_ASSERT(obj1.getField1() == 0);
	
			obj1.setField1(4000);
	
			TS_ASSERT(obj1.getField1() == 4000);
		}
	
		void testCreateAndCopyObject() {
			using namespace chalmersrevere::scaledcars;
	
			ExampleMessage obj1;
			TS_ASSERT(obj1.getField1() == 0);
	
			ExampleMessage obj2(obj1);
			TS_ASSERT(obj1.getField1() == obj2.getField1());
	
			obj1.setField1(4000);
	
			TS_ASSERT(obj1.getField1() == 4000);
	
			ExampleMessage obj3(obj1);
			TS_ASSERT(obj1.getField1() == obj3.getField1());
	
			TS_ASSERT(obj3.getField1() == 4000);
	
			TS_ASSERT(odcore::strings::StringToolbox::equalsIgnoreCase(obj1.toString(), obj3.toString()));
		}
	
		void testCreateAndAssignObject() {
			using namespace chalmersrevere::scaledcars;
	
			ExampleMessage obj1;
			TS_ASSERT(obj1.getField1() == 0);
	
			ExampleMessage obj2;
			TS_ASSERT(obj2.getField1() == 0);
	
			obj1.setField1(4000);
	
			TS_ASSERT(obj1.getField1() == 4000);
	
			obj2 = obj1;
			TS_ASSERT(obj1.getField1() == obj2.getField1());
	
			TS_ASSERT(obj2.getField1() == 4000);
	
			TS_ASSERT(odcore::strings::StringToolbox::equalsIgnoreCase(obj1.toString(), obj2.toString()));
		}
	
		void testCreateAndSerializeObject() {
			using namespace chalmersrevere::scaledcars;
	
			ExampleMessage obj1;
			TS_ASSERT(obj1.getField1() == 0);
	
			ExampleMessage obj2;
			TS_ASSERT(obj2.getField1() == 0);
	
			obj1.setField1(4000);
	
			TS_ASSERT(obj1.getField1() == 4000);
	
			stringstream sstr;
			sstr << obj1;
			sstr >> obj2;
	
			TS_ASSERT(obj1.getField1() == obj2.getField1());
	
			TS_ASSERT(obj2.getField1() == 4000);
	
			TS_ASSERT(odcore::strings::StringToolbox::equalsIgnoreCase(obj1.toString(), obj2.toString()));
		}

};

#endif /*CHALMERSREVERE_SCALEDCARS_EXAMPLEMESSAGE_TESTSUITE_H*/
