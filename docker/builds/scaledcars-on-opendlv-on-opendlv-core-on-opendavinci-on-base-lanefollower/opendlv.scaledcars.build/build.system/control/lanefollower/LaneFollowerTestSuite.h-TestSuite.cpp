/* Generated file, do not edit */

#ifndef CXXTEST_RUNNING
#define CXXTEST_RUNNING
#endif

#include <fstream>
#define _CXXTEST_HAVE_STD
#define _CXXTEST_HAVE_EH
#include <cxxtest/TestListener.h>
#include <cxxtest/TestTracker.h>
#include <cxxtest/TestRunner.h>
#include <cxxtest/RealDescriptions.h>
#include <cxxtest/TestMain.h>
#include <cxxtest/XUnitPrinter.h>

int main( int argc, char *argv[] ) {
 int status;
    std::ofstream ofstr("TEST-scaledcars-control-lanefollwer-LaneFollowerTestSuite.h.xml");
    CxxTest::XUnitPrinter tmp(ofstr);
    CxxTest::RealWorldDescription::_worldName = "scaledcars-control-lanefollwer-LaneFollowerTestSuite.h";
    status = CxxTest::Main< CxxTest::XUnitPrinter >( tmp, argc, argv );
    return status;
}
bool suite_LaneFollowerTest_init = false;
#include "../../../../opendlv.scaledcars.sources/code/control/lanefollower/testsuites/LaneFollowerTestSuite.h"

static LaneFollowerTest suite_LaneFollowerTest;

static CxxTest::List Tests_LaneFollowerTest = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_LaneFollowerTest( "/opt/opendlv.scaledcars.sources/code/control/lanefollower/testsuites/LaneFollowerTestSuite.h", 67, "LaneFollowerTest", suite_LaneFollowerTest, Tests_LaneFollowerTest );

static class TestDescription_suite_LaneFollowerTest_testLaneFollowerSuccessfullyCreated : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_LaneFollowerTest_testLaneFollowerSuccessfullyCreated() : CxxTest::RealTestDescription( Tests_LaneFollowerTest, suiteDescription_LaneFollowerTest, 101, "testLaneFollowerSuccessfullyCreated" ) {}
 void runTest() { suite_LaneFollowerTest.testLaneFollowerSuccessfullyCreated(); }
} testDescription_suite_LaneFollowerTest_testLaneFollowerSuccessfullyCreated;

#include <cxxtest/Root.cpp>
const char* CxxTest::RealWorldDescription::_worldName = "cxxtest";
