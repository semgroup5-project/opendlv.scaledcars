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
    std::ofstream ofstr("TEST-scaledcars-control-example-ExampleControlTestSuite.h.xml");
    CxxTest::XUnitPrinter tmp(ofstr);
    CxxTest::RealWorldDescription::_worldName = "scaledcars-control-example-ExampleControlTestSuite.h";
    status = CxxTest::Main< CxxTest::XUnitPrinter >( tmp, argc, argv );
    return status;
}
bool suite_ExampleControlTest_init = false;
#include "../../../../../../opendlv.scaledcars.sources/code/control/example/testsuites/ExampleControlTestSuite.h"

static ExampleControlTest suite_ExampleControlTest;

static CxxTest::List Tests_ExampleControlTest = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_ExampleControlTest( "/opt/opendlv.scaledcars.sources/code/control/example/testsuites/ExampleControlTestSuite.h", 31, "ExampleControlTest", suite_ExampleControlTest, Tests_ExampleControlTest );

static class TestDescription_suite_ExampleControlTest_testApplication : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_ExampleControlTest_testApplication() : CxxTest::RealTestDescription( Tests_ExampleControlTest, suiteDescription_ExampleControlTest, 37, "testApplication" ) {}
 void runTest() { suite_ExampleControlTest.testApplication(); }
} testDescription_suite_ExampleControlTest_testApplication;

#include <cxxtest/Root.cpp>
const char* CxxTest::RealWorldDescription::_worldName = "cxxtest";
