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
    std::ofstream ofstr("TEST-scaledcars-perception-example-ExamplePerceptionTestSuite.h.xml");
    CxxTest::XUnitPrinter tmp(ofstr);
    CxxTest::RealWorldDescription::_worldName = "scaledcars-perception-example-ExamplePerceptionTestSuite.h";
    status = CxxTest::Main< CxxTest::XUnitPrinter >( tmp, argc, argv );
    return status;
}
bool suite_ExamplePerceptionTest_init = false;
#include "../../../../opendlv.scaledcars.sources/code/perception/example/testsuites/ExamplePerceptionTestSuite.h"

static ExamplePerceptionTest suite_ExamplePerceptionTest;

static CxxTest::List Tests_ExamplePerceptionTest = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_ExamplePerceptionTest( "/opt/opendlv.scaledcars.sources/code/perception/example/testsuites/ExamplePerceptionTestSuite.h", 31, "ExamplePerceptionTest", suite_ExamplePerceptionTest, Tests_ExamplePerceptionTest );

static class TestDescription_suite_ExamplePerceptionTest_testApplication : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_ExamplePerceptionTest_testApplication() : CxxTest::RealTestDescription( Tests_ExamplePerceptionTest, suiteDescription_ExamplePerceptionTest, 37, "testApplication" ) {}
 void runTest() { suite_ExamplePerceptionTest.testApplication(); }
} testDescription_suite_ExamplePerceptionTest_testApplication;

#include <cxxtest/Root.cpp>
const char* CxxTest::RealWorldDescription::_worldName = "cxxtest";
