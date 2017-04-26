/* Generated file, do not edit */

#ifndef CXXTEST_RUNNING
#define CXXTEST_RUNNING
#endif

#include <fstream>
#define _CXXTEST_HAVE_EH
#include <cxxtest/TestListener.h>
#include <cxxtest/TestTracker.h>
#include <cxxtest/TestRunner.h>
#include <cxxtest/RealDescriptions.h>
#include <cxxtest/TestMain.h>
#include <cxxtest/XUnitPrinter.h>

int main( int argc, char *argv[] ) {
 int status;
    std::ofstream ofstr("TEST-libodvdscaledcarsdatamodel-chalmersrevere_scaledcars_ExampleMessageTestSuite.h.xml");
    CxxTest::XUnitPrinter tmp(ofstr);
    CxxTest::RealWorldDescription::_worldName = "libodvdscaledcarsdatamodel-chalmersrevere_scaledcars_ExampleMessageTestSuite.h";
    status = CxxTest::Main< CxxTest::XUnitPrinter >( tmp, argc, argv );
    return status;
}
bool suite_chalmersrevere_scaledcars_ExampleMessage_TestSuite_init = false;
#include "../../../libodvdscaledcarsdatamodel/testsuites/chalmersrevere_scaledcars_ExampleMessageTestSuite.h"

static chalmersrevere_scaledcars_ExampleMessage_TestSuite suite_chalmersrevere_scaledcars_ExampleMessage_TestSuite;

static CxxTest::List Tests_chalmersrevere_scaledcars_ExampleMessage_TestSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_chalmersrevere_scaledcars_ExampleMessage_TestSuite( "/opt/opendlv.scaledcars.build/libodvdscaledcarsdatamodel/testsuites/chalmersrevere_scaledcars_ExampleMessageTestSuite.h", 25, "chalmersrevere_scaledcars_ExampleMessage_TestSuite", suite_chalmersrevere_scaledcars_ExampleMessage_TestSuite, Tests_chalmersrevere_scaledcars_ExampleMessage_TestSuite );

static class TestDescription_suite_chalmersrevere_scaledcars_ExampleMessage_TestSuite_testCreateObject : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_chalmersrevere_scaledcars_ExampleMessage_TestSuite_testCreateObject() : CxxTest::RealTestDescription( Tests_chalmersrevere_scaledcars_ExampleMessage_TestSuite, suiteDescription_chalmersrevere_scaledcars_ExampleMessage_TestSuite, 28, "testCreateObject" ) {}
 void runTest() { suite_chalmersrevere_scaledcars_ExampleMessage_TestSuite.testCreateObject(); }
} testDescription_suite_chalmersrevere_scaledcars_ExampleMessage_TestSuite_testCreateObject;

static class TestDescription_suite_chalmersrevere_scaledcars_ExampleMessage_TestSuite_testCreateAndCopyObject : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_chalmersrevere_scaledcars_ExampleMessage_TestSuite_testCreateAndCopyObject() : CxxTest::RealTestDescription( Tests_chalmersrevere_scaledcars_ExampleMessage_TestSuite, suiteDescription_chalmersrevere_scaledcars_ExampleMessage_TestSuite, 39, "testCreateAndCopyObject" ) {}
 void runTest() { suite_chalmersrevere_scaledcars_ExampleMessage_TestSuite.testCreateAndCopyObject(); }
} testDescription_suite_chalmersrevere_scaledcars_ExampleMessage_TestSuite_testCreateAndCopyObject;

static class TestDescription_suite_chalmersrevere_scaledcars_ExampleMessage_TestSuite_testCreateAndAssignObject : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_chalmersrevere_scaledcars_ExampleMessage_TestSuite_testCreateAndAssignObject() : CxxTest::RealTestDescription( Tests_chalmersrevere_scaledcars_ExampleMessage_TestSuite, suiteDescription_chalmersrevere_scaledcars_ExampleMessage_TestSuite, 60, "testCreateAndAssignObject" ) {}
 void runTest() { suite_chalmersrevere_scaledcars_ExampleMessage_TestSuite.testCreateAndAssignObject(); }
} testDescription_suite_chalmersrevere_scaledcars_ExampleMessage_TestSuite_testCreateAndAssignObject;

static class TestDescription_suite_chalmersrevere_scaledcars_ExampleMessage_TestSuite_testCreateAndSerializeObject : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_chalmersrevere_scaledcars_ExampleMessage_TestSuite_testCreateAndSerializeObject() : CxxTest::RealTestDescription( Tests_chalmersrevere_scaledcars_ExampleMessage_TestSuite, suiteDescription_chalmersrevere_scaledcars_ExampleMessage_TestSuite, 81, "testCreateAndSerializeObject" ) {}
 void runTest() { suite_chalmersrevere_scaledcars_ExampleMessage_TestSuite.testCreateAndSerializeObject(); }
} testDescription_suite_chalmersrevere_scaledcars_ExampleMessage_TestSuite_testCreateAndSerializeObject;

#include <cxxtest/Root.cpp>
const char* CxxTest::RealWorldDescription::_worldName = "cxxtest";
