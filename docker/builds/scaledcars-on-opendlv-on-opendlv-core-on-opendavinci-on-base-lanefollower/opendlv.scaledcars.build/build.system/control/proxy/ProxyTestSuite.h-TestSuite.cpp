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
    std::ofstream ofstr("TEST-scaledcars-control-proxy-ProxyTestSuite.h.xml");
    CxxTest::XUnitPrinter tmp(ofstr);
    CxxTest::RealWorldDescription::_worldName = "scaledcars-control-proxy-ProxyTestSuite.h";
    status = CxxTest::Main< CxxTest::XUnitPrinter >( tmp, argc, argv );
    return status;
}
bool suite_ProxyTest_init = false;
#include "../../../../opendlv.scaledcars.sources/code/control/proxy/testsuites/ProxyTestSuite.h"

static ProxyTest suite_ProxyTest;

static CxxTest::List Tests_ProxyTest = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_ProxyTest( "/opt/opendlv.scaledcars.sources/code/control/proxy/testsuites/ProxyTestSuite.h", 31, "ProxyTest", suite_ProxyTest, Tests_ProxyTest );

static class TestDescription_suite_ProxyTest_testApplication : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_ProxyTest_testApplication() : CxxTest::RealTestDescription( Tests_ProxyTest, suiteDescription_ProxyTest, 37, "testApplication" ) {}
 void runTest() { suite_ProxyTest.testApplication(); }
} testDescription_suite_ProxyTest_testApplication;

#include <cxxtest/Root.cpp>
const char* CxxTest::RealWorldDescription::_worldName = "cxxtest";
