/*
 * THIS IS A GENERATED FILE - CHANGES WILL BE OVERWRITTEN.
 */

#include <iostream>
#include <odvdscaledcarsdatamodel/GeneratedHeaders_ODVDScaledCarsDataModel.h>
#include <odvdscaledcarsdatamodel/GeneratedHeaders_ODVDScaledCarsDataModel_Helper.h>
#include <opendavinci/odcore/base/Visitable.h>
#include <opendavinci/odcore/reflection/MessageFromVisitableVisitor.h>
extern "C" {
    odcore::reflection::Helper *newHelper() {
        return new GeneratedHeaders_ODVDScaledCarsDataModel_Helper;
    }
    void deleteHelper(odcore::reflection::Helper *h) {
        delete h;
    }
}
GeneratedHeaders_ODVDScaledCarsDataModel_Helper::~GeneratedHeaders_ODVDScaledCarsDataModel_Helper() {}
void GeneratedHeaders_ODVDScaledCarsDataModel_Helper::delegateVistor(odcore::data::Container &c, odcore::base::Visitor &v, bool &successfullyDelegated) {
    GeneratedHeaders_ODVDScaledCarsDataModel_Helper::__delegateVistor(c, v, successfullyDelegated);
}
odcore::reflection::Message GeneratedHeaders_ODVDScaledCarsDataModel_Helper::map(odcore::data::Container &c, bool &successfullyMapped) {
    return GeneratedHeaders_ODVDScaledCarsDataModel_Helper::__map(c, successfullyMapped);
}
void GeneratedHeaders_ODVDScaledCarsDataModel_Helper::__delegateVistor(odcore::data::Container &c, odcore::base::Visitor &v, bool &successfullyDelegated) {
    successfullyDelegated = false;
    if (c.getDataType() == chalmersrevere::scaledcars::ExampleMessage::ID()) {
        chalmersrevere::scaledcars::ExampleMessage payload = c.getData<chalmersrevere::scaledcars::ExampleMessage>();
        payload.accept(v);
        successfullyDelegated = true;
    }
}
odcore::reflection::Message GeneratedHeaders_ODVDScaledCarsDataModel_Helper::__map(odcore::data::Container &c, bool &successfullyMapped) {
    successfullyMapped = false;
    odcore::reflection::Message msg;
    odcore::reflection::MessageFromVisitableVisitor mfvv;
    __delegateVistor(c, mfvv, successfullyMapped);
    if (successfullyMapped) {
        msg = mfvv.getMessage();
    }
    return msg;
}
