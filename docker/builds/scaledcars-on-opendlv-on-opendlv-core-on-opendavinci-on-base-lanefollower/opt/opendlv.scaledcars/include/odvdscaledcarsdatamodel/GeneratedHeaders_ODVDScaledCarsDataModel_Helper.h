/*
 * THIS IS A GENERATED FILE - CHANGES WILL BE OVERWRITTEN.
 */

#ifndef GENERATEDHEADERS_ODVDSCALEDCARSDATAMODEL_HELPER_H
#define GENERATEDHEADERS_ODVDSCALEDCARSDATAMODEL_HELPER_H

#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/base/Visitor.h>
#include <opendavinci/odcore/reflection/Message.h>
#include <opendavinci/odcore/reflection/Helper.h>
class GeneratedHeaders_ODVDScaledCarsDataModel_Helper : public odcore::reflection::Helper {
    public:
        virtual ~GeneratedHeaders_ODVDScaledCarsDataModel_Helper();
        virtual void delegateVistor(odcore::data::Container &c, odcore::base::Visitor &v, bool &successfullyDelegated);
        virtual odcore::reflection::Message map(odcore::data::Container &c, bool &successfullyMapped);
        static void __delegateVistor(odcore::data::Container &c, odcore::base::Visitor &v, bool &successfullyDelegated);
        static odcore::reflection::Message __map(odcore::data::Container &c, bool &successfullyMapped);
};

#endif //GENERATEDHEADERS_ODVDSCALEDCARSDATAMODEL_HELPER_H
