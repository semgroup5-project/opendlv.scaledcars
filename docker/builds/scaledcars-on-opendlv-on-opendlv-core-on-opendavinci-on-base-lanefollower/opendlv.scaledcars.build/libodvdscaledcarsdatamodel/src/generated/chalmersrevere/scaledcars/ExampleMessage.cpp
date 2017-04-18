/*
 * This software is open source. Please see COPYING and AUTHORS for further information.
 *
 * This file is auto-generated. DO NOT CHANGE AS YOUR CHANGES MIGHT BE OVERWRITTEN!
 */

#include <memory>

#include <opendavinci/odcore/serialization/Deserializer.h>
#include <opendavinci/odcore/serialization/SerializationFactory.h>
#include <opendavinci/odcore/serialization/Serializer.h>

#include "odvdscaledcarsdatamodel/generated/chalmersrevere/scaledcars/ExampleMessage.h"

namespace chalmersrevere {
	namespace scaledcars {
			using namespace std;
			using namespace odcore::base;
			using namespace odcore::serialization;
		
		
			ExampleMessage::ExampleMessage() :
			    SerializableData(), Visitable()
				, m_field1(0)
			{
			}
		
			ExampleMessage::ExampleMessage(
				const uint32_t &val0
			) :
			    SerializableData(), Visitable()
				, m_field1(val0)
			{
			}
		
			ExampleMessage::ExampleMessage(const ExampleMessage &obj) :
			    SerializableData(), Visitable()
				, m_field1(obj.m_field1)
			{
			}
			
			ExampleMessage::~ExampleMessage() {
			}
		
			ExampleMessage& ExampleMessage::operator=(const ExampleMessage &obj) {
				m_field1 = obj.m_field1;
				return (*this);
			}
		
			int32_t ExampleMessage::ID() {
				return 801;
			}
		
			const string ExampleMessage::ShortName() {
				return "ExampleMessage";
			}
		
			const string ExampleMessage::LongName() {
				return "chalmersrevere.scaledcars.ExampleMessage";
			}
		
			int32_t ExampleMessage::getID() const {
				return ExampleMessage::ID();
			}
		
			const string ExampleMessage::getShortName() const {
				return ExampleMessage::ShortName();
			}
		
			const string ExampleMessage::getLongName() const {
				return ExampleMessage::LongName();
			}
		
			uint32_t ExampleMessage::getField1() const {
				return m_field1;
			}
			
			void ExampleMessage::setField1(const uint32_t &val) {
				m_field1 = val;
			}
		
			void ExampleMessage::accept(odcore::base::Visitor &v) {
				v.beginVisit(ID(), ShortName(), LongName());
				v.visit(1, "ExampleMessage.field1", "field1", m_field1);
				v.endVisit();
			}
		
			const string ExampleMessage::toString() const {
				stringstream s;
		
				s << "Field1: " << getField1() << " ";
		
				return s.str();
			}
		
			ostream& ExampleMessage::operator<<(ostream &out) const {
				SerializationFactory& sf = SerializationFactory::getInstance();
		
				std::shared_ptr<Serializer> s = sf.getSerializer(out);
		
				s->write(1,
						m_field1);
				return out;
			}
		
			istream& ExampleMessage::operator>>(istream &in) {
				SerializationFactory& sf = SerializationFactory::getInstance();
		
				std::shared_ptr<Deserializer> d = sf.getDeserializer(in);
		
				d->read(1,
						m_field1);
				return in;
			}
	} // scaledcars
} // chalmersrevere
