/*
 * This software is open source. Please see COPYING and AUTHORS for further information.
 *
 * This file is auto-generated. DO NOT CHANGE AS YOUR CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef CHALMERSREVERE_SCALEDCARS_EXAMPLEMESSAGE_H
#define CHALMERSREVERE_SCALEDCARS_EXAMPLEMESSAGE_H

#include "opendavinci/odcore/opendavinci.h"


#include "opendavinci/odcore/base/Visitable.h"
#include "opendavinci/odcore/data/SerializableData.h"


namespace chalmersrevere {
	namespace scaledcars {
		using namespace std;
		
		class OPENDAVINCI_API ExampleMessage : public odcore::data::SerializableData, public odcore::base::Visitable {
			public:
				ExampleMessage();
		
				ExampleMessage(
				const uint32_t &val0
				);
		
				virtual ~ExampleMessage();
		
				/**
				 * Copy constructor.
				 *
				 * @param obj Reference to an object of this class.
				 */
				ExampleMessage(const ExampleMessage &obj);
		
				/**
				 * Assignment operator.
				 *
				 * @param obj Reference to an object of this class.
				 * @return Reference to this instance.
				 */
				ExampleMessage& operator=(const ExampleMessage &obj);
		
			public:
				/**
				 * @return field1.
				 */
				uint32_t getField1() const;
				
				/**
				 * This method sets field1.
				 *
				 * @param val Value for field1.
				 */
				void setField1(const uint32_t &val);
		
			public:
				/**
				 * This method returns the message id.
				 *
				 * @return Message id.
				 */
				static int32_t ID();
		
				/**
				 * This method returns the short message name.
				 *
				 * @return Short message name.
				 */
				static const string ShortName();
		
				/**
				 * This method returns the long message name include package/sub structure.
				 *
				 * @return Long message name.
				 */
				static const string LongName();
		
				/**
				 * This method returns the message id.
				 *
				 * @return Message id.
				 */
				virtual int32_t getID() const;
		
				/**
				 * This method returns the short message name.
				 *
				 * @return Short message name.
				 */
				virtual const string getShortName() const;
		
				/**
				 * This method returns the long message name include package/sub structure.
				 *
				 * @return Long message name.
				 */
				virtual const string getLongName() const;
		
			public:
				virtual void accept(odcore::base::Visitor &v);
		
				virtual ostream& operator<<(ostream &out) const;
				virtual istream& operator>>(istream &in);
		
				virtual const string toString() const;
		
			private:
				uint32_t m_field1;
		
		};
	} // scaledcars
} // chalmersrevere

#endif /*CHALMERSREVERE_SCALEDCARS_EXAMPLEMESSAGE_H*/
