/*
 * ResponderClass.hpp
 *
 *  Created on: Aug 7, 2015
 *      Author: martin
 */

#ifndef RESPONDER_CLASS_HPP_
#define RESPONDER_CLASS_HPP_

//responder
class ResponderClass
{
public:
	ResponderClass() {}
	virtual ~ResponderClass() {}

	virtual void register_option(const char *domain, const char *name, int value) = 0;
	virtual void register_setting(const char *domain, const char *name, float minV, float maxV, float value) = 0;
	virtual void register_condition(const char *domain, int c, const char *name) = 0;
};

ResponderClass &the_responder();

#endif /* RESPONDER_HPP_ */
