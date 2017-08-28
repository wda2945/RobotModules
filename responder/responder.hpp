/*
 * ResponderClass.hpp
 *
 *  Created on: Aug 7, 2015
 *      Author: martin
 */

#ifndef RESPONDER_HPP_
#define RESPONDER_HPP_

#include "ResponderClass.hpp"

//responder
class Responder : public ResponderClass
{
	Responder();
	~Responder() {}

	void process_message_method(const void *msg, int len);

public:

	void register_option(const char *domain, const char *name, int value) override;
	void register_setting(const char *domain, const char *name, float minV, float maxV, float value) override;
	void register_condition(const char *domain, int c, const char *name) override;

private:

	friend Responder &the_responder_instance();
	friend void ResponderProcessMessage(const void *msg, int len);
};

Responder &the_responder_instance();

#endif /* RESPONDER_HPP_ */
