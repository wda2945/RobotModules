/*
 * responder.c
 *
 * Responds to ping messages from the APP
 *
 *  Created on: Jul 27, 2014
 *      Author: martin
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#include "robot.h"
#include "responder.hpp"
#include "main_debug.h"

void ResponderProcessMessage(const void *msg, int len)
{
	the_responder_instance().process_message_method(msg, len);
}

Responder::Responder()
{
	DEBUGPRINT("Add options/settings to registry");

#define optionmacro(name, val, minV, maxV, def) register_option(OPTIONS_DOMAIN, name, val);
#include "options.h"
#undef optionmacro
	sleep(1);
#define settingmacro(name, val, minV, maxV, def) register_setting(SETTINGS_DOMAIN, name, minV, maxV, val);
#include "settings.h"
#undef settingmacro

	sleep(1);
	DEBUGPRINT("Add conditions to registry");

#define CONDITION_MACRO(e, n) register_condition(STATUS_DOMAIN, e, n);
#include "conditions_status.h"
#undef CONDITION_MACRO

#define CONDITION_MACRO(e, n) register_condition(ERRORS_DOMAIN, e, n);
#include "conditions_errors.h"
#undef CONDITION_MACRO

	DEBUGPRINT("Subscribe to Announcements");
	ps_subscribe(ANNOUNCEMENTS_TOPIC, ResponderProcessMessage);

}

void Responder::process_message_method(const void *_msg, int len)
{
	psMessage_t *msg = (psMessage_t *) _msg;
	switch (msg->messageType)
	{

	case PING_MSG:
	{
		DEBUGPRINT("Ping msg received");
		psMessage_t msg2;
		msg2.responsePayload.source = SOURCE;
		msg2.responsePayload.flags = 0;
		psInitPublish(msg2, PING_RESPONSE);
		RouteMessage(msg2);

		ps_registry_send_sync();
	}
	break;

	default:
		//ignore anything else
		break;
	}

}

void Responder::register_option(const char *domain, const char *name, int value) {
	ps_registry_api_struct registry_value;

	registry_value.datatype = PS_REGISTRY_BOOL_TYPE;
	registry_value.bool_value = value;
	registry_value.flags = (ps_registry_flags_t) PS_REGISTRY_SRC_WRITE | PS_REGISTRY_ANY_WRITE;

	if (ps_registry_set_new(domain, name, registry_value) == PS_OK)
	{
		DEBUGPRINT("Registered option %s/%s", domain, name);
	}
	else
	{
		ERRORPRINT("Registering option %s/%s failed", domain, name);
	}
}

void Responder::register_setting(const char *domain, const char *name, float minV, float maxV, float value) {
	ps_registry_api_struct registry_value;

	registry_value.datatype = PS_REGISTRY_SETTING_TYPE;
	registry_value.setting.minimum = minV;
	registry_value.setting.maximum = maxV;
	registry_value.setting.value = value;
	registry_value.flags = (ps_registry_flags_t) PS_REGISTRY_SRC_WRITE | PS_REGISTRY_ANY_WRITE;

	if (ps_registry_set_new(domain, name, registry_value) == PS_OK)
	{
		DEBUGPRINT("Registered setting %s/%s", domain, name);
	}
	else
	{
		ERRORPRINT("Registering setting %s/%s failed", domain, name);
	}
}


void Responder::register_condition(const char *domain, int c, const char *name) {

	if (c == 0) return;	//null

	ps_registry_api_struct registry_value;

	registry_value.datatype = PS_REGISTRY_INT_TYPE;
	registry_value.int_value = c;
	registry_value.flags = (ps_registry_flags_t) PS_REGISTRY_READ_ONLY;

	if (ps_registry_set_new(domain, name, registry_value) == PS_OK)
	{
		DEBUGPRINT("Registered condition %i = %s/%s", c, domain, name);
	}
	else
	{
		ERRORPRINT("Error registering condition %i = %s/%s", c, domain, name);
	}
}

ResponderClass& the_responder()
{
	return the_responder_instance();
}

Responder &the_responder_instance()
{
	static Responder me;
	return me;
}
