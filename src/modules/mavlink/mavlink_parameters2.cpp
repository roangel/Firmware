/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_parameters2.cpp
 * Mavlink parameters2 manager implementation.
 *
 */

#include <stdio.h>

#include "mavlink_parameters2.h"
#include "mavlink_main.h"

MavlinkParameters2Manager::MavlinkParameters2Manager(Mavlink *mavlink) :
	_mavlink(mavlink)
{
}

unsigned
MavlinkParameters2Manager::get_size()
{
	return MAVLINK_MSG_ID_PARAM_VALUE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
}

void
MavlinkParameters2Manager::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
			break;
		}

	case MAVLINK_MSG_ID_PARAM_SET: {
			break;
		}

	case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
			break;
		}

	default:
		break;
	}
}

void
MavlinkParameters2Manager::send(const hrt_abstime t)
{
	if (_send_all_index >= 0 && _mavlink->boot_complete()) {
		/* send all parameters if requested, but only after the system has booted */

		/* The first thing we send is a hash of all values for the ground
		 * station to try and quickly load a cached copy of our params
		 */

		PX4_WARN("SEND");

		// pack_parameters
		// fill one buffer on stack as example

		mavlink_packed_param_values_t msg{};

		msg.number = 0; /* fill in */
		msg.total_size = 0;

		const int count = param_count_used();

		uint8_t buf_pos = 0;

		for (int i = 0; i < count; i++) {
			const param_t p = param_for_index(i); // TODO: revisit for used index

			size_t size = packed_param_size(p);

			if (buf_pos + size < sizeof(msg.buffer)) {

				PX4_DEBUG("packing p: %s (%d) value size: %lu packed size: %lu", param_name(p), p, param_size(p), size);

				size_t size_packed = pack_param(&msg.buffer[buf_pos], p, i);
				buf_pos += size_packed;
				msg.total_size += size_packed;
				msg.number += 1;

			} else {
				// buffer full, send then reset
				mavlink_msg_packed_param_values_send_struct(_mavlink->get_channel(), &msg);

				msg = mavlink_packed_param_values_t{};
				buf_pos = 0;

				break;
			}
		}

		// finished
		_send_all_index = -1;


		//PX4_WARN("NOW UNPACKING!!!");
		//unpack_test(msg);
	}


}

bool
MavlinkParameters2Manager::send_params()
{
	if (send_one()) {
		return true;

	} else if (send_untransmitted()) {
		return true;

	} else {
		return false;
	}
}

bool
MavlinkParameters2Manager::send_untransmitted()
{
	bool sent_one = false;

	// Check for untransmitted system parameters
	if (_mavlink_parameter_sub < 0) {
		_mavlink_parameter_sub = orb_subscribe(ORB_ID(parameter_update));
	}

	bool param_ready;
	orb_check(_mavlink_parameter_sub, &param_ready);

	if (param_ready) {
		// Clear the ready flag
		struct parameter_update_s value;
		orb_copy(ORB_ID(parameter_update), _mavlink_parameter_sub, &value);

		// Schedule an update if not already the case
		if (_param_update_time == 0) {
			_param_update_time = value.timestamp;
			_param_update_index = 0;
		}
	}

	if ((_param_update_time != 0) && ((_param_update_time + 5 * 1000) < hrt_absolute_time())) {

		param_t param = 0;

		// send out all changed values
		do {
			// skip over all parameters which are not invalid and not used
			do {
				param = param_for_index(_param_update_index);
				++_param_update_index;
			} while (param != PARAM_INVALID && !param_used(param));

			// send parameters which are untransmitted while there is
			// space in the TX buffer
			if ((param != PARAM_INVALID) && param_value_unsaved(param)) {
				int ret = send_param(param);
				char buf[100];
				strncpy(&buf[0], param_name(param), MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
				sent_one = true;

				if (ret != PX4_OK) {
					break;
				}
			}
		} while ((_mavlink->get_free_tx_buf() >= get_size()) && (_param_update_index < (int) param_count()));

		// Flag work as done once all params have been sent
		if (_param_update_index >= (int) param_count()) {
			_param_update_time = 0;
		}
	}

	return sent_one;
}

bool
MavlinkParameters2Manager::send_one()
{
	if (_send_all_index >= 0 && _mavlink->boot_complete()) {
		/* send all parameters if requested, but only after the system has booted */

		/* The first thing we send is a hash of all values for the ground
		 * station to try and quickly load a cached copy of our params
		 */
		if (_send_all_index == PARAM_HASH) {
			/* return hash check for cached params */
			uint32_t hash = param_hash_check();

			/* build the one-off response message */
			mavlink_param_value_t msg;
			msg.param_count = param_count_used();
			msg.param_index = -1;
			strncpy(msg.param_id, HASH_PARAM, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
			msg.param_type = MAV_PARAM_TYPE_UINT32;
			memcpy(&msg.param_value, &hash, sizeof(hash));
			mavlink_msg_param_value_send_struct(_mavlink->get_channel(), &msg);

			/* after this we should start sending all params */
			_send_all_index = 0;

			/* No further action, return now */
			return true;
		}

		/* look for the first parameter which is used */
		param_t p;

		do {
			/* walk through all parameters, including unused ones */
			p = param_for_index(_send_all_index);
			_send_all_index++;
		} while (p != PARAM_INVALID && !param_used(p));

		if (p != PARAM_INVALID) {
			send_param(p);
		}

		if ((p == PARAM_INVALID) || (_send_all_index >= (int) param_count())) {
			_send_all_index = -1;
			return false;

		} else {
			return true;
		}

	} else if (_send_all_index == PARAM_HASH && hrt_absolute_time() > 20 * 1000 * 1000) {
		/* the boot did not seem to ever complete, warn user and set boot complete */
		_mavlink->send_statustext_critical("WARNING: SYSTEM BOOT INCOMPLETE. CHECK CONFIG.");
		_mavlink->set_boot_complete();
	}

	return false;
}

int
MavlinkParameters2Manager::send_param(param_t param, int component_id)
{
	if (param == PARAM_INVALID) {
		return 1;
	}

	/* no free TX buf to send this param */
	if (_mavlink->get_free_tx_buf() < MAVLINK_MSG_ID_PARAM_VALUE_LEN) {
		return 1;
	}

	mavlink_param_value_t msg;

	/*
	 * get param value, since MAVLink encodes float and int params in the same
	 * space during transmission, copy param onto float val_buf
	 */
	if (param_get(param, &msg.param_value) != OK) {
		return 2;
	}

	msg.param_count = param_count_used();
	msg.param_index = param_get_used_index(param);

#if defined(__GNUC__) && __GNUC__ >= 8
#pragma GCC diagnostic ignored "-Wstringop-truncation"
#endif
	/*
	 * coverity[buffer_size_warning : FALSE]
	 *
	 * The MAVLink spec does not require the string to be NUL-terminated if it
	 * has length 16. In this case the receiving end needs to terminate it
	 * when copying it.
	 */
	strncpy(msg.param_id, param_name(param), MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
#if defined(__GNUC__) && __GNUC__ >= 8
#pragma GCC diagnostic pop
#endif

	/* query parameter type */
	param_type_t type = param_type(param);

	/*
	 * Map onboard parameter type to MAVLink type,
	 * endianess matches (both little endian)
	 */
	if (type == PARAM_TYPE_INT32) {
		msg.param_type = MAVLINK_TYPE_INT32_T;

	} else if (type == PARAM_TYPE_FLOAT) {
		msg.param_type = MAVLINK_TYPE_FLOAT;

	} else {
		msg.param_type = MAVLINK_TYPE_FLOAT;
	}

	/* default component ID */
	if (component_id < 0) {
		mavlink_msg_param_value_send_struct(_mavlink->get_channel(), &msg);

	} else {
		// Re-pack the message with a different component ID
		mavlink_message_t mavlink_packet;
		mavlink_msg_param_value_encode_chan(mavlink_system.sysid, component_id, _mavlink->get_channel(), &mavlink_packet, &msg);
		_mavlink_resend_uart(_mavlink->get_channel(), &mavlink_packet);
	}

	return 0;
}

size_t
MavlinkParameters2Manager::packed_param_size(param_t param)
{
	// num (uint16_t)
	// name length (5 bits)
	// type (3 bits)
	// name (variable)
	// value (variable length)
	// = TOTAL

	return sizeof(packed_param_header)
	       + strlen(param_name(param))  /* string length */
	       + param_size(param) /* size of value */
	       ;
}

int
MavlinkParameters2Manager::pack_param(uint8_t *dst, uint16_t param_id, param_t param)
{
	packed_param_header hdr{};
	hdr.id = param_id;
	hdr.name_length = strlen(param_name(param));

	void *hdr_dst = dst;
	void *name_dst = dst + 4;
	void *value_dst = dst + 4 + hdr.name_length;

	const param_type_t type = param_type(param);

	if (type == PARAM_TYPE_INT32) {
		hdr.type = MAV_PARAM_TYPE_INT32;

	} else if (type == PARAM_TYPE_FLOAT) {
		hdr.type = MAV_PARAM_TYPE_REAL32;
	}

	// copy header into buffer
	memcpy(hdr_dst, &hdr, sizeof(hdr));

	// copy parameter name into buffer
	memcpy(name_dst, param_name(param), hdr.name_length);

	// copy parameter value into buffer
	param_get(param, value_dst);

	return sizeof(hdr) + hdr.name_length + param_size(param);
}

int
MavlinkParameters2Manager::unpack_test(const mavlink_packed_param_values_t &msg)
{
	uint8_t buf_pos = 0;

	for (int i = 0; i < msg.number; i++) {

		PX4_DEBUG("unpack_test: i = %d", i);

		const void *hdr_start = &msg.buffer[buf_pos];
		packed_param_header *hdr = (packed_param_header *)(hdr_start);

		PX4_DEBUG("unpack test: id: %d", hdr->id);
		PX4_DEBUG("unpack test: name_length: %d", hdr->name_length);
		PX4_DEBUG("unpack test: type: %d", (int)hdr->type);

		// parameter name
		const char *name_start = (const char *)&msg.buffer[buf_pos + 4];
		char param_name[hdr->name_length + 1] {};
		strncpy(&param_name[0], name_start, hdr->name_length);

		// parameter value
		if (hdr->type == MAV_PARAM_TYPE_INT32) {
			int32_t *v = (int32_t *)&msg.buffer[buf_pos + 4 + hdr->name_length];

			PX4_DEBUG("unpack test: value: %d", *v);

		} else if (hdr->type == MAV_PARAM_TYPE_REAL32) {
			float *f = (float *)&msg.buffer[buf_pos + 4 + hdr->name_length];

			PX4_DEBUG("unpack test: value: %.3f", (double)*f);
		}

		buf_pos += sizeof(packed_param_header) + hdr->name_length + 4;  // TODO: use proper value length
	}

	return 0;
}
