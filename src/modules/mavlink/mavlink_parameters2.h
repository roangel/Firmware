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
 * @file mavlink_parameters2.h
 * Mavlink parameters2 manager definition.
 *
 */

#pragma once

#include <parameters/param.h>

#include "mavlink_bridge_header.h"
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <drivers/drv_hrt.h>

class Mavlink;

class MavlinkParameters2Manager
{
public:
	explicit MavlinkParameters2Manager(Mavlink *mavlink);
	~MavlinkParameters2Manager() = default;

	/**
	 * Handle sending of messages. Call this regularly at a fixed frequency.
	 * @param t current time
	 */
	void send_all() { _send_all_index = 0; }

	void send(const hrt_abstime t);

	unsigned get_size();

	void handle_message(const mavlink_message_t *msg);

private:
	int	_send_all_index{-1};

	/* do not allow top copying this class */
	MavlinkParameters2Manager(MavlinkParameters2Manager &);
	MavlinkParameters2Manager &operator = (const MavlinkParameters2Manager &);

protected:
	/// send a single param if a PARAM_REQUEST_LIST is in progress
	/// @return true if a parameter was sent
	bool send_one();

	/**
	 * Handle any open param send transfer
	 */
	bool send_params();

	/**
	 * Send untransmitted params
	 */
	bool send_untransmitted();

	int send_param(param_t param, int component_id = -1);


	size_t packed_param_size(param_t param);

	int pack_param(uint8_t *dst, uint16_t param_id, param_t param);
	int unpack_test(const mavlink_packed_param_values_t &msg);

	int _mavlink_parameter_sub{-1};
	hrt_abstime _param_update_time{0};
	int _param_update_index{-1};

	Mavlink *_mavlink;
};
