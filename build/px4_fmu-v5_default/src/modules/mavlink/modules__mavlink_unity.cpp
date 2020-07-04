/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file mavlink.c
 * Define MAVLink specific parameters
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <px4_config.h>
#include <unistd.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "mavlink_bridge_header.h"
#include <parameters/param.h>

mavlink_system_t mavlink_system = {
	1,
	1
}; // System ID, 1-255, Component/Subsystem ID, 1-255
/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file mavlink_command_sender.cpp
 * Mavlink commands sender with support for retransmission.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#include "mavlink_command_sender.h"
#include <px4_log.h>

#define CMD_DEBUG(FMT, ...) PX4_LOG_NAMED_COND("cmd sender", _debug_enabled, FMT, ##__VA_ARGS__)

MavlinkCommandSender *MavlinkCommandSender::_instance = nullptr;
px4_sem_t MavlinkCommandSender::_lock;

void MavlinkCommandSender::initialize()
{
	px4_sem_init(&_lock, 1, 1);

	if (_instance == nullptr) {
		_instance = new MavlinkCommandSender();
	}
}

MavlinkCommandSender &MavlinkCommandSender::instance()
{
	return *_instance;
}

MavlinkCommandSender::~MavlinkCommandSender()
{
	px4_sem_destroy(&_lock);
}

int MavlinkCommandSender::handle_vehicle_command(const struct vehicle_command_s &command, mavlink_channel_t channel)
{
	lock();
	CMD_DEBUG("new command: %d (channel: %d)", command.command, channel);

	mavlink_command_long_t msg = {};
	msg.target_system = command.target_system;
	msg.target_component = command.target_component;
	msg.command = command.command;
	msg.confirmation = command.confirmation;
	msg.param1 = command.param1;
	msg.param2 = command.param2;
	msg.param3 = command.param3;
	msg.param4 = command.param4;
	msg.param5 = command.param5;
	msg.param6 = command.param6;
	msg.param7 = command.param7;
	mavlink_msg_command_long_send_struct(channel, &msg);

	bool already_existing = false;
	_commands.reset_to_start();

	while (command_item_t *item = _commands.get_next()) {
		if (item->timestamp_us == command.timestamp) {

			// We should activate the channel by setting num_sent_per_channel from -1 to 0.
			item->num_sent_per_channel[channel] = 0;
			already_existing = true;
			break;
		}
	}

	if (!already_existing) {

		command_item_t new_item;
		new_item.command = msg;
		new_item.timestamp_us = command.timestamp;
		new_item.num_sent_per_channel[channel] = 0;
		new_item.last_time_sent_us = hrt_absolute_time();
		_commands.put(new_item);
	}

	unlock();
	return 0;
}

void MavlinkCommandSender::handle_mavlink_command_ack(const mavlink_command_ack_t &ack,
		uint8_t from_sysid, uint8_t from_compid)
{
	CMD_DEBUG("handling result %d for command %d (from %d:%d)",
		  ack.result, ack.command, from_sysid, from_compid);
	lock();

	_commands.reset_to_start();

	while (command_item_t *item = _commands.get_next()) {
		// Check if the incoming ack matches any of the commands that we have sent.
		if (item->command.command == ack.command &&
		    from_sysid == item->command.target_system &&
		    from_compid == item->command.target_component) {
			// Drop it anyway because the command seems to have arrived at the destination, even if we
			// receive IN_PROGRESS because we trust that it will be handled after that.
			_commands.drop_current();
			break;
		}
	}

	unlock();
}

void MavlinkCommandSender::check_timeout(mavlink_channel_t channel)
{
	lock();

	_commands.reset_to_start();

	while (command_item_t *item = _commands.get_next()) {
		if (hrt_elapsed_time(&item->last_time_sent_us) <= TIMEOUT_US) {
			// We keep waiting for the timeout.
			continue;
		}

		// The goal of this is to retry from all channels. Therefore, we keep
		// track of the retry count for each channel.
		//
		// When the first channel does a retry, the timeout is reset.
		// (e.g. all channel have done 2 retries, then channel 0 is called
		// and does retry number 3, and also resets the timeout timestamp).

		// First, we need to determine what the current max and min retry level
		// are because we can only level up, if all have caught up.
		// If num_sent_per_channel is at -1, the channel is inactive.
		int8_t max_sent = 0;
		int8_t min_sent = INT8_MAX;

		for (unsigned i = 0; i < MAX_MAVLINK_CHANNEL; ++i) {
			if (item->num_sent_per_channel[i] > max_sent) {
				max_sent = item->num_sent_per_channel[i];
			}

			if ((item->num_sent_per_channel[i] != -1) &&
			    (item->num_sent_per_channel[i] < min_sent)) {
				min_sent = item->num_sent_per_channel[i];
			}
		}

		if (item->num_sent_per_channel[channel] < max_sent) {
			// We are behind and need to do a retransmission.
			mavlink_msg_command_long_send_struct(channel, &item->command);
			item->num_sent_per_channel[channel]++;

			CMD_DEBUG("command %d sent (not first, retries: %d/%d, channel: %d)",
				  item->command.command,
				  item->num_sent_per_channel[channel],
				  max_sent,
				  channel);

		} else if (item->num_sent_per_channel[channel] == max_sent &&
			   min_sent == max_sent) {

			// If the next retry would be above the needed retries anyway, we can
			// drop the item, and continue with other items.
			if (item->num_sent_per_channel[channel] + 1 > RETRIES) {
				CMD_DEBUG("command %d dropped", item->command.command);
				_commands.drop_current();
				continue;
			}

			// We are the first of a new retransmission series.
			mavlink_msg_command_long_send_struct(channel, &item->command);
			item->num_sent_per_channel[channel]++;
			// Therefore, we are the ones setting the timestamp of this retry round.
			item->last_time_sent_us = hrt_absolute_time();

			CMD_DEBUG("command %d sent (first, retries: %d/%d, channel: %d)",
				  item->command.command,
				  item->num_sent_per_channel[channel],
				  max_sent,
				  channel);

		} else {
			// We are already ahead, so this should not happen.
			// If it ever does, just ignore it. It will timeout eventually.
			continue;
		}
	}

	unlock();
}
/****************************************************************************
 *
 *   Copyright (c) 2014-2016 PX4 Development Team. All rights reserved.
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

/// @file mavlink_ftp.cpp
///	@author px4dev, Don Gagne <don@thegagnes.com>

#include <crc32.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>
#include <cstring>

#include "mavlink_ftp.h"
#include "mavlink_main.h"
#include "mavlink_tests/mavlink_ftp_test.h"

constexpr const char MavlinkFTP::_root_dir[];

MavlinkFTP::MavlinkFTP(Mavlink *mavlink) :
	_mavlink(mavlink)
{
	// initialize session
	_session_info.fd = -1;
}

MavlinkFTP::~MavlinkFTP()
{
	if (_work_buffer1) {
		delete[] _work_buffer1;
	}

	if (_work_buffer2) {
		delete[] _work_buffer2;
	}
}

unsigned
MavlinkFTP::get_size()
{
	if (_session_info.stream_download) {
		return MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;

	} else {
		return 0;
	}
}

#ifdef MAVLINK_FTP_UNIT_TEST
void
MavlinkFTP::set_unittest_worker(ReceiveMessageFunc_t rcvMsgFunc, void *worker_data)
{
	_utRcvMsgFunc = rcvMsgFunc;
	_worker_data = worker_data;
}
#endif

uint8_t
MavlinkFTP::_getServerSystemId()
{
#ifdef MAVLINK_FTP_UNIT_TEST
	// We use fake ids when unit testing
	return MavlinkFtpTest::serverSystemId;
#else
	// Not unit testing, use the real thing
	return _mavlink->get_system_id();
#endif
}

uint8_t
MavlinkFTP::_getServerComponentId()
{
#ifdef MAVLINK_FTP_UNIT_TEST
	// We use fake ids when unit testing
	return MavlinkFtpTest::serverComponentId;
#else
	// Not unit testing, use the real thing
	return _mavlink->get_component_id();
#endif
}

uint8_t
MavlinkFTP::_getServerChannel()
{
#ifdef MAVLINK_FTP_UNIT_TEST
	// We use fake ids when unit testing
	return MavlinkFtpTest::serverChannel;
#else
	// Not unit testing, use the real thing
	return _mavlink->get_channel();
#endif
}

void
MavlinkFTP::handle_message(const mavlink_message_t *msg)
{
	//warnx("MavlinkFTP::handle_message %d %d", buf_size_1, buf_size_2);

	if (msg->msgid == MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL) {
		mavlink_file_transfer_protocol_t ftp_request;
		mavlink_msg_file_transfer_protocol_decode(msg, &ftp_request);

#ifdef MAVLINK_FTP_DEBUG
		PX4_INFO("FTP: received ftp protocol message target_system: %d", ftp_request.target_system);
#endif

		if (ftp_request.target_system == _getServerSystemId()) {
			_process_request(&ftp_request, msg->sysid);
		}
	}
}

/// @brief Processes an FTP message
void
MavlinkFTP::_process_request(mavlink_file_transfer_protocol_t *ftp_req, uint8_t target_system_id)
{
	bool stream_send = false;
	PayloadHeader *payload = reinterpret_cast<PayloadHeader *>(&ftp_req->payload[0]);

	ErrorCode errorCode = kErrNone;

	if (!_ensure_buffers_exist()) {
		PX4_ERR("Failed to allocate buffers");
		errorCode = kErrFailErrno;
		errno = ENOMEM;
		goto out;
	}

	// basic sanity checks; must validate length before use
	if (payload->size > kMaxDataLength) {
		errorCode = kErrInvalidDataSize;
		goto out;
	}

	// check the sequence number: if this is a resent request, resend the last response
	if (_last_reply_valid) {
		mavlink_file_transfer_protocol_t *last_reply = reinterpret_cast<mavlink_file_transfer_protocol_t *>(_last_reply);
		PayloadHeader *last_payload = reinterpret_cast<PayloadHeader *>(&last_reply->payload[0]);

		if (payload->seq_number + 1 == last_payload->seq_number) {
			// this is the same request as the one we replied to last. It means the (n)ack got lost, and the GCS
			// resent the request
			mavlink_msg_file_transfer_protocol_send_struct(_mavlink->get_channel(), last_reply);
			return;
		}
	}



#ifdef MAVLINK_FTP_DEBUG
	PX4_INFO("ftp: channel %u opc %u size %u offset %u", _getServerChannel(), payload->opcode, payload->size,
		 payload->offset);
#endif

	switch (payload->opcode) {
	case kCmdNone:
		break;

	case kCmdTerminateSession:
		errorCode = _workTerminate(payload);
		break;

	case kCmdResetSessions:
		errorCode = _workReset(payload);
		break;

	case kCmdListDirectory:
		errorCode = _workList(payload);
		break;

	case kCmdOpenFileRO:
		errorCode = _workOpen(payload, O_RDONLY);
		break;

	case kCmdCreateFile:
		errorCode = _workOpen(payload, O_CREAT | O_EXCL | O_WRONLY);
		break;

	case kCmdOpenFileWO:
		errorCode = _workOpen(payload, O_CREAT | O_WRONLY);
		break;

	case kCmdReadFile:
		errorCode = _workRead(payload);
		break;

	case kCmdBurstReadFile:
		errorCode = _workBurst(payload, target_system_id);
		stream_send = true;
		break;

	case kCmdWriteFile:
		errorCode = _workWrite(payload);
		break;

	case kCmdRemoveFile:
		errorCode = _workRemoveFile(payload);
		break;

	case kCmdRename:
		errorCode = _workRename(payload);
		break;

	case kCmdTruncateFile:
		errorCode = _workTruncateFile(payload);
		break;

	case kCmdCreateDirectory:
		errorCode = _workCreateDirectory(payload);
		break;

	case kCmdRemoveDirectory:
		errorCode = _workRemoveDirectory(payload);
		break;

	case kCmdCalcFileCRC32:
		errorCode = _workCalcFileCRC32(payload);
		break;

	default:
		errorCode = kErrUnknownCommand;
		break;
	}

out:
	payload->seq_number++;

	// handle success vs. error
	if (errorCode == kErrNone) {
		payload->req_opcode = payload->opcode;
		payload->opcode = kRspAck;

	} else {
		int r_errno = errno;
		payload->req_opcode = payload->opcode;
		payload->opcode = kRspNak;
		payload->size = 1;

		if (r_errno == EEXIST) {
			errorCode = kErrFailFileExists;
		}

		payload->data[0] = errorCode;


		if (errorCode == kErrFailErrno) {
			payload->size = 2;
			payload->data[1] = r_errno;
		}
	}

	_last_reply_valid = false;

	// Stream download replies are sent through mavlink stream mechanism. Unless we need to Nack.
	if (!stream_send || errorCode != kErrNone) {
		// respond to the request
		ftp_req->target_system = target_system_id;
		_reply(ftp_req);
	}
}

bool MavlinkFTP::_ensure_buffers_exist()
{
	_last_work_buffer_access = hrt_absolute_time();

	if (!_work_buffer1) {
		_work_buffer1 = new char[_work_buffer1_len];
	}

	if (!_work_buffer2) {
		_work_buffer2 = new char[_work_buffer2_len];
	}

	return _work_buffer1 && _work_buffer2;
}

/// @brief Sends the specified FTP response message out through mavlink
void
MavlinkFTP::_reply(mavlink_file_transfer_protocol_t *ftp_req)
{

	PayloadHeader *payload = reinterpret_cast<PayloadHeader *>(&ftp_req->payload[0]);

	// keep a copy of the last sent response ((n)ack), so that if it gets lost and the GCS resends the request,
	// we can simply resend the response.
	// we only keep small responses to reduce RAM usage and avoid large memcpy's. The larger responses are all data
	// retrievals without side-effects, meaning it's ok to reexecute them if a response gets lost
	if (payload->size <= sizeof(uint32_t)) {
		_last_reply_valid = true;
		memcpy(_last_reply, ftp_req, sizeof(_last_reply));
	}

#ifdef MAVLINK_FTP_DEBUG
	PX4_INFO("FTP: %s seq_number: %d", payload->opcode == kRspAck ? "Ack" : "Nak", payload->seq_number);
#endif

	ftp_req->target_network = 0;
	ftp_req->target_component = 0;
#ifdef MAVLINK_FTP_UNIT_TEST
	// Unit test hook is set, call that instead
	_utRcvMsgFunc(ftp_req, _worker_data);
#else
	mavlink_msg_file_transfer_protocol_send_struct(_mavlink->get_channel(), ftp_req);
#endif

}

/// @brief Responds to a List command
MavlinkFTP::ErrorCode
MavlinkFTP::_workList(PayloadHeader *payload, bool list_hidden)
{
	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, _data_as_cstring(payload), _work_buffer1_len - _root_dir_len);
	// ensure termination
	_work_buffer1[_work_buffer1_len - 1] = '\0';

	ErrorCode errorCode = kErrNone;
	unsigned offset = 0;

	DIR *dp = opendir(_work_buffer1);

	if (dp == nullptr) {
#ifdef MAVLINK_FTP_UNIT_TEST
		PX4_WARN("File open failed %s", _work_buffer1);
#else
		_mavlink->send_statustext_critical("FTP: can't open path (file system corrupted?)");
		_mavlink->send_statustext_critical(_work_buffer1);
#endif
		// this is not an FTP error, abort directory by simulating eof
		return kErrEOF;
	}

#ifdef MAVLINK_FTP_DEBUG
	PX4_INFO("FTP: list %s offset %d", _work_buffer1, payload->offset);
#endif

	struct dirent *result = nullptr;

	// move to the requested offset
	int requested_offset = payload->offset;

	while (requested_offset-- > 0 && readdir(dp));

	for (;;) {
		errno = 0;
		result = readdir(dp);

		// read the directory entry
		if (result == nullptr) {
			if (errno) {
#ifdef MAVLINK_FTP_UNIT_TEST
				PX4_WARN("readdir failed");
#else
				_mavlink->send_statustext_critical("FTP: list readdir failure");
				_mavlink->send_statustext_critical(_work_buffer1);
#endif

				payload->data[offset++] = kDirentSkip;
				*((char *)&payload->data[offset]) = '\0';
				offset++;
				payload->size = offset;
				closedir(dp);

				return errorCode;
			}

			// no more entries?
			if (payload->offset != 0 && offset == 0) {
				// User is requesting subsequent dir entries but there were none. This means the user asked
				// to seek past EOF.
				errorCode = kErrEOF;
			}

			// Otherwise we are just at the last directory entry, so we leave the errorCode at kErrorNone to signal that
			break;
		}

		uint32_t fileSize = 0;
		char direntType;

		// Determine the directory entry type
		switch (result->d_type) {
#ifdef __PX4_NUTTX

		case DTYPE_FILE: {
#else

		case DT_REG: {
#endif
				// For files we get the file size as well
				direntType = kDirentFile;
				int ret = snprintf(_work_buffer2, _work_buffer2_len, "%s/%s", _work_buffer1, result->d_name);
				bool buf_is_ok = ((ret > 0) && (ret < _work_buffer2_len));

				if (buf_is_ok) {
					struct stat st;

					if (stat(_work_buffer2, &st) == 0) {
						fileSize = st.st_size;
					}
				}

				break;
			}

#ifdef __PX4_NUTTX

		case DTYPE_DIRECTORY:
#else
		case DT_DIR:
#endif
			if ((!list_hidden && (strncmp(result->d_name, ".", 1) == 0)) ||
			    strcmp(result->d_name, ".") == 0 || strcmp(result->d_name, "..") == 0) {
				// Don't bother sending these back
				direntType = kDirentSkip;

			} else {
				direntType = kDirentDir;
			}

			break;

		default:
			// We only send back file and diretory entries, skip everything else
			direntType = kDirentSkip;
		}

		if (direntType == kDirentSkip) {
			// Skip send only dirent identifier
			_work_buffer2[0] = '\0';

		} else if (direntType == kDirentFile) {
			// Files send filename and file length
			int ret = snprintf(_work_buffer2, _work_buffer2_len, "%s\t%d", result->d_name, fileSize);
			bool buf_is_ok = ((ret > 0) && (ret < _work_buffer2_len));

			if (!buf_is_ok) {
				_work_buffer2[_work_buffer2_len - 1] = '\0';
			}

		} else {
			// Everything else just sends name
			strncpy(_work_buffer2, result->d_name, _work_buffer2_len);
			_work_buffer2[_work_buffer2_len - 1] = '\0';
		}

		size_t nameLen = strlen(_work_buffer2);

		// Do we have room for the name, the one char directory identifier and the null terminator?
		if ((offset + nameLen + 2) > kMaxDataLength) {
			break;
		}

		// Move the data into the buffer
		payload->data[offset++] = direntType;
		strcpy((char *)&payload->data[offset], _work_buffer2);
#ifdef MAVLINK_FTP_DEBUG
		PX4_INFO("FTP: list %s %s", _work_buffer1, (char *)&payload->data[offset - 1]);
#endif
		offset += nameLen + 1;
	}

	closedir(dp);
	payload->size = offset;

	return errorCode;
}

/// @brief Responds to an Open command
MavlinkFTP::ErrorCode
MavlinkFTP::_workOpen(PayloadHeader *payload, int oflag)
{
	if (_session_info.fd >= 0) {
		PX4_ERR("FTP: Open failed - out of sessions\n");
		return kErrNoSessionsAvailable;
	}

	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, _data_as_cstring(payload), _work_buffer1_len - _root_dir_len);

#ifdef MAVLINK_FTP_DEBUG
	PX4_INFO("FTP: open '%s'", _work_buffer1);
#endif

	uint32_t fileSize = 0;
	struct stat st;

	if (stat(_work_buffer1, &st) != 0) {
		// fail only if requested open for read
		if (oflag & O_RDONLY) {
			return kErrFailErrno;

		} else {
			st.st_size = 0;
		}
	}

	fileSize = st.st_size;

	// Set mode to 666 incase oflag has O_CREAT
	int fd = ::open(_work_buffer1, oflag, PX4_O_MODE_666);

	if (fd < 0) {
		return kErrFailErrno;
	}

	_session_info.fd = fd;
	_session_info.file_size = fileSize;
	_session_info.stream_download = false;

	payload->session = 0;
	payload->size = sizeof(uint32_t);
	std::memcpy(payload->data, &fileSize, payload->size);

	return kErrNone;
}

/// @brief Responds to a Read command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRead(PayloadHeader *payload)
{
	if (payload->session != 0 || _session_info.fd < 0) {
		return kErrInvalidSession;
	}

#ifdef MAVLINK_FTP_DEBUG
	PX4_INFO("FTP: read offset:%d", payload->offset);
#endif

	// We have to test seek past EOF ourselves, lseek will allow seek past EOF
	if (payload->offset >= _session_info.file_size) {
		PX4_ERR("request past EOF");
		return kErrEOF;
	}

	if (lseek(_session_info.fd, payload->offset, SEEK_SET) < 0) {
		PX4_ERR("seek fail");
		return kErrFailErrno;
	}

	int bytes_read = ::read(_session_info.fd, &payload->data[0], kMaxDataLength);

	if (bytes_read < 0) {
		// Negative return indicates error other than eof
		PX4_ERR("read fail %d", bytes_read);
		return kErrFailErrno;
	}

	payload->size = bytes_read;

	return kErrNone;
}

/// @brief Responds to a Stream command
MavlinkFTP::ErrorCode
MavlinkFTP::_workBurst(PayloadHeader *payload, uint8_t target_system_id)
{
	if (payload->session != 0 && _session_info.fd < 0) {
		return kErrInvalidSession;
	}

#ifdef MAVLINK_FTP_DEBUG
	PX4_INFO("FTP: burst offset:%d", payload->offset);
#endif
	// Setup for streaming sends
	_session_info.stream_download = true;
	_session_info.stream_offset = payload->offset;
	_session_info.stream_chunk_transmitted = 0;
	_session_info.stream_seq_number = payload->seq_number + 1;
	_session_info.stream_target_system_id = target_system_id;

	return kErrNone;
}

/// @brief Responds to a Write command
MavlinkFTP::ErrorCode
MavlinkFTP::_workWrite(PayloadHeader *payload)
{
	if (payload->session != 0 && _session_info.fd < 0) {
		return kErrInvalidSession;
	}

	if (lseek(_session_info.fd, payload->offset, SEEK_SET) < 0) {
		// Unable to see to the specified location
		PX4_ERR("seek fail");
		return kErrFailErrno;
	}

	int bytes_written = ::write(_session_info.fd, &payload->data[0], payload->size);

	if (bytes_written < 0) {
		// Negative return indicates error other than eof
		PX4_ERR("write fail %d", bytes_written);
		return kErrFailErrno;
	}

	payload->size = sizeof(uint32_t);
	std::memcpy(payload->data, &bytes_written, payload->size);

	return kErrNone;
}

/// @brief Responds to a RemoveFile command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRemoveFile(PayloadHeader *payload)
{
	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, _data_as_cstring(payload), _work_buffer1_len - _root_dir_len);
	// ensure termination
	_work_buffer1[_work_buffer1_len - 1] = '\0';

	if (unlink(_work_buffer1) == 0) {
		payload->size = 0;
		return kErrNone;

	} else {
		return kErrFailErrno;
	}
}

/// @brief Responds to a TruncateFile command
MavlinkFTP::ErrorCode
MavlinkFTP::_workTruncateFile(PayloadHeader *payload)
{
	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, _data_as_cstring(payload), _work_buffer1_len - _root_dir_len);
	// ensure termination
	_work_buffer1[_work_buffer1_len - 1] = '\0';
	payload->size = 0;

#ifdef __PX4_NUTTX

	// emulate truncate(_work_buffer1, payload->offset) by
	// copying to temp and overwrite with O_TRUNC flag (NuttX does not support truncate()).
	const char temp_file[] = PX4_STORAGEDIR"/.trunc.tmp";

	struct stat st;

	if (stat(_work_buffer1, &st) != 0) {
		return kErrFailErrno;
	}

	if (!S_ISREG(st.st_mode)) {
		errno = EISDIR;
		return kErrFailErrno;
	}

	// check perms allow us to write (not romfs)
	if (!(st.st_mode & (S_IWUSR | S_IWGRP | S_IWOTH))) {
		errno = EROFS;
		return kErrFailErrno;
	}

	if (payload->offset == (unsigned)st.st_size) {
		// nothing to do
		return kErrNone;

	} else if (payload->offset == 0) {
		// 1: truncate all data
		int fd = ::open(_work_buffer1, O_TRUNC | O_WRONLY);

		if (fd < 0) {
			return kErrFailErrno;
		}

		::close(fd);
		return kErrNone;

	} else if (payload->offset > (unsigned)st.st_size) {
		// 2: extend file
		int fd = ::open(_work_buffer1, O_WRONLY);

		if (fd < 0) {
			return kErrFailErrno;
		}

		if (lseek(fd, payload->offset - 1, SEEK_SET) < 0) {
			::close(fd);
			return kErrFailErrno;
		}

		bool ok = 1 == ::write(fd, "", 1);
		::close(fd);

		return (ok) ? kErrNone : kErrFailErrno;

	} else {
		// 3: truncate
		if (_copy_file(_work_buffer1, temp_file, payload->offset) != 0) {
			return kErrFailErrno;
		}

		if (_copy_file(temp_file, _work_buffer1, payload->offset) != 0) {
			return kErrFailErrno;
		}

		if (::unlink(temp_file) != 0) {
			return kErrFailErrno;
		}

		return kErrNone;
	}

#else
	int ret = truncate(_work_buffer1, payload->offset);

	if (ret == 0) {
		return kErrNone;
	}

	return kErrFailErrno;
#endif /* __PX4_NUTTX */
}

/// @brief Responds to a Terminate command
MavlinkFTP::ErrorCode
MavlinkFTP::_workTerminate(PayloadHeader *payload)
{
	if (payload->session != 0 || _session_info.fd < 0) {
		return kErrInvalidSession;
	}

	::close(_session_info.fd);
	_session_info.fd = -1;
	_session_info.stream_download = false;

	payload->size = 0;

	return kErrNone;
}

/// @brief Responds to a Reset command
MavlinkFTP::ErrorCode
MavlinkFTP::_workReset(PayloadHeader *payload)
{
	if (_session_info.fd != -1) {
		::close(_session_info.fd);
		_session_info.fd = -1;
		_session_info.stream_download = false;
	}

	payload->size = 0;

	return kErrNone;
}

/// @brief Responds to a Rename command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRename(PayloadHeader *payload)
{
	char *ptr = _data_as_cstring(payload);
	size_t oldpath_sz = strlen(ptr);

	if (oldpath_sz == payload->size) {
		// no newpath
		errno = EINVAL;
		return kErrFailErrno;
	}

	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, ptr, _work_buffer1_len - _root_dir_len);
	_work_buffer1[_work_buffer1_len - 1] = '\0'; // ensure termination

	strncpy(_work_buffer2, _root_dir, _work_buffer2_len);
	strncpy(_work_buffer2 + _root_dir_len, ptr + oldpath_sz + 1, _work_buffer2_len - _root_dir_len);
	_work_buffer2[_work_buffer2_len - 1] = '\0'; // ensure termination

	if (rename(_work_buffer1, _work_buffer2) == 0) {
		payload->size = 0;
		return kErrNone;

	} else {
		return kErrFailErrno;
	}
}

/// @brief Responds to a RemoveDirectory command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRemoveDirectory(PayloadHeader *payload)
{
	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, _data_as_cstring(payload), _work_buffer1_len - _root_dir_len);
	// ensure termination
	_work_buffer1[_work_buffer1_len - 1] = '\0';

	if (rmdir(_work_buffer1) == 0) {
		payload->size = 0;
		return kErrNone;

	} else {
		return kErrFailErrno;
	}
}

/// @brief Responds to a CreateDirectory command
MavlinkFTP::ErrorCode
MavlinkFTP::_workCreateDirectory(PayloadHeader *payload)
{
	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, _data_as_cstring(payload), _work_buffer1_len - _root_dir_len);
	// ensure termination
	_work_buffer1[_work_buffer1_len - 1] = '\0';

	if (mkdir(_work_buffer1, S_IRWXU | S_IRWXG | S_IRWXO) == 0) {
		payload->size = 0;
		return kErrNone;

	} else {
		return kErrFailErrno;
	}
}

/// @brief Responds to a CalcFileCRC32 command
MavlinkFTP::ErrorCode
MavlinkFTP::_workCalcFileCRC32(PayloadHeader *payload)
{
	uint32_t checksum = 0;
	ssize_t bytes_read;
	strncpy(_work_buffer2, _root_dir, _work_buffer2_len);
	strncpy(_work_buffer2 + _root_dir_len, _data_as_cstring(payload), _work_buffer2_len - _root_dir_len);
	// ensure termination
	_work_buffer2[_work_buffer2_len - 1] = '\0';

	int fd = ::open(_work_buffer2, O_RDONLY);

	if (fd < 0) {
		return kErrFailErrno;
	}

	do {
		bytes_read = ::read(fd, _work_buffer2, _work_buffer2_len);

		if (bytes_read < 0) {
			int r_errno = errno;
			::close(fd);
			errno = r_errno;
			return kErrFailErrno;
		}

		checksum = crc32part((uint8_t *)_work_buffer2, bytes_read, checksum);
	} while (bytes_read == _work_buffer2_len);

	::close(fd);

	payload->size = sizeof(uint32_t);
	std::memcpy(payload->data, &checksum, payload->size);
	return kErrNone;
}

/// @brief Guarantees that the payload data is null terminated.
///     @return Returns a pointer to the payload data as a char *
char *
MavlinkFTP::_data_as_cstring(PayloadHeader *payload)
{
	// guarantee nul termination
	if (payload->size < kMaxDataLength) {
		payload->data[payload->size] = '\0';

	} else {
		payload->data[kMaxDataLength - 1] = '\0';
	}

	// and return data
	return (char *) & (payload->data[0]);
}

/// @brief Copy file (with limited space)
int
MavlinkFTP::_copy_file(const char *src_path, const char *dst_path, size_t length)
{
	int src_fd = -1, dst_fd = -1;
	int op_errno = 0;

	src_fd = ::open(src_path, O_RDONLY);

	if (src_fd < 0) {
		return -1;
	}

	dst_fd = ::open(dst_path, O_CREAT | O_TRUNC | O_WRONLY
// POSIX requires the permissions to be supplied if O_CREAT passed
#ifdef __PX4_POSIX
			, 0666
#endif
		       );

	if (dst_fd < 0) {
		op_errno = errno;
		::close(src_fd);
		errno = op_errno;
		return -1;
	}

	while (length > 0) {
		ssize_t bytes_read, bytes_written;
		size_t blen = (length > _work_buffer2_len) ? _work_buffer2_len : length;

		bytes_read = ::read(src_fd, _work_buffer2, blen);

		if (bytes_read == 0) {
			// EOF
			break;

		} else if (bytes_read < 0) {
			PX4_ERR("cp: read");
			op_errno = errno;
			break;
		}

		bytes_written = ::write(dst_fd, _work_buffer2, bytes_read);

		if (bytes_written != bytes_read) {
			PX4_ERR("cp: short write");
			op_errno = errno;
			break;
		}

		length -= bytes_written;
	}

	::close(src_fd);
	::close(dst_fd);

	errno = op_errno;
	return (length > 0) ? -1 : 0;
}

void MavlinkFTP::send(const hrt_abstime t)
{

	if (_work_buffer1 || _work_buffer2) {
		// free the work buffers if they are not used for a while
		if (hrt_elapsed_time(&_last_work_buffer_access) > 2000000) {
			if (_work_buffer1) {
				delete[] _work_buffer1;
				_work_buffer1 = nullptr;
			}

			if (_work_buffer2) {
				delete[] _work_buffer2;
				_work_buffer2 = nullptr;
			}
		}
	}

	// Anything to stream?
	if (!_session_info.stream_download) {
		return;
	}

#ifndef MAVLINK_FTP_UNIT_TEST
	// Skip send if not enough room
	unsigned max_bytes_to_send = _mavlink->get_free_tx_buf();
#ifdef MAVLINK_FTP_DEBUG
	PX4_INFO("MavlinkFTP::send max_bytes_to_send(%d) get_free_tx_buf(%d)", max_bytes_to_send, _mavlink->get_free_tx_buf());
#endif

	if (max_bytes_to_send < get_size()) {
		return;
	}

#endif

	// Send stream packets until buffer is full

	bool more_data;

	do {
		more_data = false;

		ErrorCode error_code = kErrNone;

		mavlink_file_transfer_protocol_t ftp_msg;
		PayloadHeader *payload = reinterpret_cast<PayloadHeader *>(&ftp_msg.payload[0]);

		payload->seq_number = _session_info.stream_seq_number;
		payload->session = 0;
		payload->opcode = kRspAck;
		payload->req_opcode = kCmdBurstReadFile;
		payload->offset = _session_info.stream_offset;
		_session_info.stream_seq_number++;

#ifdef MAVLINK_FTP_DEBUG
		PX4_INFO("stream send: offset %d", _session_info.stream_offset);
#endif

		// We have to test seek past EOF ourselves, lseek will allow seek past EOF
		if (_session_info.stream_offset >= _session_info.file_size) {
			error_code = kErrEOF;
#ifdef MAVLINK_FTP_DEBUG
			PX4_INFO("stream download: sending Nak EOF");
#endif
		}

		if (error_code == kErrNone) {
			if (lseek(_session_info.fd, payload->offset, SEEK_SET) < 0) {
				error_code = kErrFailErrno;
#ifdef MAVLINK_FTP_DEBUG
				PX4_WARN("stream download: seek fail");
#endif
			}
		}

		if (error_code == kErrNone) {
			int bytes_read = ::read(_session_info.fd, &payload->data[0], kMaxDataLength);

			if (bytes_read < 0) {
				// Negative return indicates error other than eof
				error_code = kErrFailErrno;
#ifdef MAVLINK_FTP_DEBUG
				PX4_WARN("stream download: read fail");
#endif

			} else {
				payload->size = bytes_read;
				_session_info.stream_offset += bytes_read;
				_session_info.stream_chunk_transmitted += bytes_read;
			}
		}

		if (error_code != kErrNone) {
			payload->opcode = kRspNak;
			payload->size = 1;
			uint8_t *pData = &payload->data[0];
			*pData = error_code; // Straight reference to data[0] is causing bogus gcc array subscript error

			if (error_code == kErrFailErrno) {
				int r_errno = errno;
				payload->size = 2;
				payload->data[1] = r_errno;
			}

			_session_info.stream_download = false;

		} else {
#ifndef MAVLINK_FTP_UNIT_TEST

			if (max_bytes_to_send < (get_size() * 2)) {
				more_data = false;

				/* perform transfers in 35K chunks - this is determined empirical */
				if (_session_info.stream_chunk_transmitted > 35000) {
					payload->burst_complete = true;
					_session_info.stream_download = false;
					_session_info.stream_chunk_transmitted = 0;
				}

			} else {
#endif
				more_data = true;
				payload->burst_complete = false;
#ifndef MAVLINK_FTP_UNIT_TEST
				max_bytes_to_send -= get_size();
			}

#endif
		}

		ftp_msg.target_system = _session_info.stream_target_system_id;
		_reply(&ftp_msg);
	} while (more_data);
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_high_latency2.cpp
 *
 * @author Achermann Florian <acfloria@ethz.ch>
 */

#include "mavlink_high_latency2.h"

#include <commander/px4_custom_mode.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/uORB.h>

using matrix::wrap_2pi;

MavlinkStreamHighLatency2::MavlinkStreamHighLatency2(Mavlink *mavlink) : MavlinkStream(mavlink),
	_actuator_sub_0(_mavlink->add_orb_subscription(ORB_ID(actuator_controls_0))),
	_actuator_time_0(0),
	_actuator_sub_1(_mavlink->add_orb_subscription(ORB_ID(actuator_controls_1))),
	_actuator_time_1(0),
	_airspeed_sub(_mavlink->add_orb_subscription(ORB_ID(airspeed))),
	_airspeed_time(0),
	_attitude_sp_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_attitude_setpoint))),
	_attitude_sp_time(0),
	_battery_sub(_mavlink->add_orb_subscription(ORB_ID(battery_status))),
	_battery_time(0),
	_estimator_status_sub(_mavlink->add_orb_subscription(ORB_ID(estimator_status))),
	_estimator_status_time(0),
	_pos_ctrl_status_sub(_mavlink->add_orb_subscription(ORB_ID(position_controller_status))),
	_pos_ctrl_status_time(0),
	_geofence_sub(_mavlink->add_orb_subscription(ORB_ID(geofence_result))),
	_geofence_time(0),
	_global_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_global_position))),
	_global_pos_time(0),
	_gps_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_gps_position))),
	_gps_time(0),
	_mission_result_sub(_mavlink->add_orb_subscription(ORB_ID(mission_result))),
	_mission_result_time(0),
	_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status))),
	_status_time(0),
	_status_flags_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status_flags))),
	_status_flags_time(0),
	_tecs_status_sub(_mavlink->add_orb_subscription(ORB_ID(tecs_status))),
	_tecs_time(0),
	_wind_sub(_mavlink->add_orb_subscription(ORB_ID(wind_estimate))),
	_wind_time(0),
	_airspeed(SimpleAnalyzer::AVERAGE),
	_airspeed_sp(SimpleAnalyzer::AVERAGE),
	_battery(SimpleAnalyzer::AVERAGE),
	_climb_rate(SimpleAnalyzer::MAX),
	_eph(SimpleAnalyzer::MAX),
	_epv(SimpleAnalyzer::MAX),
	_groundspeed(SimpleAnalyzer::AVERAGE),
	_temperature(SimpleAnalyzer::AVERAGE),
	_throttle(SimpleAnalyzer::AVERAGE),
	_windspeed(SimpleAnalyzer::AVERAGE)
{
	reset_last_sent();
}

bool MavlinkStreamHighLatency2::send(const hrt_abstime t)
{
	// only send the struct if transmitting is allowed
	// this assures that the stream timer is only reset when actually a message is transmitted
	if (_mavlink->should_transmit()) {
		mavlink_high_latency2_t msg = {};
		set_default_values(msg);

		bool updated = _airspeed.valid();
		updated |= _airspeed_sp.valid();
		updated |= _battery.valid();
		updated |= _climb_rate.valid();
		updated |= _eph.valid();
		updated |= _epv.valid();
		updated |= _groundspeed.valid();
		updated |= _temperature.valid();
		updated |= _throttle.valid();
		updated |= _windspeed.valid();
		updated |= write_airspeed(&msg);
		updated |= write_attitude_sp(&msg);
		updated |= write_battery_status(&msg);
		updated |= write_estimator_status(&msg);
		updated |= write_fw_ctrl_status(&msg);
		updated |= write_geofence_result(&msg);
		updated |= write_global_position(&msg);
		updated |= write_mission_result(&msg);
		updated |= write_tecs_status(&msg);
		updated |= write_vehicle_status(&msg);
		updated |= write_vehicle_status_flags(&msg);
		updated |= write_wind_estimate(&msg);

		if (updated) {
			uint32_t timestamp;
			convert_limit_safe(t / 1000, timestamp);
			msg.timestamp = timestamp;

			msg.type = _mavlink->get_system_type();
			msg.autopilot = MAV_AUTOPILOT_PX4;

			if (_airspeed.valid()) {
				_airspeed.get_scaled(msg.airspeed, 5.0f);
			}

			if (_airspeed_sp.valid()) {
				_airspeed_sp.get_scaled(msg.airspeed_sp, 5.0f);
			}

			if (_battery.valid()) {
				_battery.get_scaled(msg.battery, 100.0f);
			}

			if (_climb_rate.valid()) {
				_climb_rate.get_scaled(msg.climb_rate, 10.0f);
			}

			if (_eph.valid()) {
				_eph.get_scaled(msg.eph, 10.0f);
			}

			if (_epv.valid()) {
				_epv.get_scaled(msg.epv, 10.0f);
			}

			if (_groundspeed.valid()) {
				_groundspeed.get_scaled(msg.groundspeed, 5.0f);
			}

			if (_temperature.valid()) {
				_temperature.get_scaled(msg.temperature_air, 1.0f);
			}

			if (_throttle.valid()) {
				_throttle.get_scaled(msg.throttle, 100.0f);
			}

			if (_windspeed.valid()) {
				_windspeed.get_scaled(msg.windspeed, 5.0f);
			}

			reset_analysers(t);

			mavlink_msg_high_latency2_send_struct(_mavlink->get_channel(), &msg);
		}

		return updated;

	} else {
		// reset the analysers every 60 seconds if nothing should be transmitted
		if ((t - _last_reset_time) > 60000000) {
			reset_analysers(t);
			PX4_DEBUG("Resetting HIGH_LATENCY2 analysers");
		}

		return false;
	}
}

void MavlinkStreamHighLatency2::reset_analysers(const hrt_abstime t)
{
	// reset the analyzers
	_airspeed.reset();
	_airspeed_sp.reset();
	_battery.reset();
	_climb_rate.reset();
	_eph.reset();
	_epv.reset();
	_groundspeed.reset();
	_temperature.reset();
	_throttle.reset();
	_windspeed.reset();

	_last_reset_time = t;
}

bool MavlinkStreamHighLatency2::write_airspeed(mavlink_high_latency2_t *msg)
{
	struct airspeed_s airspeed;

	const bool updated = _airspeed_sub->update(&_airspeed_time, &airspeed);

	if (_airspeed_time > 0) {
		if (airspeed.confidence < 0.95f) { // the same threshold as for the commander
			msg->failure_flags |= HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE;
		}
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_attitude_sp(mavlink_high_latency2_t *msg)
{
	struct vehicle_attitude_setpoint_s attitude_sp;

	const bool updated = _attitude_sp_sub->update(&_attitude_sp_time, &attitude_sp);

	if (_attitude_sp_time > 0) {
		msg->target_heading = static_cast<uint8_t>(math::degrees(wrap_2pi(attitude_sp.yaw_body)) * 0.5f);
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_battery_status(mavlink_high_latency2_t *msg)
{
	struct battery_status_s battery;

	const bool updated = _battery_sub->update(&_battery_time, &battery);

	if (_battery_time > 0) {
		if (battery.warning > battery_status_s::BATTERY_WARNING_LOW) {
			msg->failure_flags |= HL_FAILURE_FLAG_BATTERY;
		}
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_estimator_status(mavlink_high_latency2_t *msg)
{
	struct estimator_status_s estimator_status;

	const bool updated = _estimator_status_sub->update(&_estimator_status_time, &estimator_status);

	if (_estimator_status_time > 0) {
		if (estimator_status.gps_check_fail_flags > 0 ||
		    estimator_status.filter_fault_flags > 0 ||
		    estimator_status.innovation_check_flags > 0) {
			msg->failure_flags |= HL_FAILURE_FLAG_ESTIMATOR;
		}
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_fw_ctrl_status(mavlink_high_latency2_t *msg)
{
	position_controller_status_s pos_ctrl_status = {};

	const bool updated = _pos_ctrl_status_sub->update(&_pos_ctrl_status_time, &pos_ctrl_status);

	if (_pos_ctrl_status_time > 0) {
		uint16_t target_distance;
		convert_limit_safe(pos_ctrl_status.wp_dist * 0.1f, target_distance);
		msg->target_distance = target_distance;
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_geofence_result(mavlink_high_latency2_t *msg)
{
	struct geofence_result_s geofence;

	const bool updated = _geofence_sub->update(&_geofence_time, &geofence);

	if (_geofence_time > 0) {
		if (geofence.geofence_violated) {
			msg->failure_flags |= HL_FAILURE_FLAG_GEOFENCE;
		}
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_global_position(mavlink_high_latency2_t *msg)
{
	struct vehicle_global_position_s global_pos;

	const bool updated = _global_pos_sub->update(&_global_pos_time, &global_pos);

	if (_global_pos_time > 0) {
		int32_t latitude, longitude;
		convert_limit_safe(global_pos.lat * 1e7, latitude);
		convert_limit_safe(global_pos.lon * 1e7, longitude);
		msg->latitude = latitude;
		msg->longitude = longitude;

		int16_t altitude;

		if (global_pos.alt > 0) {
			convert_limit_safe(global_pos.alt + 0.5f, altitude);

		} else {
			convert_limit_safe(global_pos.alt - 0.5f, altitude);
		}

		msg->altitude = altitude;

		msg->heading = static_cast<uint8_t>(math::degrees(wrap_2pi(global_pos.yaw)) * 0.5f);
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_mission_result(mavlink_high_latency2_t *msg)
{
	struct mission_result_s mission_result;

	const bool updated = _mission_result_sub->update(&_mission_result_time, &mission_result);

	if (_mission_result_time > 0) {
		msg->wp_num = mission_result.seq_current;
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_tecs_status(mavlink_high_latency2_t *msg)
{
	struct tecs_status_s tecs_status;

	const bool updated = _tecs_status_sub->update(&_tecs_time, &tecs_status);

	if (_tecs_time > 0) {
		int16_t target_altitude;
		convert_limit_safe(tecs_status.altitude_sp, target_altitude);
		msg->target_altitude = target_altitude;
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_vehicle_status(mavlink_high_latency2_t *msg)
{
	struct vehicle_status_s status;

	const bool updated = _status_sub->update(&_status_time, &status);

	if (_status_time > 0) {
		if ((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE)
		    && !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE)) {
			msg->failure_flags |= HL_FAILURE_FLAG_ABSOLUTE_PRESSURE;
		}

		if (((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_ACCEL)
		     && !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_ACCEL)) ||
		    ((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_ACCEL2) &&
		     !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_ACCEL2))) {
			msg->failure_flags |= HL_FAILURE_FLAG_3D_ACCEL;
		}

		if (((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_GYRO)
		     && !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_GYRO)) ||
		    ((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_GYRO2) &&
		     !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_GYRO2))) {
			msg->failure_flags |= HL_FAILURE_FLAG_3D_GYRO;
		}

		if (((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_MAG)
		     && !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_MAG)) ||
		    ((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_MAG2) &&
		     !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_MAG2))) {
			msg->failure_flags |= HL_FAILURE_FLAG_3D_MAG;
		}

		if ((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_TERRAIN)
		    && !(status.onboard_control_sensors_health & MAV_SYS_STATUS_TERRAIN)) {
			msg->failure_flags |= HL_FAILURE_FLAG_TERRAIN;
		}

		if (status.rc_signal_lost) {
			msg->failure_flags |= HL_FAILURE_FLAG_RC_RECEIVER;
		}

		if (status.engine_failure) {
			msg->failure_flags |= HL_FAILURE_FLAG_ENGINE;
		}

		if (status.mission_failure) {
			msg->failure_flags |= HL_FAILURE_FLAG_MISSION;
		}

		// flight mode
		union px4_custom_mode custom_mode;
		uint8_t mavlink_base_mode;
		get_mavlink_navigation_mode(&status, &mavlink_base_mode, &custom_mode);
		msg->custom_mode = custom_mode.custom_mode_hl;
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_vehicle_status_flags(mavlink_high_latency2_t *msg)
{
	struct vehicle_status_flags_s status_flags;

	const bool updated = _status_flags_sub->update(&_status_flags_time, &status_flags);

	if (_status_flags_time > 0) {
		if (!status_flags.condition_global_position_valid) { //TODO check if there is a better way to get only GPS failure
			msg->failure_flags |= HL_FAILURE_FLAG_GPS;
		}

		if (status_flags.offboard_control_signal_lost && status_flags.offboard_control_signal_found_once) {
			msg->failure_flags |= HL_FAILURE_FLAG_OFFBOARD_LINK;
		}
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_wind_estimate(mavlink_high_latency2_t *msg)
{
	struct wind_estimate_s wind;

	const bool updated = _wind_sub->update(&_wind_time, &wind);

	if (_wind_time > 0) {
		msg->wind_heading = static_cast<uint8_t>(
					    math::degrees(wrap_2pi(atan2f(wind.windspeed_east, wind.windspeed_north))) * 0.5f);
	}

	return updated;
}

void MavlinkStreamHighLatency2::update_data()
{
	const hrt_abstime t = hrt_absolute_time();

	if (t > _last_update_time) {
		// first order low pass filter for the update rate
		_update_rate_filtered = 0.97f * _update_rate_filtered + 0.03f / ((t - _last_update_time) * 1e-6f);
		_last_update_time = t;
	}

	update_airspeed();

	update_tecs_status();

	update_battery_status();

	update_global_position();

	update_gps();

	update_vehicle_status();

	update_wind_estimate();
}

void MavlinkStreamHighLatency2::update_airspeed()
{
	airspeed_s airspeed;

	if (_airspeed_sub->update(&airspeed)) {
		_airspeed.add_value(airspeed.indicated_airspeed_m_s, _update_rate_filtered);
		_temperature.add_value(airspeed.air_temperature_celsius, _update_rate_filtered);
	}
}

void MavlinkStreamHighLatency2::update_tecs_status()
{
	tecs_status_s tecs_status;

	if (_tecs_status_sub->update(&tecs_status)) {
		_airspeed_sp.add_value(tecs_status.airspeed_sp, _update_rate_filtered);
	}
}

void MavlinkStreamHighLatency2::update_battery_status()
{
	battery_status_s battery;

	if (_battery_sub->update(&battery)) {
		_battery.add_value(battery.remaining, _update_rate_filtered);
	}
}

void MavlinkStreamHighLatency2::update_global_position()
{
	vehicle_global_position_s global_pos;

	if (_global_pos_sub->update(&global_pos)) {
		_climb_rate.add_value(fabsf(global_pos.vel_d), _update_rate_filtered);
		_groundspeed.add_value(sqrtf(global_pos.vel_n * global_pos.vel_n + global_pos.vel_e * global_pos.vel_e),
				       _update_rate_filtered);
	}
}

void MavlinkStreamHighLatency2::update_gps()
{
	vehicle_gps_position_s gps;

	if (_gps_sub->update(&gps)) {
		_eph.add_value(gps.eph, _update_rate_filtered);
		_epv.add_value(gps.epv, _update_rate_filtered);
	}
}

void MavlinkStreamHighLatency2::update_vehicle_status()
{
	vehicle_status_s status;

	if (_status_sub->update(&status)) {
		if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			struct actuator_controls_s actuator = {};

			if (status.is_vtol && !status.is_rotary_wing) {
				if (_actuator_sub_1->update(&actuator)) {
					_throttle.add_value(actuator.control[actuator_controls_s::INDEX_THROTTLE], _update_rate_filtered);
				}

			} else {
				if (_actuator_sub_0->update(&actuator)) {
					_throttle.add_value(actuator.control[actuator_controls_s::INDEX_THROTTLE], _update_rate_filtered);
				}
			}

		} else {
			_throttle.add_value(0.0f, _update_rate_filtered);
		}
	}
}

void MavlinkStreamHighLatency2::update_wind_estimate()
{
	wind_estimate_s wind;

	if (_wind_sub->update(&wind)) {
		_windspeed.add_value(sqrtf(wind.windspeed_north * wind.windspeed_north + wind.windspeed_east * wind.windspeed_east),
				     _update_rate_filtered);
	}
}

void MavlinkStreamHighLatency2::set_default_values(mavlink_high_latency2_t &msg) const
{
	msg.airspeed = 0;
	msg.airspeed_sp = 0;
	msg.altitude = 0;
	msg.autopilot = MAV_AUTOPILOT_ENUM_END;
	msg.battery = -1;
	msg.climb_rate = 0;
	msg.custom0 = INT8_MIN;
	msg.custom1 = INT8_MIN;
	msg.custom2 = INT8_MIN;
	msg.eph = UINT8_MAX;
	msg.epv = UINT8_MAX;
	msg.failure_flags = 0;
	msg.custom_mode = 0;
	msg.groundspeed = 0;
	msg.heading = 0;
	msg.latitude = 0;
	msg.longitude = 0;
	msg.target_altitude = 0;
	msg.target_distance = 0;
	msg.target_heading = 0;
	msg.temperature_air = INT8_MIN;
	msg.throttle = 0;
	msg.timestamp = 0;
	msg.type = MAV_TYPE_ENUM_END;
	msg.wind_heading = 0;
	msg.windspeed = 0;
	msg.wp_num = UINT16_MAX;
}
/****************************************************************************
 *
 *   Copyright (c) 2014-2016 PX4 Development Team. All rights reserved.
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

/// @file mavlink_log_handler.h
/// @author px4dev, Gus Grubba <mavlink@grubba.com>

#include "mavlink_log_handler.h"
#include "mavlink_main.h"
#include <sys/stat.h>
#include <time.h>

#define MOUNTPOINT PX4_STORAGEDIR

static const char *kLogRoot    = MOUNTPOINT "/log";
static const char *kLogData    = MOUNTPOINT "/logdata.txt";
static const char *kTmpData    = MOUNTPOINT "/$log$.txt";

#ifdef __PX4_NUTTX
#define PX4LOG_REGULAR_FILE DTYPE_FILE
#define PX4LOG_DIRECTORY    DTYPE_DIRECTORY
#else
#define PX4LOG_REGULAR_FILE DT_REG
#define PX4LOG_DIRECTORY    DT_DIR
#endif

//#define MAVLINK_LOG_HANDLER_VERBOSE

#ifdef MAVLINK_LOG_HANDLER_VERBOSE
#define PX4LOG_WARN(fmt, ...) warnx(fmt, ##__VA_ARGS__)
#else
#define PX4LOG_WARN(fmt, ...)
#endif

//-------------------------------------------------------------------
static bool
stat_file(const char *file, time_t *date = nullptr, uint32_t *size = nullptr)
{
	struct stat st;

	if (stat(file, &st) == 0) {
		if (date) { *date = st.st_mtime; }

		if (size) { *size = st.st_size; }

		return true;
	}

	return false;
}

//-------------------------------------------------------------------
MavlinkLogHandler::MavlinkLogHandler(Mavlink *mavlink)
	: _pLogHandlerHelper(nullptr),
	  _mavlink(mavlink)
{

}

//-------------------------------------------------------------------
void
MavlinkLogHandler::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
		_log_request_list(msg);
		break;

	case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
		_log_request_data(msg);
		break;

	case MAVLINK_MSG_ID_LOG_ERASE:
		_log_request_erase(msg);
		break;

	case MAVLINK_MSG_ID_LOG_REQUEST_END:
		_log_request_end(msg);
		break;
	}
}

//-------------------------------------------------------------------
unsigned
MavlinkLogHandler::get_size()
{
	//-- Sending Log Entries
	if (_pLogHandlerHelper && _pLogHandlerHelper->current_status == LogListHelper::LOG_HANDLER_LISTING) {
		return MAVLINK_MSG_ID_LOG_ENTRY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		//-- Sending Log Data (contents of one of the log files)

	} else if (_pLogHandlerHelper && _pLogHandlerHelper->current_status == LogListHelper::LOG_HANDLER_SENDING_DATA) {
		return MAVLINK_MSG_ID_LOG_DATA_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		//-- Idle

	} else {
		return 0;
	}
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::send(const hrt_abstime /*t*/)
{
	//-- An arbitrary count of max bytes in one go (one of the two below but never both)
#define MAX_BYTES_SEND 256 * 1024
	size_t count = 0;

	//-- Log Entries
	while (_pLogHandlerHelper && _pLogHandlerHelper->current_status == LogListHelper::LOG_HANDLER_LISTING
	       && _mavlink->get_free_tx_buf() > get_size() && count < MAX_BYTES_SEND) {
		count += _log_send_listing();
	}

	//-- Log Data
	while (_pLogHandlerHelper && _pLogHandlerHelper->current_status == LogListHelper::LOG_HANDLER_SENDING_DATA
	       && _mavlink->get_free_tx_buf() > get_size() && count < MAX_BYTES_SEND) {
		count += _log_send_data();
	}
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_log_request_list(const mavlink_message_t *msg)
{
	mavlink_log_request_list_t request;
	mavlink_msg_log_request_list_decode(msg, &request);

	//-- Check for re-requests (data loss) or new request
	if (_pLogHandlerHelper) {
		_pLogHandlerHelper->current_status = LogListHelper::LOG_HANDLER_IDLE;

		//-- Is this a new request?
		if ((request.end - request.start) > _pLogHandlerHelper->log_count) {
			delete _pLogHandlerHelper;
			_pLogHandlerHelper = nullptr;
		}
	}

	if (!_pLogHandlerHelper) {
		//-- Prepare new request
		_pLogHandlerHelper = new LogListHelper;
	}

	if (_pLogHandlerHelper->log_count) {
		//-- Define (and clamp) range
		_pLogHandlerHelper->next_entry = request.start < _pLogHandlerHelper->log_count ? request.start :
						 _pLogHandlerHelper->log_count - 1;
		_pLogHandlerHelper->last_entry = request.end   < _pLogHandlerHelper->log_count ? request.end   :
						 _pLogHandlerHelper->log_count - 1;
	}

	PX4LOG_WARN("\nMavlinkLogHandler::_log_request_list: start: %u last: %u count: %u\n",
		    _pLogHandlerHelper->next_entry,
		    _pLogHandlerHelper->last_entry,
		    _pLogHandlerHelper->log_count);
	//-- Enable streaming
	_pLogHandlerHelper->current_status = LogListHelper::LOG_HANDLER_LISTING;
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_log_request_data(const mavlink_message_t *msg)
{
	//-- If we haven't listed, we can't do much
	if (!_pLogHandlerHelper) {
		PX4LOG_WARN("MavlinkLogHandler::_log_request_data Log request with no list requested.\n");
		return;
	}

	mavlink_log_request_data_t request;
	mavlink_msg_log_request_data_decode(msg, &request);

	//-- Does the requested log exist?
	if (request.id >= _pLogHandlerHelper->log_count) {
		PX4LOG_WARN("MavlinkLogHandler::_log_request_data Requested log %u but we only have %u.\n", request.id,
			    _pLogHandlerHelper->log_count);
		return;
	}

	//-- If we were sending log entries, stop it
	_pLogHandlerHelper->current_status = LogListHelper::LOG_HANDLER_IDLE;

	if (_pLogHandlerHelper->current_log_index != request.id) {
		//-- Init send log dataset
		_pLogHandlerHelper->current_log_filename[0] = 0;
		_pLogHandlerHelper->current_log_index = request.id;
		uint32_t time_utc = 0;

		if (!_pLogHandlerHelper->get_entry(_pLogHandlerHelper->current_log_index, _pLogHandlerHelper->current_log_size,
						   time_utc,
						   _pLogHandlerHelper->current_log_filename, sizeof(_pLogHandlerHelper->current_log_filename))) {
			PX4LOG_WARN("LogListHelper::get_entry failed.\n");
			return;
		}

		_pLogHandlerHelper->open_for_transmit();
	}

	_pLogHandlerHelper->current_log_data_offset = request.ofs;

	if (_pLogHandlerHelper->current_log_data_offset >= _pLogHandlerHelper->current_log_size) {
		_pLogHandlerHelper->current_log_data_remaining = 0;

	} else {
		_pLogHandlerHelper->current_log_data_remaining = _pLogHandlerHelper->current_log_size - request.ofs;
	}

	if (_pLogHandlerHelper->current_log_data_remaining > request.count) {
		_pLogHandlerHelper->current_log_data_remaining = request.count;
	}

	//-- Enable streaming
	_pLogHandlerHelper->current_status = LogListHelper::LOG_HANDLER_SENDING_DATA;
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_log_request_erase(const mavlink_message_t * /*msg*/)
{
	/*
	mavlink_log_erase_t request;
	mavlink_msg_log_erase_decode(msg, &request);
	*/
	if (_pLogHandlerHelper) {
		delete _pLogHandlerHelper;
		_pLogHandlerHelper = nullptr;
	}

	//-- Delete all logs
	LogListHelper::delete_all(kLogRoot);
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_log_request_end(const mavlink_message_t * /*msg*/)
{
	PX4LOG_WARN("MavlinkLogHandler::_log_request_end\n");

	if (_pLogHandlerHelper) {
		delete _pLogHandlerHelper;
		_pLogHandlerHelper = nullptr;
	}
}

//-------------------------------------------------------------------
size_t
MavlinkLogHandler::_log_send_listing()
{
	mavlink_log_entry_t response;
	uint32_t size, date;
	_pLogHandlerHelper->get_entry(_pLogHandlerHelper->next_entry, size, date);
	response.size         = size;
	response.time_utc     = date;
	response.id           = _pLogHandlerHelper->next_entry;
	response.num_logs     = _pLogHandlerHelper->log_count;
	response.last_log_num = _pLogHandlerHelper->last_entry;
	mavlink_msg_log_entry_send_struct(_mavlink->get_channel(), &response);

	//-- If we're done listing, flag it.
	if (_pLogHandlerHelper->next_entry == _pLogHandlerHelper->last_entry) {
		_pLogHandlerHelper->current_status = LogListHelper::LOG_HANDLER_IDLE;

	} else {
		_pLogHandlerHelper->next_entry++;
	}

	PX4LOG_WARN("MavlinkLogHandler::_log_send_listing id: %u count: %u last: %u size: %u date: %u status: %d\n",
		    response.id,
		    response.num_logs,
		    response.last_log_num,
		    response.size,
		    response.time_utc,
		    _pLogHandlerHelper->current_status);
	return sizeof(response);
}

//-------------------------------------------------------------------
size_t
MavlinkLogHandler::_log_send_data()
{
	mavlink_log_data_t response;
	memset(&response, 0, sizeof(response));
	uint32_t len = _pLogHandlerHelper->current_log_data_remaining;

	if (len > sizeof(response.data)) {
		len = sizeof(response.data);
	}

	size_t read_size = _pLogHandlerHelper->get_log_data(len, response.data);
	response.ofs     = _pLogHandlerHelper->current_log_data_offset;
	response.id      = _pLogHandlerHelper->current_log_index;
	response.count   = read_size;
	mavlink_msg_log_data_send_struct(_mavlink->get_channel(), &response);
	_pLogHandlerHelper->current_log_data_offset    += read_size;
	_pLogHandlerHelper->current_log_data_remaining -= read_size;

	if (read_size < sizeof(response.data) || _pLogHandlerHelper->current_log_data_remaining == 0) {
		_pLogHandlerHelper->current_status = LogListHelper::LOG_HANDLER_IDLE;
	}

	return sizeof(response);
}

//-------------------------------------------------------------------
LogListHelper::LogListHelper()
	: next_entry(0)
	, last_entry(0)
	, log_count(0)
	, current_status(LOG_HANDLER_IDLE)
	, current_log_index(UINT16_MAX)
	, current_log_size(0)
	, current_log_data_offset(0)
	, current_log_data_remaining(0)
	, current_log_filep(nullptr)
{
	_init();
}

//-------------------------------------------------------------------
LogListHelper::~LogListHelper()
{
	// Remove log data files (if any)
	unlink(kLogData);
	unlink(kTmpData);
}

//-------------------------------------------------------------------
bool
LogListHelper::get_entry(int idx, uint32_t &size, uint32_t &date, char *filename, int filename_len)
{
	//-- Find log file in log list file created during init()
	size = 0;
	date = 0;
	bool result = false;
	//-- Open list of log files
	FILE *f = ::fopen(kLogData, "r");

	if (f) {
		//--- Find requested entry
		char line[160];
		int count = 0;

		while (fgets(line, sizeof(line), f)) {
			//-- Found our "index"
			if (count++ == idx) {
				char file[160];

				if (sscanf(line, "%u %u %s", &date, &size, file) == 3) {
					if (filename && filename_len > 0) {
						strncpy(filename, file, filename_len);
						filename[filename_len - 1] = 0; // ensure null-termination
					}

					result = true;
					break;
				}
			}
		}

		fclose(f);
	}

	return result;
}

//-------------------------------------------------------------------
bool
LogListHelper::open_for_transmit()
{
	if (current_log_filep) {
		::fclose(current_log_filep);
		current_log_filep = nullptr;
	}

	current_log_filep = ::fopen(current_log_filename, "rb");

	if (!current_log_filep) {
		PX4LOG_WARN("MavlinkLogHandler::open_for_transmit Could not open %s\n", current_log_filename);
		return false;
	}

	return true;
}

//-------------------------------------------------------------------
size_t
LogListHelper::get_log_data(uint8_t len, uint8_t *buffer)
{
	if (!current_log_filename[0]) {
		return 0;
	}

	if (!current_log_filep) {
		PX4LOG_WARN("MavlinkLogHandler::get_log_data file not open %s\n", current_log_filename);
		return 0;
	}

	long int offset = current_log_data_offset - ftell(current_log_filep);

	if (offset && fseek(current_log_filep, offset, SEEK_CUR)) {
		fclose(current_log_filep);
		current_log_filep = nullptr;
		PX4LOG_WARN("MavlinkLogHandler::get_log_data Seek error in %s\n", current_log_filename);
		return 0;
	}

	size_t result = fread(buffer, 1, len, current_log_filep);
	return result;
}

//-------------------------------------------------------------------
void
LogListHelper::_init()
{
	/*

		When this helper is created, it scans the log directory
		and collects all log files found into one file for easy,
		subsequent access.
	*/

	current_log_filename[0] = 0;
	// Remove old log data file (if any)
	unlink(kLogData);
	// Open log directory
	DIR *dp = opendir(kLogRoot);

	if (dp == nullptr) {
		// No log directory. Nothing to do.
		return;
	}

	// Create work file
	FILE *f = ::fopen(kTmpData, "w");

	if (!f) {
		PX4LOG_WARN("MavlinkLogHandler::init Error creating %s\n", kTmpData);
		closedir(dp);
		return;
	}

	// Scan directory and collect log files
	struct dirent *result = nullptr;

	while ((result = readdir(dp))) {
		if (result->d_type == PX4LOG_DIRECTORY) {
			time_t tt = 0;
			char log_path[128];
			int ret = snprintf(log_path, sizeof(log_path), "%s/%s", kLogRoot, result->d_name);
			bool path_is_ok = (ret > 0) && (ret < (int)sizeof(log_path));

			if (path_is_ok) {
				if (_get_session_date(log_path, result->d_name, tt)) {
					_scan_logs(f, log_path, tt);
				}
			}
		}
	}

	closedir(dp);
	fclose(f);

	// Rename temp file to data file
	if (rename(kTmpData, kLogData)) {
		PX4LOG_WARN("MavlinkLogHandler::init Error renaming %s\n", kTmpData);
		log_count = 0;
	}
}

//-------------------------------------------------------------------
bool
LogListHelper::_get_session_date(const char *path, const char *dir, time_t &date)
{
	if (strlen(dir) > 4) {
		// Always try to get file time first
		if (stat_file(path, &date)) {
			// Try to prevent taking date if it's around 1970 (use the logic below instead)
			if (date > 60 * 60 * 24) {
				return true;
			}
		}

		// Convert "sess000" to 00:00 Jan 1 1970 (day per session)
		if (strncmp(dir, "sess", 4) == 0) {
			unsigned u;

			if (sscanf(&dir[4], "%u", &u) == 1) {
				date = u * 60 * 60 * 24;
				return true;
			}
		}
	}

	return false;
}

//-------------------------------------------------------------------
void
LogListHelper::_scan_logs(FILE *f, const char *dir, time_t &date)
{
	DIR *dp = opendir(dir);

	if (dp) {
		struct dirent *result = nullptr;

		while ((result = readdir(dp))) {
			if (result->d_type == PX4LOG_REGULAR_FILE) {
				time_t  ldate = date;
				uint32_t size = 0;
				char log_file_path[128];
				int ret = snprintf(log_file_path, sizeof(log_file_path), "%s/%s", dir, result->d_name);
				bool path_is_ok = (ret > 0) && (ret < (int)sizeof(log_file_path));

				if (path_is_ok) {
					if (_get_log_time_size(log_file_path, result->d_name, ldate, size)) {
						//-- Write result->out to list file
						fprintf(f, "%u %u %s\n", (unsigned)ldate, (unsigned)size, log_file_path);
						log_count++;
					}
				}
			}
		}

		closedir(dp);
	}
}

//-------------------------------------------------------------------
bool
LogListHelper::_get_log_time_size(const char *path, const char *file, time_t &date, uint32_t &size)
{
	if (file && file[0]) {
		if (strstr(file, ".px4log") || strstr(file, ".ulg")) {
			// Always try to get file time first
			if (stat_file(path, &date, &size)) {
				// Try to prevent taking date if it's around 1970 (use the logic below instead)
				if (date > 60 * 60 * 24) {
					return true;
				}
			}

			// Convert "log000" to 00:00 (minute per flight in session)
			if (strncmp(file, "log", 3) == 0) {
				unsigned u;

				if (sscanf(&file[3], "%u", &u) == 1) {
					date += (u * 60);

					if (stat_file(path, nullptr, &size)) {
						return true;
					}
				}
			}
		}
	}

	return false;
}

//-------------------------------------------------------------------
void
LogListHelper::delete_all(const char *dir)
{
	//-- Open log directory
	DIR *dp = opendir(dir);

	if (dp == nullptr) {
		return;
	}

	struct dirent *result = nullptr;

	while ((result = readdir(dp))) {
		// no more entries?
		if (result == nullptr) {
			break;
		}

		if (result->d_type == PX4LOG_DIRECTORY && result->d_name[0] != '.') {
			char log_path[128];
			int ret = snprintf(log_path, sizeof(log_path), "%s/%s", dir, result->d_name);
			bool path_is_ok = (ret > 0) && (ret < (int)sizeof(log_path));

			if (path_is_ok) {
				LogListHelper::delete_all(log_path);

				if (rmdir(log_path)) {
					PX4LOG_WARN("MavlinkLogHandler::delete_all Error removing %s\n", log_path);
				}
			}
		}

		if (result->d_type == PX4LOG_REGULAR_FILE) {
			char log_path[128];
			int ret = snprintf(log_path, sizeof(log_path), "%s/%s", dir, result->d_name);
			bool path_is_ok = (ret > 0) && (ret < (int)sizeof(log_path));

			if (path_is_ok) {
				if (unlink(log_path)) {
					PX4LOG_WARN("MavlinkLogHandler::delete_all Error deleting %s\n", log_path);
				}
			}
		}
	}

	closedir(dp);
}
/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_main.cpp
 * MAVLink 1.0 protocol implementation.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_module.h>
#include <px4_cli.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <math.h>
#include <poll.h>
#include <termios.h>
#include <time.h>

#ifdef CONFIG_NET
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netutils/netlib.h>
#endif

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <parameters/param.h>
#include <systemlib/err.h>
#include <perf/perf_counter.h>
#include <systemlib/mavlink_log.h>
#include <lib/ecl/geo/geo.h>
#include <dataman/dataman.h>
#include <version/version.h>
#include <mathlib/mathlib.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/mavlink_log.h>

#include "mavlink_bridge_header.h"
#include "mavlink_main.h"
#include "mavlink_messages.h"
#include "mavlink_receiver.h"
#include "mavlink_rate_limiter.h"
#include "mavlink_command_sender.h"

// Guard against MAVLink misconfiguration
#ifndef MAVLINK_CRC_EXTRA
#error MAVLINK_CRC_EXTRA has to be defined on PX4 systems
#endif

// Guard against flow control misconfiguration
#if defined (CRTSCTS) && defined (__PX4_NUTTX) && (CRTSCTS != (CRTS_IFLOW | CCTS_OFLOW))
#error The non-standard CRTSCTS define is incorrect. Fix this in the OS or replace with (CRTS_IFLOW | CCTS_OFLOW)
#endif

#ifdef CONFIG_NET
#define MAVLINK_NET_ADDED_STACK 350
#else
#define MAVLINK_NET_ADDED_STACK 0
#endif

#define DEFAULT_REMOTE_PORT_UDP			14550 ///< GCS port per MAVLink spec
#define DEFAULT_DEVICE_NAME			"/dev/ttyS1"
#define MAX_DATA_RATE				10000000	///< max data rate in bytes/s
#define MAIN_LOOP_DELAY 			10000	///< 100 Hz @ 1000 bytes/s data rate
#define FLOW_CONTROL_DISABLE_THRESHOLD		40	///< picked so that some messages still would fit it.
//#define MAVLINK_PRINT_PACKETS

static Mavlink *_mavlink_instances = nullptr;

/**
 * mavlink app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mavlink_main(int argc, char *argv[]);

extern mavlink_system_t mavlink_system;

void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length)
{
	Mavlink *m = Mavlink::get_instance(chan);

	if (m != nullptr) {
		m->send_bytes(ch, length);
#ifdef MAVLINK_PRINT_PACKETS

		for (unsigned i = 0; i < length; i++) {
			printf("%02x", (unsigned char)ch[i]);
		}

#endif
	}
}

void mavlink_start_uart_send(mavlink_channel_t chan, int length)
{
	Mavlink *m = Mavlink::get_instance(chan);

	if (m != nullptr) {
		(void)m->begin_send();
#ifdef MAVLINK_PRINT_PACKETS
		printf("START PACKET (%u): ", (unsigned)chan);
#endif
	}
}

void mavlink_end_uart_send(mavlink_channel_t chan, int length)
{
	Mavlink *m = Mavlink::get_instance(chan);

	if (m != nullptr) {
		(void)m->send_packet();
#ifdef MAVLINK_PRINT_PACKETS
		printf("\n");
#endif
	}
}

/*
 * Internal function to give access to the channel status for each channel
 */
mavlink_status_t *mavlink_get_channel_status(uint8_t channel)
{
	Mavlink *m = Mavlink::get_instance(channel);

	if (m != nullptr) {
		return m->get_status();

	} else {
		return nullptr;
	}
}

/*
 * Internal function to give access to the channel buffer for each channel
 */
mavlink_message_t *mavlink_get_channel_buffer(uint8_t channel)
{
	Mavlink *m = Mavlink::get_instance(channel);

	if (m != nullptr) {
		return m->get_buffer();

	} else {
		return nullptr;
	}
}

static void usage();

bool Mavlink::_boot_complete = false;

Mavlink::Mavlink() :
	ModuleParams(nullptr),
	_device_name("/dev/ttyS1"),
	_task_should_exit(false),
	next(nullptr),
	_instance_id(0),
	_transmitting_enabled(true),
	_transmitting_enabled_commanded(false),
	_mavlink_log_pub(nullptr),
	_task_running(false),
	_mavlink_buffer{},
	_mavlink_status{},
	_hil_enabled(false),
	_generate_rc(false),
	_is_usb_uart(false),
	_wait_to_transmit(false),
	_received_messages(false),
	_main_loop_delay(1000),
	_subscriptions(nullptr),
	_streams(nullptr),
	_mavlink_shell(nullptr),
	_mavlink_ulog(nullptr),
	_mavlink_ulog_stop_requested(false),
	_mode(MAVLINK_MODE_NORMAL),
	_channel(MAVLINK_COMM_0),
	_logbuffer(5, sizeof(mavlink_log_s)),
	_receive_thread{},
	_forwarding_on(false),
	_ftp_on(false),
	_uart_fd(-1),
	_baudrate(57600),
	_datarate(1000),
	_rate_mult(1.0f),
	_mavlink_param_queue_index(0),
	mavlink_link_termination_allowed(false),
	_subscribe_to_stream(nullptr),
	_subscribe_to_stream_rate(0.0f),
	_udp_initialised(false),
	_flow_control_mode(Mavlink::FLOW_CONTROL_OFF),
	_last_write_success_time(0),
	_last_write_try_time(0),
	_mavlink_start_time(0),
	_protocol_version_switch(-1),
	_protocol_version(0),
	_bytes_tx(0),
	_bytes_txerr(0),
	_bytes_rx(0),
	_bytes_timestamp(0),
#if defined(CONFIG_NET) || defined(__PX4_POSIX)
	_myaddr {},
	_src_addr{},
	_bcast_addr{},
	_src_addr_initialized(false),
	_broadcast_address_found(false),
	_broadcast_address_not_found_warned(false),
	_broadcast_failed_warned(false),
	_network_buf{},
	_network_buf_len(0),
#endif
	_socket_fd(-1),
	_protocol(SERIAL),
	_network_port(14556),
	_remote_port(DEFAULT_REMOTE_PORT_UDP),
	_message_buffer {},
	_message_buffer_mutex {},
	_send_mutex {},

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mavlink_el"))
{
	_instance_id = Mavlink::instance_count();

	/* set channel according to instance id */
	switch (_instance_id) {
	case 0:
		_channel = MAVLINK_COMM_0;
		break;

	case 1:
		_channel = MAVLINK_COMM_1;
		break;

	case 2:
		_channel = MAVLINK_COMM_2;
		break;

	case 3:
		_channel = MAVLINK_COMM_3;
		break;
#ifdef MAVLINK_COMM_4

	case 4:
		_channel = MAVLINK_COMM_4;
		break;
#endif
#ifdef MAVLINK_COMM_5

	case 5:
		_channel = MAVLINK_COMM_5;
		break;
#endif
#ifdef MAVLINK_COMM_6

	case 6:
		_channel = MAVLINK_COMM_6;
		break;
#endif

	default:
		PX4_WARN("instance ID is out of range");
		px4_task_exit(1);
		break;
	}

	// initialise parameter cache
	mavlink_update_parameters();

	// save the current system- and component ID because we don't allow them to change during operation
	int sys_id = _param_system_id.get();

	if (sys_id > 0 && sys_id < 255) {
		mavlink_system.sysid = sys_id;
	}

	int comp_id = _param_component_id.get();

	if (comp_id > 0 && comp_id < 255) {
		mavlink_system.compid = comp_id;
	}
}

Mavlink::~Mavlink()
{
	perf_free(_loop_perf);

	if (_task_running) {
		/* task wakes up every 10ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			px4_usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				//TODO store main task handle in Mavlink instance to allow killing task
				//task_delete(_mavlink_task);
				break;
			}
		} while (_task_running);
	}
}

void Mavlink::mavlink_update_parameters()
{
	updateParams();

	int32_t proto = _param_mav_proto_version.get();

	if (_protocol_version_switch != proto) {
		_protocol_version_switch = proto;
		set_proto_version(proto);
	}

	if (_param_system_type.get() < 0 || _param_system_type.get() >= MAV_TYPE_ENUM_END) {
		_param_system_type.set(0);
		_param_system_type.commit_no_notification();
		PX4_ERR("MAV_TYPE parameter invalid, resetting to 0.");
	}
}

void
Mavlink::set_proto_version(unsigned version)
{
	if ((version == 1 || version == 0) &&
	    ((_protocol_version_switch == 0) || (_protocol_version_switch == 1))) {
		get_status()->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
		_protocol_version = 1;

	} else if (version == 2 &&
		   ((_protocol_version_switch == 0) || (_protocol_version_switch == 2))) {
		get_status()->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
		_protocol_version = 2;
	}
}

int
Mavlink::instance_count()
{
	unsigned inst_index = 0;
	Mavlink *inst;

	LL_FOREACH(::_mavlink_instances, inst) {
		inst_index++;
	}

	return inst_index;
}

Mavlink *
Mavlink::get_instance(int instance)
{
	Mavlink *inst;
	LL_FOREACH(::_mavlink_instances, inst) {
		if (instance == inst->get_instance_id()) {
			return inst;
		}
	}

	return nullptr;
}

Mavlink *
Mavlink::get_instance_for_device(const char *device_name)
{
	Mavlink *inst;

	LL_FOREACH(::_mavlink_instances, inst) {
		if (strcmp(inst->_device_name, device_name) == 0) {
			return inst;
		}
	}

	return nullptr;
}

Mavlink *
Mavlink::get_instance_for_network_port(unsigned long port)
{
	Mavlink *inst;

	LL_FOREACH(::_mavlink_instances, inst) {
		if (inst->_network_port == port) {
			return inst;
		}
	}

	return nullptr;
}

int
Mavlink::destroy_all_instances()
{
	/* start deleting from the end */
	Mavlink *inst_to_del = nullptr;
	Mavlink *next_inst = ::_mavlink_instances;

	unsigned iterations = 0;

	PX4_INFO("waiting for instances to stop");

	while (next_inst != nullptr) {
		inst_to_del = next_inst;
		next_inst = inst_to_del->next;

		/* set flag to stop thread and wait for all threads to finish */
		inst_to_del->_task_should_exit = true;

		while (inst_to_del->_task_running) {
			printf(".");
			fflush(stdout);
			px4_usleep(10000);
			iterations++;

			if (iterations > 1000) {
				PX4_ERR("Couldn't stop all mavlink instances.");
				return PX4_ERROR;
			}
		}

	}

	//we know all threads have exited, so it's safe to manipulate the linked list and delete objects.
	while (_mavlink_instances) {
		inst_to_del = _mavlink_instances;
		LL_DELETE(_mavlink_instances, inst_to_del);
		delete inst_to_del;
	}

	printf("\n");
	PX4_INFO("all instances stopped");
	return OK;
}

int
Mavlink::get_status_all_instances(bool show_streams_status)
{
	Mavlink *inst = ::_mavlink_instances;

	unsigned iterations = 0;

	while (inst != nullptr) {

		printf("\ninstance #%u:\n", iterations);

		if (show_streams_status) {
			inst->display_status_streams();

		} else {
			inst->display_status();
		}

		/* move on */
		inst = inst->next;
		iterations++;
	}

	/* return an error if there are no instances */
	return (iterations == 0);
}

bool
Mavlink::instance_exists(const char *device_name, Mavlink *self)
{
	Mavlink *inst = ::_mavlink_instances;

	while (inst != nullptr) {

		/* don't compare with itself */
		if (inst != self && !strcmp(device_name, inst->_device_name)) {
			return true;
		}

		inst = inst->next;
	}

	return false;
}

void
Mavlink::forward_message(const mavlink_message_t *msg, Mavlink *self)
{
	Mavlink *inst;
	LL_FOREACH(_mavlink_instances, inst) {
		if (inst != self) {
			const mavlink_msg_entry_t *meta = mavlink_get_msg_entry(msg->msgid);

			int target_system_id = 0;
			int target_component_id = 233;

			// might be nullptr if message is unknown
			if (meta) {
				// Extract target system and target component if set
				if (meta->target_system_ofs != 0) {
					target_system_id = ((uint8_t *)msg)[meta->target_system_ofs];
				}

				if (meta->target_component_ofs != 0) {
					target_component_id = ((uint8_t *)msg)[meta->target_component_ofs];
				}
			}

			// Broadcast or addressing this system and not trying to talk
			// to the autopilot component -> pass on to other components
			if ((target_system_id == 0 || target_system_id == self->get_system_id())
			    && (target_component_id == 0 || target_component_id != self->get_component_id())
			    && !(!self->forward_heartbeats_enabled() && msg->msgid == MAVLINK_MSG_ID_HEARTBEAT)) {

				inst->pass_message(msg);
			}
		}
	}
}


int
Mavlink::get_uart_fd(unsigned index)
{
	Mavlink *inst = get_instance(index);

	if (inst) {
		return inst->get_uart_fd();
	}

	return -1;
}

int
Mavlink::get_uart_fd()
{
	return _uart_fd;
}

int
Mavlink::get_instance_id()
{
	return _instance_id;
}

mavlink_channel_t
Mavlink::get_channel()
{
	return _channel;
}

int Mavlink::get_system_id()
{
	return mavlink_system.sysid;
}

int Mavlink::get_component_id()
{
	return mavlink_system.compid;
}

int Mavlink::mavlink_open_uart(int baud, const char *uart_name, bool force_flow_control)
{
#ifndef B460800
#define B460800 460800
#endif

#ifndef B500000
#define B500000 500000
#endif

#ifndef B921600
#define B921600 921600
#endif

#ifndef B1000000
#define B1000000 1000000
#endif

	/* process baud rate */
	int speed;

	switch (baud) {
	case 0:      speed = B0;      break;

	case 50:     speed = B50;     break;

	case 75:     speed = B75;     break;

	case 110:    speed = B110;    break;

	case 134:    speed = B134;    break;

	case 150:    speed = B150;    break;

	case 200:    speed = B200;    break;

	case 300:    speed = B300;    break;

	case 600:    speed = B600;    break;

	case 1200:   speed = B1200;   break;

	case 1800:   speed = B1800;   break;

	case 2400:   speed = B2400;   break;

	case 4800:   speed = B4800;   break;

	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	case 460800: speed = B460800; break;

	case 500000: speed = B500000; break;

	case 921600: speed = B921600; break;

	case 1000000: speed = B1000000; break;

#ifdef B1500000

	case 1500000: speed = B1500000; break;
#endif

#ifdef B2000000

	case 2000000: speed = B2000000; break;
#endif

#ifdef B3000000

	case 3000000: speed = B3000000; break;
#endif

	default:
		PX4_ERR("Unsupported baudrate: %d\n\tsupported examples:\n\t9600, 19200, 38400, 57600\t\n115200\n230400\n460800\n500000\n921600\n1000000\n",
			baud);
		return -EINVAL;
	}

	/* back off 1800 ms to avoid running into the USB setup timing */
	while (_mode == MAVLINK_MODE_CONFIG &&
	       hrt_absolute_time() < 1800U * 1000U) {
		px4_usleep(50000);
	}

	/* open uart */
	_uart_fd = ::open(uart_name, O_RDWR | O_NOCTTY);

	/* if this is a config link, stay here and wait for it to open */
	if (_uart_fd < 0 && _mode == MAVLINK_MODE_CONFIG) {

		int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
		struct actuator_armed_s armed;

		/* get the system arming state and abort on arming */
		while (_uart_fd < 0) {

			/* abort if an arming topic is published and system is armed */
			bool updated = false;
			orb_check(armed_sub, &updated);

			if (updated) {
				/* the system is now providing arming status feedback.
				 * instead of timing out, we resort to abort bringing
				 * up the terminal.
				 */
				orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);

				if (armed.armed) {
					/* this is not an error, but we are done */
					orb_unsubscribe(armed_sub);
					return -1;
				}
			}

			px4_usleep(100000);
			_uart_fd = ::open(uart_name, O_RDWR | O_NOCTTY);
		}

		orb_unsubscribe(armed_sub);
	}

	if (_uart_fd < 0) {
		return _uart_fd;
	}

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;
	_is_usb_uart = false;

	/* Initialize the uart config */
	if ((termios_state = tcgetattr(_uart_fd, &uart_config)) < 0) {
		PX4_ERR("ERR GET CONF %s: %d\n", uart_name, termios_state);
		::close(_uart_fd);
		return -1;
	}

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* USB serial is indicated by /dev/ttyACM0*/
	if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/ttyACM1") != OK) {

		/* Set baud rate */
		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
			PX4_ERR("ERR SET BAUD %s: %d\n", uart_name, termios_state);
			::close(_uart_fd);
			return -1;
		}

	} else {
		_is_usb_uart = true;

		/* USB has no baudrate, but use a magic number for 'fast' */
		_baudrate = 2000000;

		set_telemetry_status_type(telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_USB);
	}

#if defined(__PX4_LINUX) || defined(__PX4_DARWIN) || defined(__PX4_CYGWIN)
	/* Put in raw mode */
	cfmakeraw(&uart_config);
#endif

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		PX4_WARN("ERR SET CONF %s\n", uart_name);
		::close(_uart_fd);
		return -1;
	}

	/*
	 * Setup hardware flow control. If the port has no RTS pin this call will fail,
	 * which is not an issue, but requires a separate call so we can fail silently.
	*/

	/* setup output flow control */
	if (enable_flow_control(force_flow_control ? FLOW_CONTROL_ON : FLOW_CONTROL_AUTO)) {
		PX4_WARN("hardware flow control not supported");
	}

	return _uart_fd;
}

int
Mavlink::enable_flow_control(enum FLOW_CONTROL_MODE mode)
{
	// We can't do this on USB - skip
	if (_is_usb_uart) {
		_flow_control_mode = FLOW_CONTROL_OFF;
		return OK;
	}

	struct termios uart_config;

	int ret = tcgetattr(_uart_fd, &uart_config);

	if (mode) {
		uart_config.c_cflag |= CRTSCTS;

	} else {
		uart_config.c_cflag &= ~CRTSCTS;

	}

	ret = tcsetattr(_uart_fd, TCSANOW, &uart_config);

	if (!ret) {
		_flow_control_mode = mode;
	}

	return ret;
}

int
Mavlink::set_hil_enabled(bool hil_enabled)
{
	int ret = OK;

	/* enable HIL (only on links with sufficient bandwidth) */
	if (hil_enabled && !_hil_enabled && _datarate > 5000) {
		_hil_enabled = true;
		ret = configure_stream("HIL_ACTUATOR_CONTROLS", 200.0f);
	}

	/* disable HIL */
	if (!hil_enabled && _hil_enabled) {
		_hil_enabled = false;
		ret = configure_stream("HIL_ACTUATOR_CONTROLS", 0.0f);
	}

	return ret;
}

unsigned
Mavlink::get_free_tx_buf()
{
	/*
	 * Check if the OS buffer is full and disable HW
	 * flow control if it continues to be full
	 */
	int buf_free = 0;

	// if we are using network sockets, return max length of one packet
	if (get_protocol() == UDP || get_protocol() == TCP) {
		return  1500;

	} else {
		// No FIONSPACE on Linux todo:use SIOCOUTQ  and queue size to emulate FIONSPACE
#if defined(__PX4_LINUX) || defined(__PX4_DARWIN) || defined(__PX4_CYGWIN)
		//Linux cp210x does not support TIOCOUTQ
		buf_free = 256;
#else
		(void) ioctl(_uart_fd, FIONSPACE, (unsigned long)&buf_free);
#endif

		if (_flow_control_mode == FLOW_CONTROL_AUTO && buf_free < FLOW_CONTROL_DISABLE_THRESHOLD) {
			/* Disable hardware flow control in FLOW_CONTROL_AUTO mode:
			 * if no successful write since a defined time
			 * and if the last try was not the last successful write
			 */
			if (_last_write_try_time != 0 &&
			    hrt_elapsed_time(&_last_write_success_time) > 500_ms &&
			    _last_write_success_time != _last_write_try_time) {

				enable_flow_control(FLOW_CONTROL_OFF);
			}
		}
	}

	return buf_free;
}

void
Mavlink::begin_send()
{
	// must protect the network buffer so other calls from receive_thread do not
	// mangle the message.
	pthread_mutex_lock(&_send_mutex);
}

int
Mavlink::send_packet()
{
	int ret = -1;

#if defined(CONFIG_NET) || defined(__PX4_POSIX)

	/* Only send packets if there is something in the buffer. */
	if (_network_buf_len == 0) {
		pthread_mutex_unlock(&_send_mutex);
		return 0;
	}

	if (get_protocol() == UDP) {

#ifdef CONFIG_NET

		if (_src_addr_initialized) {
#endif
			ret = sendto(_socket_fd, _network_buf, _network_buf_len, 0,
				     (struct sockaddr *)&_src_addr, sizeof(_src_addr));
#ifdef CONFIG_NET
		}

#endif

		/* resend message via broadcast if no valid connection exists */
		if ((_mode != MAVLINK_MODE_ONBOARD) && broadcast_enabled() &&
		    (!get_client_source_initialized()
		     || (hrt_elapsed_time(&_tstatus.heartbeat_time) > 3_s))) {

			if (!_broadcast_address_found) {
				find_broadcast_address();
			}

			if (_broadcast_address_found && _network_buf_len > 0) {

				int bret = sendto(_socket_fd, _network_buf, _network_buf_len, 0,
						  (struct sockaddr *)&_bcast_addr, sizeof(_bcast_addr));

				if (bret <= 0) {
					if (!_broadcast_failed_warned) {
						PX4_ERR("sending broadcast failed, errno: %d: %s", errno, strerror(errno));
						_broadcast_failed_warned = true;
					}

				} else {
					_broadcast_failed_warned = false;
				}
			}
		}

	} else if (get_protocol() == TCP) {
		/* not implemented, but possible to do so */
		PX4_ERR("TCP transport pending implementation");
	}

	_network_buf_len = 0;
#endif

	pthread_mutex_unlock(&_send_mutex);
	return ret;
}

void
Mavlink::send_bytes(const uint8_t *buf, unsigned packet_len)
{
	/* If the wait until transmit flag is on, only transmit after we've received messages.
	   Otherwise, transmit all the time. */
	if (!should_transmit()) {
		return;
	}

	_last_write_try_time = hrt_absolute_time();

	if (_mavlink_start_time == 0) {
		_mavlink_start_time = _last_write_try_time;
	}

	if (get_protocol() == SERIAL) {
		/* check if there is space in the buffer, let it overflow else */
		unsigned buf_free = get_free_tx_buf();

		if (buf_free < packet_len) {
			/* not enough space in buffer to send */
			count_txerrbytes(packet_len);
			return;
		}
	}

	size_t ret = -1;

	/* send message to UART */
	if (get_protocol() == SERIAL) {
		ret = ::write(_uart_fd, buf, packet_len);
	}

#if defined(CONFIG_NET) || defined(__PX4_POSIX)

	else {
		if (_network_buf_len + packet_len < sizeof(_network_buf) / sizeof(_network_buf[0])) {
			memcpy(&_network_buf[_network_buf_len], buf, packet_len);
			_network_buf_len += packet_len;

			ret = packet_len;
		}
	}

#endif

	if (ret != (size_t) packet_len) {
		count_txerrbytes(packet_len);

	} else {
		_last_write_success_time = _last_write_try_time;
		count_txbytes(packet_len);
	}
}

void
Mavlink::find_broadcast_address()
{
#if defined(__PX4_LINUX) || defined(__PX4_DARWIN) || defined(__PX4_CYGWIN)
	struct ifconf ifconf;
	int ret;

#if defined(__APPLE__) && defined(__MACH__)
	// On Mac, we can't determine the required buffer
	// size in advance, so we just use what tends to work.
	ifconf.ifc_len = 1024;
#else
	// On Linux, we can determine the required size of the
	// buffer first by providing NULL to ifc_req.
	ifconf.ifc_req = nullptr;
	ifconf.ifc_len = 0;

	ret = ioctl(_socket_fd, SIOCGIFCONF, &ifconf);

	if (ret != 0) {
		PX4_WARN("getting required buffer size failed");
		return;
	}

#endif

	PX4_DEBUG("need to allocate %d bytes", ifconf.ifc_len);

	// Allocate buffer.
	ifconf.ifc_req = (struct ifreq *)(new uint8_t[ifconf.ifc_len]);

	if (ifconf.ifc_req == nullptr) {
		PX4_ERR("Could not allocate ifconf buffer");
		return;
	}

	memset(ifconf.ifc_req, 0, ifconf.ifc_len);

	ret = ioctl(_socket_fd, SIOCGIFCONF, &ifconf);

	if (ret != 0) {
		PX4_ERR("getting network config failed");
		delete[] ifconf.ifc_req;
		return;
	}

	int offset = 0;
	// Later used to point to next network interface in buffer.
	struct ifreq *cur_ifreq = (struct ifreq *) & (((uint8_t *)ifconf.ifc_req)[offset]);

	// The ugly `for` construct is used because it allows to use
	// `continue` and `break`.
	for (;
	     offset < ifconf.ifc_len;
#if defined(__APPLE__) && defined(__MACH__)
	     // On Mac, to get to next entry in buffer, jump by the size of
	     // the interface name size plus whatever is greater, either the
	     // sizeof sockaddr or ifr_addr.sa_len.
	     offset += IF_NAMESIZE
		       + (sizeof(struct sockaddr) > cur_ifreq->ifr_addr.sa_len ?
			  sizeof(struct sockaddr) : cur_ifreq->ifr_addr.sa_len)
#else
	     // On Linux, it's much easier to traverse the buffer, every entry
	     // has the constant length.
	     offset += sizeof(struct ifreq)
#endif
	    ) {
		// Point to next network interface in buffer.
		cur_ifreq = (struct ifreq *) & (((uint8_t *)ifconf.ifc_req)[offset]);

		PX4_DEBUG("looking at %s", cur_ifreq->ifr_name);

		// ignore loopback network
		if (strcmp(cur_ifreq->ifr_name, "lo") == 0 ||
		    strcmp(cur_ifreq->ifr_name, "lo0") == 0 ||
		    strcmp(cur_ifreq->ifr_name, "lo1") == 0 ||
		    strcmp(cur_ifreq->ifr_name, "lo2") == 0) {
			PX4_DEBUG("skipping loopback");
			continue;
		}

		struct in_addr &sin_addr = ((struct sockaddr_in *)&cur_ifreq->ifr_addr)->sin_addr;

		// Accept network interfaces to local network only. This means it's an IP starting with:
		// 192./172./10.
		// Also see https://tools.ietf.org/html/rfc1918#section-3

		uint8_t first_byte = sin_addr.s_addr & 0xFF;

		if (first_byte != 192 && first_byte != 172 && first_byte != 10) {
			continue;
		}

		if (!_broadcast_address_found) {
			const struct in_addr netmask_addr = query_netmask_addr(_socket_fd, *cur_ifreq);
			const struct in_addr broadcast_addr = compute_broadcast_addr(sin_addr, netmask_addr);

			if (_interface_name && strstr(cur_ifreq->ifr_name, _interface_name) == nullptr) { continue; }

			PX4_INFO("using network interface %s, IP: %s", cur_ifreq->ifr_name, inet_ntoa(sin_addr));
			PX4_INFO("with netmask: %s", inet_ntoa(netmask_addr));
			PX4_INFO("and broadcast IP: %s", inet_ntoa(broadcast_addr));

			_bcast_addr.sin_family = AF_INET;
			_bcast_addr.sin_addr = broadcast_addr;

			_broadcast_address_found = true;

		} else {
			PX4_DEBUG("ignoring additional network interface %s, IP:  %s",
				  cur_ifreq->ifr_name, inet_ntoa(sin_addr));
		}
	}

#elif defined (CONFIG_NET) && defined (__PX4_NUTTX)
	int ret;

	PX4_INFO("using network interface");

	struct in_addr eth_addr;
	struct in_addr bc_addr;
	struct in_addr netmask_addr;
	ret = netlib_get_ipv4addr("eth0", &eth_addr);

	if (ret != 0) {
		PX4_ERR("getting network config failed");
		return;
	}

	ret = netlib_get_ipv4netmask("eth0", &netmask_addr);

	if (ret != 0) {
		PX4_ERR("getting network config failed");
		return;
	}

	PX4_INFO("ipv4addr IP: %s", inet_ntoa(eth_addr));
	PX4_INFO("netmask_addr IP: %s", inet_ntoa(netmask_addr));

	bc_addr.s_addr = eth_addr.s_addr | ~(netmask_addr.s_addr);

	if (!_broadcast_address_found) {
		PX4_INFO("using network interface %s, IP: %s", "eth0", inet_ntoa(eth_addr));

		//struct in_addr &bc_addr = ((struct sockaddr_in *)&bc_ifreq.ifr_broadaddr)->sin_addr;
		PX4_INFO("with broadcast IP: %s", inet_ntoa(bc_addr));

		_bcast_addr.sin_family = AF_INET;
		_bcast_addr.sin_addr = bc_addr;

		_broadcast_address_found = true;
	}

#endif

#if defined (__PX4_LINUX) || defined (__PX4_DARWIN) || (defined (CONFIG_NET) && defined (__PX4_NUTTX))

	if (_broadcast_address_found) {
		_bcast_addr.sin_port = htons(_remote_port);

		int broadcast_opt = 1;

		if (setsockopt(_socket_fd, SOL_SOCKET, SO_BROADCAST, &broadcast_opt, sizeof(broadcast_opt)) < 0) {
			PX4_WARN("setting broadcast permission failed");
		}

		_broadcast_address_not_found_warned = false;

	} else {
		if (!_broadcast_address_not_found_warned) {
			PX4_WARN("no broadcasting address found");
			_broadcast_address_not_found_warned = true;
		}
	}

#if defined (__PX4_LINUX) || defined (__PX4_DARWIN)
	delete[] ifconf.ifc_req;
#endif

#endif
}

#ifdef __PX4_POSIX
const in_addr
Mavlink::query_netmask_addr(const int socket_fd, const ifreq &ifreq)
{
	struct ifreq netmask_ifreq;
	memset(&netmask_ifreq, 0, sizeof(netmask_ifreq));
	strncpy(netmask_ifreq.ifr_name, ifreq.ifr_name, IF_NAMESIZE);
	ioctl(socket_fd, SIOCGIFNETMASK, &netmask_ifreq);

	return ((struct sockaddr_in *)&netmask_ifreq.ifr_addr)->sin_addr;
}

const in_addr
Mavlink::compute_broadcast_addr(const in_addr &host_addr, const in_addr &netmask_addr)
{
	struct in_addr broadcast_addr;
	broadcast_addr.s_addr = ~netmask_addr.s_addr | host_addr.s_addr;

	return broadcast_addr;
}
#endif

void
Mavlink::init_udp()
{
#if defined (__PX4_LINUX) || defined (__PX4_DARWIN) || defined(__PX4_CYGWIN) || defined(CONFIG_NET)

	PX4_DEBUG("Setting up UDP with port %d", _network_port);

	_myaddr.sin_family = AF_INET;
	_myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	_myaddr.sin_port = htons(_network_port);

	if ((_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		PX4_WARN("create socket failed: %s", strerror(errno));
		return;
	}

	if (bind(_socket_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
		PX4_WARN("bind failed: %s", strerror(errno));
		return;
	}

	/* set default target address, but not for onboard mode (will be set on first received packet) */
	if (!_src_addr_initialized) {
		_src_addr.sin_family = AF_INET;
		inet_aton("127.0.0.1", &_src_addr.sin_addr);
	}

	_src_addr.sin_port = htons(_remote_port);

#endif
}

void
Mavlink::handle_message(const mavlink_message_t *msg)
{
	/*
	 *  NOTE: this is called from the receiver thread
	 */

	if (get_forwarding_on()) {
		/* forward any messages to other mavlink instances */
		Mavlink::forward_message(msg, this);
	}
}

void
Mavlink::send_statustext_info(const char *string)
{
	mavlink_log_info(&_mavlink_log_pub, "%s", string);
}

void
Mavlink::send_statustext_critical(const char *string)
{
	mavlink_log_critical(&_mavlink_log_pub, "%s", string);
	PX4_ERR("%s", string);
}

void
Mavlink::send_statustext_emergency(const char *string)
{
	mavlink_log_emergency(&_mavlink_log_pub, "%s", string);
}

void Mavlink::send_autopilot_capabilites()
{
	struct vehicle_status_s status;

	MavlinkOrbSubscription *status_sub = this->add_orb_subscription(ORB_ID(vehicle_status));

	if (status_sub->update(&status)) {
		mavlink_autopilot_version_t msg = {};

		msg.capabilities = MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_MISSION_INT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_COMMAND_INT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_FTP;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_MAVLINK2;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_MISSION_FENCE;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_MISSION_RALLY;
		msg.flight_sw_version = px4_firmware_version();
		msg.middleware_sw_version = px4_firmware_version();
		msg.os_sw_version = px4_os_version();
		msg.board_version = px4_board_version();
		uint64_t fw_git_version_binary = px4_firmware_version_binary();
		memcpy(&msg.flight_custom_version, &fw_git_version_binary, sizeof(msg.flight_custom_version));
		memcpy(&msg.middleware_custom_version, &fw_git_version_binary, sizeof(msg.middleware_custom_version));
		uint64_t os_git_version_binary = px4_os_version_binary();
		memcpy(&msg.os_custom_version, &os_git_version_binary, sizeof(msg.os_custom_version));
#ifdef CONFIG_CDCACM_VENDORID
		msg.vendor_id = CONFIG_CDCACM_VENDORID;
#else
		msg.vendor_id = 0;
#endif
#ifdef CONFIG_CDCACM_PRODUCTID
		msg.product_id = CONFIG_CDCACM_PRODUCTID;
#else
		msg.product_id = 0;
#endif
		uuid_uint32_t uid;
		board_get_uuid32(uid);
		msg.uid = (((uint64_t)uid[PX4_CPU_UUID_WORD32_UNIQUE_M]) << 32) | uid[PX4_CPU_UUID_WORD32_UNIQUE_H];

#ifndef BOARD_HAS_NO_UUID
		px4_guid_t px4_guid;
		board_get_px4_guid(px4_guid);
		static_assert(sizeof(px4_guid_t) == sizeof(msg.uid2), "GUID byte length mismatch");
		memcpy(&msg.uid2, &px4_guid, sizeof(msg.uid2));
#endif /* BOARD_HAS_NO_UUID */

#ifdef CONFIG_ARCH_BOARD_PX4_SITL
		// To avoid that multiple SITL instances have the same UUID, we add the mavlink
		// system ID. We subtract 1, so that the first UUID remains unchanged given the
		// default system ID is 1.
		//
		// Note that the UUID show in `ver` will still be the same for all instances.
		msg.uid += mavlink_system.sysid - 1;
		msg.uid2[0] += mavlink_system.sysid - 1;
#endif /* CONFIG_ARCH_BOARD_PX4_SITL */
		mavlink_msg_autopilot_version_send_struct(get_channel(), &msg);
	}
}

void Mavlink::send_protocol_version()
{
	mavlink_protocol_version_t msg = {};

	msg.version = _protocol_version * 100;
	msg.min_version = 100;
	msg.max_version = 200;
	uint64_t mavlink_lib_git_version_binary = px4_mavlink_lib_version_binary();
	// TODO add when available
	//memcpy(&msg.spec_version_hash, &mavlink_spec_git_version_binary, sizeof(msg.spec_version_hash));
	memcpy(&msg.library_version_hash, &mavlink_lib_git_version_binary, sizeof(msg.library_version_hash));

	// Switch to MAVLink 2
	int curr_proto_ver = _protocol_version;
	set_proto_version(2);
	// Send response - if it passes through the link its fine to use MAVLink 2
	mavlink_msg_protocol_version_send_struct(get_channel(), &msg);
	// Reset to previous value
	set_proto_version(curr_proto_ver);
}

MavlinkOrbSubscription *Mavlink::add_orb_subscription(const orb_id_t topic, int instance, bool disable_sharing)
{
	if (!disable_sharing) {
		/* check if already subscribed to this topic */
		MavlinkOrbSubscription *sub;

		LL_FOREACH(_subscriptions, sub) {
			if (sub->get_topic() == topic && sub->get_instance() == instance) {
				/* already subscribed */
				return sub;
			}
		}
	}

	/* add new subscription */
	MavlinkOrbSubscription *sub_new = new MavlinkOrbSubscription(topic, instance);

	LL_APPEND(_subscriptions, sub_new);

	return sub_new;
}

int
Mavlink::configure_stream(const char *stream_name, const float rate)
{
	PX4_DEBUG("configure_stream(%s, %.3f)", stream_name, (double)rate);

	/* calculate interval in us, -1 means unlimited stream, 0 means disabled */
	int interval = 0;

	if (rate > 0.000001f) {
		interval = (1000000.0f / rate);

	} else if (rate < 0.0f) {
		interval = -1;
	}

	/* search if stream exists */
	MavlinkStream *stream;
	LL_FOREACH(_streams, stream) {
		if (strcmp(stream_name, stream->get_name()) == 0) {
			if (interval != 0) {
				/* set new interval */
				stream->set_interval(interval);

			} else {
				/* delete stream */
				LL_DELETE(_streams, stream);
				delete stream;
			}

			return OK;
		}
	}

	if (interval == 0) {
		/* stream was not active and is requested to be disabled, do nothing */
		return OK;
	}

	// search for stream with specified name in supported streams list
	// create new instance if found
	stream = create_mavlink_stream(stream_name, this);

	if (stream != nullptr) {
		stream->set_interval(interval);
		LL_APPEND(_streams, stream);

		return OK;
	}

	/* if we reach here, the stream list does not contain the stream */
	PX4_WARN("stream %s not found", stream_name);

	return PX4_ERROR;
}

void
Mavlink::configure_stream_threadsafe(const char *stream_name, const float rate)
{
	/* orb subscription must be done from the main thread,
	 * set _subscribe_to_stream and _subscribe_to_stream_rate fields
	 * which polled in mavlink main loop */
	if (!_task_should_exit) {
		/* wait for previous subscription completion */
		while (_subscribe_to_stream != nullptr) {
			px4_usleep(MAIN_LOOP_DELAY / 2);
		}

		/* copy stream name */
		unsigned n = strlen(stream_name) + 1;
		char *s = new char[n];
		strcpy(s, stream_name);

		/* set subscription task */
		_subscribe_to_stream_rate = rate;
		_subscribe_to_stream = s;

		/* wait for subscription */
		do {
			px4_usleep(MAIN_LOOP_DELAY / 2);
		} while (_subscribe_to_stream != nullptr);

		delete[] s;
	}
}

int
Mavlink::message_buffer_init(int size)
{

	_message_buffer.size = size;
	_message_buffer.write_ptr = 0;
	_message_buffer.read_ptr = 0;
	_message_buffer.data = (char *)malloc(_message_buffer.size);

	int ret;

	if (_message_buffer.data == nullptr) {
		ret = PX4_ERROR;
		_message_buffer.size = 0;

	} else {
		ret = OK;
	}

	return ret;
}

void
Mavlink::message_buffer_destroy()
{
	_message_buffer.size = 0;
	_message_buffer.write_ptr = 0;
	_message_buffer.read_ptr = 0;
	free(_message_buffer.data);
}

int
Mavlink::message_buffer_count()
{
	int n = _message_buffer.write_ptr - _message_buffer.read_ptr;

	if (n < 0) {
		n += _message_buffer.size;
	}

	return n;
}

int
Mavlink::message_buffer_is_empty()
{
	return _message_buffer.read_ptr == _message_buffer.write_ptr;
}


bool
Mavlink::message_buffer_write(const void *ptr, int size)
{
	// bytes available to write
	int available = _message_buffer.read_ptr - _message_buffer.write_ptr - 1;

	if (available < 0) {
		available += _message_buffer.size;
	}

	if (size > available) {
		// buffer overflow
		return false;
	}

	char *c = (char *) ptr;
	int n = _message_buffer.size - _message_buffer.write_ptr;	// bytes to end of the buffer

	if (n < size) {
		// message goes over end of the buffer
		memcpy(&(_message_buffer.data[_message_buffer.write_ptr]), c, n);
		_message_buffer.write_ptr = 0;

	} else {
		n = 0;
	}

	// now: n = bytes already written
	int p = size - n;	// number of bytes to write
	memcpy(&(_message_buffer.data[_message_buffer.write_ptr]), &(c[n]), p);
	_message_buffer.write_ptr = (_message_buffer.write_ptr + p) % _message_buffer.size;
	return true;
}

int
Mavlink::message_buffer_get_ptr(void **ptr, bool *is_part)
{
	// bytes available to read
	int available = _message_buffer.write_ptr - _message_buffer.read_ptr;

	if (available == 0) {
		return 0;	// buffer is empty
	}

	int n = 0;

	if (available > 0) {
		// read pointer is before write pointer, all available bytes can be read
		n = available;
		*is_part = false;

	} else {
		// read pointer is after write pointer, read bytes from read_ptr to end of the buffer
		n = _message_buffer.size - _message_buffer.read_ptr;
		*is_part = _message_buffer.write_ptr > 0;
	}

	*ptr = &(_message_buffer.data[_message_buffer.read_ptr]);
	return n;
}

void
Mavlink::message_buffer_mark_read(int n)
{
	_message_buffer.read_ptr = (_message_buffer.read_ptr + n) % _message_buffer.size;
}

void
Mavlink::pass_message(const mavlink_message_t *msg)
{
	if (_forwarding_on) {
		/* size is 8 bytes plus variable payload */
		int size = MAVLINK_NUM_NON_PAYLOAD_BYTES + msg->len;
		pthread_mutex_lock(&_message_buffer_mutex);
		message_buffer_write(msg, size);
		pthread_mutex_unlock(&_message_buffer_mutex);
	}
}

MavlinkShell *
Mavlink::get_shell()
{
	if (!_mavlink_shell) {
		_mavlink_shell = new MavlinkShell();

		if (!_mavlink_shell) {
			PX4_ERR("Failed to allocate a shell");

		} else {
			int ret = _mavlink_shell->start();

			if (ret != 0) {
				PX4_ERR("Failed to start shell (%i)", ret);
				delete _mavlink_shell;
				_mavlink_shell = nullptr;
			}
		}
	}

	return _mavlink_shell;
}

void
Mavlink::close_shell()
{
	if (_mavlink_shell) {
		delete _mavlink_shell;
		_mavlink_shell = nullptr;
	}
}

void
Mavlink::update_rate_mult()
{
	float const_rate = 0.0f;
	float rate = 0.0f;

	/* scale down rates if their theoretical bandwidth is exceeding the link bandwidth */
	MavlinkStream *stream;
	LL_FOREACH(_streams, stream) {
		if (stream->const_rate()) {
			const_rate += (stream->get_interval() > 0) ? stream->get_size_avg() * 1000000.0f / stream->get_interval() : 0;

		} else {
			rate += (stream->get_interval() > 0) ? stream->get_size_avg() * 1000000.0f / stream->get_interval() : 0;
		}
	}

	float mavlink_ulog_streaming_rate_inv = 1.0f;

	if (_mavlink_ulog) {
		mavlink_ulog_streaming_rate_inv = 1.f - _mavlink_ulog->current_data_rate();
	}

	/* scale up and down as the link permits */
	float bandwidth_mult = (float)(_datarate * mavlink_ulog_streaming_rate_inv - const_rate) / rate;

	/* if we do not have flow control, limit to the set data rate */
	if (!get_flow_control_enabled()) {
		bandwidth_mult = fminf(1.0f, bandwidth_mult);
	}

	float hardware_mult = 1.0f;

	/* scale down if we have a TX err rate suggesting link congestion */
	if (_tstatus.rate_txerr > 0.0f && !_radio_status_critical) {
		hardware_mult = (_tstatus.rate_tx) / (_tstatus.rate_tx + _tstatus.rate_txerr);

	} else if (_radio_status_available) {

		// check for RADIO_STATUS timeout and reset
		if (hrt_elapsed_time(&_rstatus.timestamp) > 5_s) {
			PX4_ERR("instance %d: RADIO_STATUS timeout", _instance_id);
			set_telemetry_status_type(telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_GENERIC);

			_radio_status_available = false;
			_radio_status_critical = false;
			_radio_status_mult = 1.0f;
		}

		hardware_mult *= _radio_status_mult;
	}

	/* pick the minimum from bandwidth mult and hardware mult as limit */
	_rate_mult = fminf(bandwidth_mult, hardware_mult);

	/* ensure the rate multiplier never drops below 5% so that something is always sent */
	_rate_mult = math::constrain(_rate_mult, 0.05f, 1.0f);
}

void
Mavlink::update_radio_status(const radio_status_s &radio_status)
{
	_rstatus = radio_status;
	set_telemetry_status_type(telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO);

	/* check hardware limits */
	_radio_status_available = true;
	_radio_status_critical = (radio_status.txbuf < RADIO_BUFFER_LOW_PERCENTAGE);

	if (radio_status.txbuf < RADIO_BUFFER_CRITICAL_LOW_PERCENTAGE) {
		/* this indicates link congestion, reduce rate by 20% */
		_radio_status_mult *= 0.80f;

	} else if (radio_status.txbuf < RADIO_BUFFER_LOW_PERCENTAGE) {
		/* this indicates link congestion, reduce rate by 2.5% */
		_radio_status_mult *= 0.975f;

	} else if (radio_status.txbuf > RADIO_BUFFER_HALF_PERCENTAGE) {
		/* this indicates spare bandwidth, increase by 2.5% */
		_radio_status_mult *= 1.025f;
	}
}

int
Mavlink::configure_streams_to_default(const char *configure_single_stream)
{
	int ret = 0;
	bool stream_configured = false;

	auto configure_stream_local =
	[&stream_configured, configure_single_stream, &ret, this](const char *stream_name, float rate) {
		if (!configure_single_stream || strcmp(configure_single_stream, stream_name) == 0) {
			int ret_local = configure_stream(stream_name, rate);

			if (ret_local != 0) {
				ret = ret_local;
			}

			stream_configured = true;
		}
	};

	const float unlimited_rate = -1.f;

	switch (_mode) {
	case MAVLINK_MODE_NORMAL:
		configure_stream_local("ADSB_VEHICLE", unlimited_rate);
		configure_stream_local("ALTITUDE", 1.0f);
		configure_stream_local("ATTITUDE", 20.0f);
		configure_stream_local("ATTITUDE_TARGET", 2.0f);
		configure_stream_local("CAMERA_IMAGE_CAPTURED", unlimited_rate);
		configure_stream_local("COLLISION", unlimited_rate);
		configure_stream_local("DEBUG", 1.0f);
		configure_stream_local("DEBUG_VECT", 1.0f);
		configure_stream_local("DEBUG_FLOAT_ARRAY", 1.0f);
		configure_stream_local("DISTANCE_SENSOR", 0.5f);
		configure_stream_local("ESTIMATOR_STATUS", 0.5f);
		configure_stream_local("EXTENDED_SYS_STATE", 1.0f);
		configure_stream_local("GLOBAL_POSITION_INT", 5.0f);
		configure_stream_local("GPS_RAW_INT", 1.0f);
		configure_stream_local("GPS2_RAW", 1.0f);
		configure_stream_local("HIGHRES_IMU", 1.5f);
		configure_stream_local("HOME_POSITION", 0.5f);
		configure_stream_local("LOCAL_POSITION_NED", 1.0f);
		configure_stream_local("NAMED_VALUE_FLOAT", 1.0f);
		configure_stream_local("NAV_CONTROLLER_OUTPUT", 1.5f);
		configure_stream_local("OPTICAL_FLOW_RAD", 1.0f);
		configure_stream_local("PING", 0.1f);
		configure_stream_local("POSITION_TARGET_LOCAL_NED", 1.5f);
		configure_stream_local("POSITION_TARGET_GLOBAL_INT", 1.5f);
		configure_stream_local("RC_CHANNELS", 5.0f);
		configure_stream_local("SERVO_OUTPUT_RAW_0", 1.0f);
		configure_stream_local("SYS_STATUS", 1.0f);
		configure_stream_local("TRAJECTORY_REPRESENTATION_WAYPOINTS", 5.0f);
		configure_stream_local("UTM_GLOBAL_POSITION", 1.0f);
		configure_stream_local("VFR_HUD", 4.0f);
		configure_stream_local("ODOMETRY", 3.0f);
		configure_stream_local("WIND_COV", 1.0f);
		configure_stream_local("ORBIT_EXECUTION_STATUS", 5.f);
		break;

	case MAVLINK_MODE_ONBOARD:
		configure_stream_local("ACTUATOR_CONTROL_TARGET0", 10.0f);
		configure_stream_local("ADSB_VEHICLE", unlimited_rate);
		configure_stream_local("ALTITUDE", 10.0f);
		configure_stream_local("ATTITUDE", 100.0f);
		configure_stream_local("ATTITUDE_QUATERNION", 100.0f);
		configure_stream_local("ATTITUDE_TARGET", 10.0f);
		configure_stream_local("CAMERA_CAPTURE", 2.0f);
		configure_stream_local("CAMERA_IMAGE_CAPTURED", unlimited_rate);
		configure_stream_local("CAMERA_TRIGGER", unlimited_rate);
		configure_stream_local("COLLISION", unlimited_rate);
		configure_stream_local("DEBUG", 10.0f);
		configure_stream_local("DEBUG_VECT", 10.0f);
		configure_stream_local("DEBUG_FLOAT_ARRAY", 10.0f);
		configure_stream_local("DISTANCE_SENSOR", 10.0f);
		configure_stream_local("ESTIMATOR_STATUS", 1.0f);
		configure_stream_local("EXTENDED_SYS_STATE", 5.0f);
		configure_stream_local("GLOBAL_POSITION_INT", 50.0f);
		configure_stream_local("GPS_RAW_INT", unlimited_rate);
		configure_stream_local("GPS2_RAW", unlimited_rate);
		configure_stream_local("HIGHRES_IMU", unlimited_rate);//configure_stream_local("RAW_IMU_CUSTOM", unlimited_rate);
		configure_stream_local("HOME_POSITION", 0.5f);
		configure_stream_local("LOCAL_POSITION_NED", 30.0f);
		configure_stream_local("NAMED_VALUE_FLOAT", 10.0f);
		configure_stream_local("NAV_CONTROLLER_OUTPUT", 10.0f);
		configure_stream_local("OPTICAL_FLOW_RAD", 10.0f);
		configure_stream_local("PING", 1.0f);
		configure_stream_local("POSITION_TARGET_GLOBAL_INT", 10.0f);
		configure_stream_local("POSITION_TARGET_LOCAL_NED", 10.0f);
		configure_stream_local("RC_CHANNELS", 20.0f);
		configure_stream_local("SCALED_IMU", 50.0f);
		configure_stream_local("SERVO_OUTPUT_RAW_0", 10.0f);
		configure_stream_local("SYS_STATUS", 5.0f);
		configure_stream_local("SYSTEM_TIME", 1.0f);
		configure_stream_local("TIMESYNC", 10.0f);
		configure_stream_local("TRAJECTORY_REPRESENTATION_WAYPOINTS", 5.0f);
		configure_stream_local("UTM_GLOBAL_POSITION", 1.0f);
		configure_stream_local("VFR_HUD", 10.0f);
		configure_stream_local("ODOMETRY", 30.0f);
		configure_stream_local("WIND_COV", 10.0f);
		configure_stream_local("ORBIT_EXECUTION_STATUS", 5.f);
		break;

	case MAVLINK_MODE_OSD:
		configure_stream_local("ALTITUDE", 1.0f);
		configure_stream_local("ATTITUDE", 25.0f);
		configure_stream_local("ATTITUDE_TARGET", 10.0f);
		configure_stream_local("ESTIMATOR_STATUS", 1.0f);
		configure_stream_local("EXTENDED_SYS_STATE", 1.0f);
		configure_stream_local("GLOBAL_POSITION_INT", 10.0f);
		configure_stream_local("GPS_RAW_INT", 1.0f);
		configure_stream_local("HOME_POSITION", 0.5f);
		configure_stream_local("RC_CHANNELS", 5.0f);
		configure_stream_local("SERVO_OUTPUT_RAW_0", 1.0f);
		configure_stream_local("SYS_STATUS", 5.0f);
		configure_stream_local("SYSTEM_TIME", 1.0f);
		configure_stream_local("VFR_HUD", 25.0f);
		configure_stream_local("WIND_COV", 2.0f);
		break;

	case MAVLINK_MODE_MAGIC:

	/* fallthrough */
	case MAVLINK_MODE_CUSTOM:
		//stream nothing
		break;

	case MAVLINK_MODE_CONFIG:
		// Enable a number of interesting streams we want via USB
		configure_stream_local("ACTUATOR_CONTROL_TARGET0", 30.0f);
		configure_stream_local("ADSB_VEHICLE", unlimited_rate);
		configure_stream_local("ALTITUDE", 10.0f);
		configure_stream_local("ATTITUDE", 50.0f);
		configure_stream_local("ATTITUDE_TARGET", 8.0f);
		configure_stream_local("ATTITUDE_QUATERNION", 100.0f);
		configure_stream_local("CAMERA_TRIGGER", unlimited_rate);
		configure_stream_local("CAMERA_IMAGE_CAPTURED", unlimited_rate);
		configure_stream_local("COLLISION", unlimited_rate);
		configure_stream_local("DEBUG", 50.0f);
		configure_stream_local("DEBUG_VECT", 50.0f);
		configure_stream_local("DEBUG_FLOAT_ARRAY", 50.0f);
		configure_stream_local("DISTANCE_SENSOR", 10.0f);
		configure_stream_local("GPS_RAW_INT", unlimited_rate);
		configure_stream_local("GPS2_RAW", unlimited_rate);
		configure_stream_local("ESTIMATOR_STATUS", 5.0f);
		configure_stream_local("EXTENDED_SYS_STATE", 2.0f);
		configure_stream_local("GLOBAL_POSITION_INT", 10.0f);
		configure_stream_local("HIGHRES_IMU", unlimited_rate);
		configure_stream_local("HOME_POSITION", 0.5f);
		configure_stream_local("LOCAL_POSITION_NED", 30.0f);
		configure_stream_local("MANUAL_CONTROL", 5.0f);
		configure_stream_local("NAMED_VALUE_FLOAT", 50.0f);
		configure_stream_local("NAV_CONTROLLER_OUTPUT", 10.0f);
		configure_stream_local("OPTICAL_FLOW_RAD", 10.0f);
		configure_stream_local("PING", 1.0f);
		configure_stream_local("POSITION_TARGET_GLOBAL_INT", 10.0f);
		configure_stream_local("RC_CHANNELS", 10.0f);
		configure_stream_local("SERVO_OUTPUT_RAW_0", 20.0f);
		configure_stream_local("SERVO_OUTPUT_RAW_1", 20.0f);
		configure_stream_local("SYS_STATUS", 1.0f);
		configure_stream_local("SYSTEM_TIME", 1.0f);
		configure_stream_local("TIMESYNC", 10.0f);
		configure_stream_local("UTM_GLOBAL_POSITION", 1.0f);
		configure_stream_local("VFR_HUD", 20.0f);
		configure_stream_local("ODOMETRY", 30.0f);
		configure_stream_local("WIND_COV", 10.0f);
		configure_stream_local("ORBIT_EXECUTION_STATUS", 5.f);
		break;

	case MAVLINK_MODE_IRIDIUM:
		configure_stream_local("HIGH_LATENCY2", 0.015f);
		break;

	case MAVLINK_MODE_MINIMAL:
		configure_stream_local("ALTITUDE", 0.5f);
		configure_stream_local("ATTITUDE", 10.0f);
		configure_stream_local("EXTENDED_SYS_STATE", 0.1f);
		configure_stream_local("GPS_RAW_INT", 0.5f);
		configure_stream_local("GLOBAL_POSITION_INT", 5.0f);
		configure_stream_local("HOME_POSITION", 0.1f);
		configure_stream_local("NAMED_VALUE_FLOAT", 1.0f);
		configure_stream_local("RC_CHANNELS", 0.5f);
		configure_stream_local("SYS_STATUS", 0.1f);
		configure_stream_local("VFR_HUD", 1.0f);
		break;

	default:
		ret = -1;
		break;
	}

	if (configure_single_stream && !stream_configured && strcmp(configure_single_stream, "HEARTBEAT") != 0) {
		// stream was not found, assume it is disabled by default
		return configure_stream(configure_single_stream, 0.f);
	}

	return ret;
}

int
Mavlink::task_main(int argc, char *argv[])
{
	int ch;
	_baudrate = 57600;
	_datarate = 0;
	_mode = MAVLINK_MODE_NORMAL;
	bool _force_flow_control = false;

	_interface_name = nullptr;

#ifdef __PX4_NUTTX
	/* the NuttX optarg handler does not
	 * ignore argv[0] like the POSIX handler
	 * does, nor does it deal with non-flag
	 * verbs well. So we remove the application
	 * name and the verb.
	 */
	argc -= 2;
	argv += 2;
#endif

	/* don't exit from getopt loop to leave getopt global variables in consistent state,
	 * set error flag instead */
	bool err_flag = false;
	int myoptind = 1;
	const char *myoptarg = nullptr;
#if defined(CONFIG_NET) || defined(__PX4_POSIX)
	char *eptr;
	int temp_int_arg;
#endif

	while ((ch = px4_getopt(argc, argv, "b:r:d:n:u:o:m:t:c:fwxz", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			if (px4_get_parameter_value(myoptarg, _baudrate) != 0) {
				PX4_ERR("baudrate parsing failed");
				err_flag = true;
			}

			if (_baudrate < 9600 || _baudrate > 3000000) {
				PX4_ERR("invalid baud rate '%s'", myoptarg);
				err_flag = true;
			}

			break;

		case 'r':
			if (px4_get_parameter_value(myoptarg, _datarate) != 0) {
				PX4_ERR("datarate parsing failed");
				err_flag = true;
			}

			if (_datarate > MAX_DATA_RATE) {
				PX4_ERR("invalid data rate '%s'", myoptarg);
				err_flag = true;
			}

			break;

		case 'd':
			_device_name = myoptarg;
			set_protocol(SERIAL);
			break;

		case 'n':
			_interface_name = myoptarg;
			break;

#if defined(CONFIG_NET) || defined(__PX4_POSIX)

		case 'u':
			temp_int_arg = strtoul(myoptarg, &eptr, 10);

			if (*eptr == '\0') {
				_network_port = temp_int_arg;
				set_protocol(UDP);

			} else {
				PX4_ERR("invalid data udp_port '%s'", myoptarg);
				err_flag = true;
			}

			break;

		case 'o':
			temp_int_arg = strtoul(myoptarg, &eptr, 10);

			if (*eptr == '\0') {
				_remote_port = temp_int_arg;
				set_protocol(UDP);

			} else {
				PX4_ERR("invalid remote udp_port '%s'", myoptarg);
				err_flag = true;
			}

			break;

		case 't':
			_src_addr.sin_family = AF_INET;

			if (inet_aton(myoptarg, &_src_addr.sin_addr)) {
				_src_addr_initialized = true;

			} else {
				PX4_ERR("invalid partner ip '%s'", myoptarg);
				err_flag = true;
			}

			break;

#if defined(CONFIG_NET_IGMP) && defined(CONFIG_NET_ROUTE)

		// multicast
		case 'c':
			_src_addr.sin_family = AF_INET;

			if (inet_aton(myoptarg, &_src_addr.sin_addr)) {
				_src_addr_initialized = true;

			} else {
				PX4_ERR("invalid partner ip '%s'", myoptarg);
				err_flag = true;
			}

			break;
#else

		case 'c':
			PX4_ERR("Multicast option is not supported on this platform");
			err_flag = true;
			break;
#endif
#else

		case 'u':
		case 'o':
		case 't':
			PX4_ERR("UDP options not supported on this platform");
			err_flag = true;
			break;
#endif

//		case 'e':
//			mavlink_link_termination_allowed = true;
//			break;

		case 'm': {

				int mode;

				if (px4_get_parameter_value(myoptarg, mode) == 0) {
					if (mode >= 0 && mode < (int)MAVLINK_MODE_COUNT) {
						_mode = (MAVLINK_MODE)mode;

					} else {
						PX4_ERR("invalid mode");
						err_flag = true;
					}

				} else {
					if (strcmp(myoptarg, "custom") == 0) {
						_mode = MAVLINK_MODE_CUSTOM;

					} else if (strcmp(myoptarg, "camera") == 0) {
						// left in here for compatibility
						_mode = MAVLINK_MODE_ONBOARD;

					} else if (strcmp(myoptarg, "onboard") == 0) {
						_mode = MAVLINK_MODE_ONBOARD;

					} else if (strcmp(myoptarg, "osd") == 0) {
						_mode = MAVLINK_MODE_OSD;

					} else if (strcmp(myoptarg, "magic") == 0) {
						_mode = MAVLINK_MODE_MAGIC;

					} else if (strcmp(myoptarg, "config") == 0) {
						_mode = MAVLINK_MODE_CONFIG;

					} else if (strcmp(myoptarg, "iridium") == 0) {
						_mode = MAVLINK_MODE_IRIDIUM;
						set_telemetry_status_type(telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_IRIDIUM);

					} else if (strcmp(myoptarg, "minimal") == 0) {
						_mode = MAVLINK_MODE_MINIMAL;

					} else {
						PX4_ERR("invalid mode");
						err_flag = true;
					}
				}

				break;
			}

		case 'f':
			_forwarding_on = true;
			break;

		case 'w':
			_wait_to_transmit = true;
			break;

		case 'x':
			_ftp_on = true;
			break;

		case 'z':
			_force_flow_control = true;
			break;

		default:
			err_flag = true;
			break;
		}
	}

	if (err_flag) {
		usage();
		return PX4_ERROR;
	}

	if (_datarate == 0) {
		/* convert bits to bytes and use 1/2 of bandwidth by default */
		_datarate = _baudrate / 20;
	}

	if (_datarate > MAX_DATA_RATE) {
		_datarate = MAX_DATA_RATE;
	}

	if (get_protocol() == SERIAL) {
		if (Mavlink::instance_exists(_device_name, this)) {
			PX4_ERR("%s already running", _device_name);
			return PX4_ERROR;
		}

		PX4_INFO("mode: %s, data rate: %d B/s on %s @ %dB",
			 mavlink_mode_str(_mode), _datarate, _device_name, _baudrate);

		/* flush stdout in case MAVLink is about to take it over */
		fflush(stdout);

		/* default values for arguments */
		_uart_fd = mavlink_open_uart(_baudrate, _device_name, _force_flow_control);

		if (_uart_fd < 0 && _mode != MAVLINK_MODE_CONFIG) {
			PX4_ERR("could not open %s", _device_name);
			return PX4_ERROR;

		} else if (_uart_fd < 0 && _mode == MAVLINK_MODE_CONFIG) {
			/* the config link is optional */
			return OK;
		}

	} else if (get_protocol() == UDP) {
		if (Mavlink::get_instance_for_network_port(_network_port) != nullptr) {
			PX4_ERR("port %d already occupied", _network_port);
			return PX4_ERROR;
		}

		PX4_INFO("mode: %s, data rate: %d B/s on udp port %hu remote port %hu",
			 mavlink_mode_str(_mode), _datarate, _network_port, _remote_port);
	}

	/* initialize send mutex */
	pthread_mutex_init(&_send_mutex, nullptr);

	/* if we are passing on mavlink messages, we need to prepare a buffer for this instance */
	if (_forwarding_on) {
		/* initialize message buffer if multiplexing is on.
		 * make space for two messages plus off-by-one space as we use the empty element
		 * marker ring buffer approach.
		 */
		if (OK != message_buffer_init(2 * sizeof(mavlink_message_t) + 1)) {
			PX4_ERR("msg buf alloc fail");
			return 1;
		}

		/* initialize message buffer mutex */
		pthread_mutex_init(&_message_buffer_mutex, nullptr);
	}

	MavlinkOrbSubscription *cmd_sub = add_orb_subscription(ORB_ID(vehicle_command), 0, true);
	MavlinkOrbSubscription *param_sub = add_orb_subscription(ORB_ID(parameter_update));
	uint64_t param_time = 0;
	MavlinkOrbSubscription *status_sub = add_orb_subscription(ORB_ID(vehicle_status));
	uint64_t status_time = 0;
	MavlinkOrbSubscription *ack_sub = add_orb_subscription(ORB_ID(vehicle_command_ack), 0, true);
	/* We don't want to miss the first advertise of an ACK, so we subscribe from the
	 * beginning and not just when the topic exists. */
	ack_sub->subscribe_from_beginning(true);
	cmd_sub->subscribe_from_beginning(true);

	/* command ack */
	orb_advert_t command_ack_pub = nullptr;

	MavlinkOrbSubscription *mavlink_log_sub = add_orb_subscription(ORB_ID(mavlink_log));

	struct vehicle_status_s status;
	status_sub->update(&status_time, &status);

	/* Activate sending the data by default (for the IRIDIUM mode it will be disabled after the first round of packages is sent)*/
	_transmitting_enabled = true;
	_transmitting_enabled_commanded = true;

	if (_mode == MAVLINK_MODE_IRIDIUM) {
		_transmitting_enabled_commanded = false;
	}

	/* add default streams depending on mode */
	if (_mode != MAVLINK_MODE_IRIDIUM) {

		/* HEARTBEAT is constant rate stream, rate never adjusted */
		configure_stream("HEARTBEAT", 1.0f);

		/* STATUSTEXT stream is like normal stream but gets messages from logbuffer instead of uORB */
		configure_stream("STATUSTEXT", 20.0f);

		/* COMMAND_LONG stream: use unlimited rate to send all commands */
		configure_stream("COMMAND_LONG");

	}

	if (configure_streams_to_default() != 0) {
		PX4_ERR("configure_streams_to_default() failed");
	}

	/* set main loop delay depending on data rate to minimize CPU overhead */
	_main_loop_delay = (MAIN_LOOP_DELAY * 1000) / _datarate;

	/* hard limit to 1000 Hz at max */
	if (_main_loop_delay < MAVLINK_MIN_INTERVAL) {
		_main_loop_delay = MAVLINK_MIN_INTERVAL;
	}

	/* hard limit to 100 Hz at least */
	if (_main_loop_delay > MAVLINK_MAX_INTERVAL) {
		_main_loop_delay = MAVLINK_MAX_INTERVAL;
	}

	/* now the instance is fully initialized and we can bump the instance count */
	LL_APPEND(_mavlink_instances, this);

	/* init socket if necessary */
	if (get_protocol() == UDP) {
		init_udp();
	}

	/* if the protocol is serial, we send the system version blindly */
	if (get_protocol() == SERIAL) {
		send_autopilot_capabilites();
	}

	/* start the MAVLink receiver last to avoid a race */
	MavlinkReceiver::receive_start(&_receive_thread, this);

	while (!_task_should_exit) {
		/* main loop */
		px4_usleep(_main_loop_delay);

		perf_begin(_loop_perf);

		hrt_abstime t = hrt_absolute_time();

		update_rate_mult();

		if (param_sub->update(&param_time, nullptr)) {
			mavlink_update_parameters();

#if defined(CONFIG_NET)

			if (_param_broadcast_mode.get() != BROADCAST_MODE_MULTICAST) {
				_src_addr_initialized = false;
			}

#endif
		}

		check_radio_config();

		if (status_sub->update(&status_time, &status)) {
			/* switch HIL mode if required */
			set_hil_enabled(status.hil_state == vehicle_status_s::HIL_STATE_ON);

			set_manual_input_mode_generation(status.rc_input_mode == vehicle_status_s::RC_IN_MODE_GENERATED);

			if (_mode == MAVLINK_MODE_IRIDIUM) {
				if (_transmitting_enabled &&
				    !status.high_latency_data_link_active &&
				    !_transmitting_enabled_commanded &&
				    (_first_heartbeat_sent)) {
					_transmitting_enabled = false;
					mavlink_and_console_log_info(&_mavlink_log_pub, "Disable transmitting with IRIDIUM mavlink on device %s", _device_name);

				} else if (!_transmitting_enabled && status.high_latency_data_link_active) {
					_transmitting_enabled = true;
					mavlink_and_console_log_info(&_mavlink_log_pub, "Enable transmitting with IRIDIUM mavlink on device %s", _device_name);
				}
			}
		}

		struct vehicle_command_s vehicle_cmd;

		if (cmd_sub->update_if_changed(&vehicle_cmd)) {
			if ((vehicle_cmd.command == vehicle_command_s::VEHICLE_CMD_CONTROL_HIGH_LATENCY) &&
			    (_mode == MAVLINK_MODE_IRIDIUM)) {
				if (vehicle_cmd.param1 > 0.5f) {
					if (!_transmitting_enabled) {
						mavlink_and_console_log_info(&_mavlink_log_pub, "Enable transmitting with IRIDIUM mavlink on device %s by command",
									     _device_name);
					}

					_transmitting_enabled = true;
					_transmitting_enabled_commanded = true;

				} else {
					if (_transmitting_enabled) {
						mavlink_and_console_log_info(&_mavlink_log_pub, "Disable transmitting with IRIDIUM mavlink on device %s by command",
									     _device_name);
					}

					_transmitting_enabled = false;
					_transmitting_enabled_commanded = false;
				}

				// send positive command ack
				vehicle_command_ack_s command_ack = {};
				command_ack.timestamp = vehicle_cmd.timestamp;
				command_ack.command = vehicle_cmd.command;
				command_ack.result = vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED;
				command_ack.from_external = !vehicle_cmd.from_external;
				command_ack.target_system = vehicle_cmd.source_system;
				command_ack.target_component = vehicle_cmd.source_component;

				if (command_ack_pub != nullptr) {
					orb_publish(ORB_ID(vehicle_command_ack), command_ack_pub, &command_ack);

				} else {
					command_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &command_ack,
									      vehicle_command_ack_s::ORB_QUEUE_LENGTH);
				}
			}
		}

		/* send command ACK */
		uint16_t current_command_ack = 0;
		struct vehicle_command_ack_s command_ack;

		if (ack_sub->update_if_changed(&command_ack)) {
			if (!command_ack.from_external) {
				mavlink_command_ack_t msg;
				msg.result = command_ack.result;
				msg.command = command_ack.command;
				msg.progress = command_ack.result_param1;
				msg.result_param2 = command_ack.result_param2;
				msg.target_system = command_ack.target_system;
				msg.target_component = command_ack.target_component;
				current_command_ack = command_ack.command;

				// TODO: always transmit the acknowledge once it is only sent over the instance the command is received
				//bool _transmitting_enabled_temp = _transmitting_enabled;
				//_transmitting_enabled = true;
				mavlink_msg_command_ack_send_struct(get_channel(), &msg);
				//_transmitting_enabled = _transmitting_enabled_temp;
			}
		}

		struct mavlink_log_s mavlink_log;

		if (mavlink_log_sub->update_if_changed(&mavlink_log)) {
			_logbuffer.put(&mavlink_log);
		}

		/* check for shell output */
		if (_mavlink_shell && _mavlink_shell->available() > 0) {
			if (get_free_tx_buf() >= MAVLINK_MSG_ID_SERIAL_CONTROL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
				mavlink_serial_control_t msg;
				msg.baudrate = 0;
				msg.flags = SERIAL_CONTROL_FLAG_REPLY;
				msg.timeout = 0;
				msg.device = SERIAL_CONTROL_DEV_SHELL;
				msg.count = _mavlink_shell->read(msg.data, sizeof(msg.data));
				mavlink_msg_serial_control_send_struct(get_channel(), &msg);
			}
		}

		/* check for ulog streaming messages */
		if (_mavlink_ulog) {
			if (_mavlink_ulog_stop_requested) {
				_mavlink_ulog->stop();
				_mavlink_ulog = nullptr;
				_mavlink_ulog_stop_requested = false;

			} else {
				if (current_command_ack == vehicle_command_s::VEHICLE_CMD_LOGGING_START) {
					_mavlink_ulog->start_ack_received();
				}

				int ret = _mavlink_ulog->handle_update(get_channel());

				if (ret < 0) { //abort the streaming on error
					if (ret != -1) {
						PX4_WARN("mavlink ulog stream update failed, stopping (%i)", ret);
					}

					_mavlink_ulog->stop();
					_mavlink_ulog = nullptr;
				}
			}
		}

		/* check for requested subscriptions */
		if (_subscribe_to_stream != nullptr) {
			if (_subscribe_to_stream_rate < -1.5f) {
				if (configure_streams_to_default(_subscribe_to_stream) == 0) {
					if (get_protocol() == SERIAL) {
						PX4_DEBUG("stream %s on device %s set to default rate", _subscribe_to_stream, _device_name);

					} else if (get_protocol() == UDP) {
						PX4_DEBUG("stream %s on UDP port %d set to default rate", _subscribe_to_stream, _network_port);
					}

				} else {
					PX4_ERR("setting stream %s to default failed", _subscribe_to_stream);
				}

			} else if (configure_stream(_subscribe_to_stream, _subscribe_to_stream_rate) == 0) {
				if (fabsf(_subscribe_to_stream_rate) > 0.00001f) {
					if (get_protocol() == SERIAL) {
						PX4_DEBUG("stream %s on device %s enabled with rate %.1f Hz", _subscribe_to_stream, _device_name,
							  (double)_subscribe_to_stream_rate);

					} else if (get_protocol() == UDP) {
						PX4_DEBUG("stream %s on UDP port %d enabled with rate %.1f Hz", _subscribe_to_stream, _network_port,
							  (double)_subscribe_to_stream_rate);
					}

				} else {
					if (get_protocol() == SERIAL) {
						PX4_DEBUG("stream %s on device %s disabled", _subscribe_to_stream, _device_name);

					} else if (get_protocol() == UDP) {
						PX4_DEBUG("stream %s on UDP port %d disabled", _subscribe_to_stream, _network_port);
					}
				}

			} else {
				if (get_protocol() == SERIAL) {
					PX4_ERR("stream %s on device %s not found", _subscribe_to_stream, _device_name);

				} else if (get_protocol() == UDP) {
					PX4_ERR("stream %s on UDP port %d not found", _subscribe_to_stream, _network_port);
				}
			}

			_subscribe_to_stream = nullptr;
		}

		/* update streams */
		MavlinkStream *stream;
		LL_FOREACH(_streams, stream) {
			stream->update(t);

			if (!_first_heartbeat_sent) {
				if (_mode == MAVLINK_MODE_IRIDIUM) {
					if (stream->get_id() == MAVLINK_MSG_ID_HIGH_LATENCY2) {
						_first_heartbeat_sent = stream->first_message_sent();
					}

				} else {
					if (stream->get_id() == MAVLINK_MSG_ID_HEARTBEAT) {
						_first_heartbeat_sent = stream->first_message_sent();
					}
				}
			}
		}

		/* pass messages from other UARTs */
		if (_forwarding_on) {

			bool is_part;
			uint8_t *read_ptr;
			uint8_t *write_ptr;

			pthread_mutex_lock(&_message_buffer_mutex);
			int available = message_buffer_get_ptr((void **)&read_ptr, &is_part);
			pthread_mutex_unlock(&_message_buffer_mutex);

			if (available > 0) {
				// Reconstruct message from buffer

				mavlink_message_t msg;
				write_ptr = (uint8_t *)&msg;

				// Pull a single message from the buffer
				size_t read_count = available;

				if (read_count > sizeof(mavlink_message_t)) {
					read_count = sizeof(mavlink_message_t);
				}

				memcpy(write_ptr, read_ptr, read_count);

				// We hold the mutex until after we complete the second part of the buffer. If we don't
				// we may end up breaking the empty slot overflow detection semantics when we mark the
				// possibly partial read below.
				pthread_mutex_lock(&_message_buffer_mutex);

				message_buffer_mark_read(read_count);

				/* write second part of buffer if there is some */
				if (is_part && read_count < sizeof(mavlink_message_t)) {
					write_ptr += read_count;
					available = message_buffer_get_ptr((void **)&read_ptr, &is_part);
					read_count = sizeof(mavlink_message_t) - read_count;
					memcpy(write_ptr, read_ptr, read_count);
					message_buffer_mark_read(available);
				}

				pthread_mutex_unlock(&_message_buffer_mutex);

				resend_message(&msg);
			}
		}

		/* update TX/RX rates*/
		if (t > _bytes_timestamp + 1000000) {
			if (_bytes_timestamp != 0) {
				const float dt = (t - _bytes_timestamp) / 1000.0f;

				_tstatus.rate_tx = _bytes_tx / dt;
				_tstatus.rate_txerr = _bytes_txerr / dt;
				_tstatus.rate_rx = _bytes_rx / dt;

				_bytes_tx = 0;
				_bytes_txerr = 0;
				_bytes_rx = 0;
			}

			_bytes_timestamp = t;
		}

		// publish status at 1 Hz, or sooner if HEARTBEAT has updated
		if ((hrt_elapsed_time(&_tstatus.timestamp) >= 1_s) || (_tstatus.timestamp < _tstatus.heartbeat_time)) {
			publish_telemetry_status();
		}

		perf_end(_loop_perf);

		/* confirm task running only once fully initialized */
		_task_running = true;
	}

	/* first wait for threads to complete before tearing down anything */
	pthread_join(_receive_thread, nullptr);

	delete _subscribe_to_stream;
	_subscribe_to_stream = nullptr;

	/* delete streams */
	MavlinkStream *stream_to_del = nullptr;
	MavlinkStream *stream_next = _streams;

	while (stream_next != nullptr) {
		stream_to_del = stream_next;
		stream_next = stream_to_del->next;
		delete stream_to_del;
	}

	_streams = nullptr;

	/* delete subscriptions */
	MavlinkOrbSubscription *sub_to_del = nullptr;
	MavlinkOrbSubscription *sub_next = _subscriptions;

	while (sub_next != nullptr) {
		sub_to_del = sub_next;
		sub_next = sub_to_del->next;
		delete sub_to_del;
	}

	_subscriptions = nullptr;

	if (_uart_fd >= 0 && !_is_usb_uart) {
		/* close UART */
		::close(_uart_fd);
	}

	if (_socket_fd >= 0) {
		close(_socket_fd);
		_socket_fd = -1;
	}

	if (_forwarding_on) {
		message_buffer_destroy();
		pthread_mutex_destroy(&_message_buffer_mutex);
	}

	if (_mavlink_ulog) {
		_mavlink_ulog->stop();
		_mavlink_ulog = nullptr;
	}

	PX4_INFO("exiting channel %i", (int)_channel);

	return OK;
}

void Mavlink::publish_telemetry_status()
{
	// many fields are populated in place

	_tstatus.mode = _mode;
	_tstatus.data_rate = _datarate;
	_tstatus.rate_multiplier = _rate_mult;
	_tstatus.flow_control = get_flow_control_enabled();
	_tstatus.ftp = ftp_enabled();
	_tstatus.forwarding = get_forwarding_on();
	_tstatus.mavlink_v2 = (_protocol_version == 2);

	int num_streams = 0;

	MavlinkStream *stream;
	LL_FOREACH(_streams, stream) {
		// count
		num_streams++;
	}

	_tstatus.streams = num_streams;

	_tstatus.timestamp = hrt_absolute_time();
	int instance;
	orb_publish_auto(ORB_ID(telemetry_status), &_telem_status_pub, &_tstatus, &instance, ORB_PRIO_DEFAULT);
}

void Mavlink::check_radio_config()
{
	/* radio config check */
	if (_uart_fd >= 0 && _param_radio_id.get() != 0
	    && _tstatus.type == telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO) {
		/* request to configure radio and radio is present */
		FILE *fs = fdopen(_uart_fd, "w");

		if (fs) {
			/* switch to AT command mode */
			px4_usleep(1200000);
			fprintf(fs, "+++\n");
			px4_usleep(1200000);

			if (_param_radio_id.get() > 0) {
				/* set channel */
				fprintf(fs, "ATS3=%u\n", _param_radio_id.get());
				px4_usleep(200000);

			} else {
				/* reset to factory defaults */
				fprintf(fs, "AT&F\n");
				px4_usleep(200000);
			}

			/* write config */
			fprintf(fs, "AT&W");
			px4_usleep(200000);

			/* reboot */
			fprintf(fs, "ATZ");
			px4_usleep(200000);

			// XXX NuttX suffers from a bug where
			// fclose() also closes the fd, not just
			// the file stream. Since this is a one-time
			// config thing, we leave the file struct
			// allocated.
#ifndef __PX4_NUTTX
			fclose(fs);
#endif

		} else {
			PX4_WARN("open fd %d failed", _uart_fd);
		}

		/* reset param and save */
		_param_radio_id.set(0);
		_param_radio_id.commit_no_notification();
	}
}

int Mavlink::start_helper(int argc, char *argv[])
{
	/* create the instance in task context */
	Mavlink *instance = new Mavlink();

	int res;

	if (!instance) {

		/* out of memory */
		res = -ENOMEM;
		PX4_ERR("OUT OF MEM");

	} else {
		/* this will actually only return once MAVLink exits */
		res = instance->task_main(argc, argv);
		instance->_task_running = false;

	}

	return res;
}

int
Mavlink::start(int argc, char *argv[])
{
	MavlinkULog::initialize();
	MavlinkCommandSender::initialize();

	// Wait for the instance count to go up one
	// before returning to the shell
	int ic = Mavlink::instance_count();

	if (ic == Mavlink::MAVLINK_MAX_INSTANCES) {
		PX4_ERR("Maximum MAVLink instance count of %d reached.",
			(int)Mavlink::MAVLINK_MAX_INSTANCES);
		return 1;
	}

	// Instantiate thread
	char buf[24];
	sprintf(buf, "mavlink_if%d", ic);

	// This is where the control flow splits
	// between the starting task and the spawned
	// task - start_helper() only returns
	// when the started task exits.
	px4_task_spawn_cmd(buf,
			   SCHED_DEFAULT,
			   SCHED_PRIORITY_DEFAULT,
			   2650 + MAVLINK_NET_ADDED_STACK,
			   (px4_main_t)&Mavlink::start_helper,
			   (char *const *)argv);

	// Ensure that this shell command
	// does not return before the instance
	// is fully initialized. As this is also
	// the only path to create a new instance,
	// this is effectively a lock on concurrent
	// instance starting. XXX do a real lock.

	// Sleep 500 us between each attempt
	const unsigned sleeptime = 500;

	// Wait 100 ms max for the startup.
	const unsigned limit = 100 * 1000 / sleeptime;

	unsigned count = 0;

	while (ic == Mavlink::instance_count() && count < limit) {
		px4_usleep(sleeptime);
		count++;
	}

	if (ic == Mavlink::instance_count()) {
		return PX4_ERROR;

	} else {
		return PX4_OK;
	}
}

void
Mavlink::display_status()
{
	if (_tstatus.heartbeat_time > 0) {
		printf("\tGCS heartbeat:\t%llu us ago\n", (unsigned long long)hrt_elapsed_time(&_tstatus.heartbeat_time));
	}

	printf("\tmavlink chan: #%u\n", _channel);

	if (_tstatus.timestamp > 0) {

		printf("\ttype:\t\t");

		switch (_tstatus.type) {
		case telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO:
			printf("3DR RADIO\n");
			printf("\t  rssi:\t\t%d\n", _rstatus.rssi);
			printf("\t  remote rssi:\t%u\n", _rstatus.remote_rssi);
			printf("\t  txbuf:\t%u\n", _rstatus.txbuf);
			printf("\t  noise:\t%d\n", _rstatus.noise);
			printf("\t  remote noise:\t%u\n", _rstatus.remote_noise);
			printf("\t  rx errors:\t%u\n", _rstatus.rxerrors);
			printf("\t  fixed:\t%u\n", _rstatus.fix);
			break;

		case telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_USB:
			printf("USB CDC\n");
			break;

		default:
			printf("GENERIC LINK OR RADIO\n");
			break;
		}

	} else {
		printf("\tno radio status.\n");
	}

	printf("\tflow control: %s\n", _flow_control_mode ? "ON" : "OFF");
	printf("\trates:\n");
	printf("\t  tx: %.3f kB/s\n", (double)_tstatus.rate_tx);
	printf("\t  txerr: %.3f kB/s\n", (double)_tstatus.rate_txerr);
	printf("\t  tx rate mult: %.3f\n", (double)_rate_mult);
	printf("\t  tx rate max: %i B/s\n", _datarate);
	printf("\t  rx: %.3f kB/s\n", (double)_tstatus.rate_rx);

	if (_mavlink_ulog) {
		printf("\tULog rate: %.1f%% of max %.1f%%\n", (double)_mavlink_ulog->current_data_rate() * 100.,
		       (double)_mavlink_ulog->maximum_data_rate() * 100.);
	}

	printf("\tFTP enabled: %s, TX enabled: %s\n",
	       _ftp_on ? "YES" : "NO",
	       _transmitting_enabled ? "YES" : "NO");
	printf("\tmode: %s\n", mavlink_mode_str(_mode));
	printf("\tMAVLink version: %i\n", _protocol_version);

	printf("\ttransport protocol: ");

	switch (_protocol) {
	case UDP:
		printf("UDP (%i, remote port: %i)\n", _network_port, _remote_port);
#ifdef __PX4_POSIX

		if (get_client_source_initialized()) {
			printf("\tpartner IP: %s\n", inet_ntoa(get_client_source_address().sin_addr));
		}

#endif
		break;

	case TCP:
		printf("TCP\n");
		break;

	case SERIAL:
		printf("serial (%s @%i)\n", _device_name, _baudrate);
		break;
	}

	if (_ping_stats.last_ping_time > 0) {
		printf("\tping statistics:\n");
		printf("\t  last: %0.2f ms\n", (double)_ping_stats.last_rtt);
		printf("\t  mean: %0.2f ms\n", (double)_ping_stats.mean_rtt);
		printf("\t  max: %0.2f ms\n", (double)_ping_stats.max_rtt);
		printf("\t  min: %0.2f ms\n", (double)_ping_stats.min_rtt);
		printf("\t  dropped packets: %u\n", _ping_stats.dropped_packets);
	}
}

void
Mavlink::display_status_streams()
{
	printf("\t%-20s%-16s %s\n", "Name", "Rate Config (current) [Hz]", "Message Size (if active) [B]");

	const float rate_mult = _rate_mult;
	MavlinkStream *stream;
	LL_FOREACH(_streams, stream) {
		const int interval = stream->get_interval();
		const unsigned size = stream->get_size();
		char rate_str[20];

		if (interval < 0) {
			strcpy(rate_str, "unlimited");

		} else {
			float rate = 1000000.0f / (float)interval;
			// Note that the actual current rate can be lower if the associated uORB topic updates at a
			// lower rate.
			float rate_current = stream->const_rate() ? rate : rate * rate_mult;
			snprintf(rate_str, sizeof(rate_str), "%6.2f (%.3f)", (double)rate, (double)rate_current);
		}

		printf("\t%-30s%-16s", stream->get_name(), rate_str);

		if (size > 0) {
			printf(" %3i\n", size);

		} else {
			printf("\n");
		}
	}
}

int
Mavlink::stream_command(int argc, char *argv[])
{
	const char *device_name = DEFAULT_DEVICE_NAME;
	float rate = -1.0f;
	const char *stream_name = nullptr;
	unsigned short network_port = 0;
	char *eptr;
	int temp_int_arg;
	bool provided_device = false;
	bool provided_network_port = false;
	/*
	 * Called via main with original argv
	 *   mavlink start
	 *
	 *  Remove 2
	 */
	argc -= 2;
	argv += 2;

	/* don't exit from getopt loop to leave getopt global variables in consistent state,
	 * set error flag instead */
	bool err_flag = false;

	int i = 0;

	while (i < argc) {

		if (0 == strcmp(argv[i], "-r") && i < argc - 1) {
			rate = strtod(argv[i + 1], nullptr);

			if (rate < 0.0f) {
				err_flag = true;
			}

			i++;

		} else if (0 == strcmp(argv[i], "-d") && i < argc - 1) {
			provided_device = true;
			device_name = argv[i + 1];
			i++;

		} else if (0 == strcmp(argv[i], "-s") && i < argc - 1) {
			stream_name = argv[i + 1];
			i++;

		} else if (0 == strcmp(argv[i], "-u") && i < argc - 1) {
			provided_network_port = true;
			temp_int_arg = strtoul(argv[i + 1], &eptr, 10);

			if (*eptr == '\0') {
				network_port = temp_int_arg;

			} else {
				err_flag = true;
			}

			i++;

		} else {
			err_flag = true;
		}

		i++;
	}

	if (!err_flag && stream_name != nullptr) {

		Mavlink *inst = nullptr;

		if (provided_device && !provided_network_port) {
			inst = get_instance_for_device(device_name);

		} else if (provided_network_port && !provided_device) {
			inst = get_instance_for_network_port(network_port);

		} else if (provided_device && provided_network_port) {
			PX4_WARN("please provide either a device name or a network port");
			return 1;
		}

		if (rate < 0.f) {
			rate = -2.f; // use default rate
		}

		if (inst != nullptr) {
			inst->configure_stream_threadsafe(stream_name, rate);

		} else {

			// If the link is not running we should complain, but not fall over
			// because this is so easy to get wrong and not fatal. Warning is sufficient.
			if (provided_device) {
				PX4_WARN("mavlink for device %s is not running", device_name);

			} else {
				PX4_WARN("mavlink for network on port %hu is not running", network_port);
			}

			return 1;
		}

	} else {
		usage();
		return 1;
	}

	return OK;
}

void
Mavlink::set_boot_complete()
{
	_boot_complete = true;

#if defined(CONFIG_NET) || defined(__PX4_POSIX)
	Mavlink *inst;
	LL_FOREACH(::_mavlink_instances, inst) {
		if ((inst->get_mode() != MAVLINK_MODE_ONBOARD) &&
		    (!inst->broadcast_enabled()) &&
		    ((inst->get_protocol() == UDP) || (inst->get_protocol() == TCP))) {
			PX4_INFO("MAVLink only on localhost (set param MAV_BROADCAST = 1 to enable network)");
		}
	}
#endif

}

static void usage()
{

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module implements the MAVLink protocol, which can be used on a Serial link or UDP network connection.
It communicates with the system via uORB: some messages are directly handled in the module (eg. mission
protocol), others are published via uORB (eg. vehicle_command).

Streams are used to send periodic messages with a specific rate, such as the vehicle attitude.
When starting the mavlink instance, a mode can be specified, which defines the set of enabled streams with their rates.
For a running instance, streams can be configured via `mavlink stream` command.

There can be multiple independent instances of the module, each connected to one serial device or network port.

### Implementation
The implementation uses 2 threads, a sending and a receiving thread. The sender runs at a fixed rate and dynamically
reduces the rates of the streams if the combined bandwidth is higher than the configured rate (`-r`) or the
physical link becomes saturated. This can be checked with `mavlink status`, see if `rate mult` is less than 1.

**Careful**: some of the data is accessed and modified from both threads, so when changing code or extend the
functionality, this needs to be take into account, in order to avoid race conditions and corrupt data.

### Examples
Start mavlink on ttyS1 serial with baudrate 921600 and maximum sending rate of 80kB/s:
$ mavlink start -d /dev/ttyS1 -b 921600 -m onboard -r 80000

Start mavlink on UDP port 14556 and enable the HIGHRES_IMU message with 50Hz:
$ mavlink start -u 14556 -r 1000000
$ mavlink stream -u 14556 -s HIGHRES_IMU -r 50
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mavlink", "communication");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start a new instance");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS1", "<file:dev>", "Select Serial Device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 57600, 9600, 3000000, "Baudrate (can also be p:<param_name>)", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 10, 10000000, "Maximum sending data rate in B/s (if 0, use baudrate / 20)", true);
#if defined(CONFIG_NET) || defined(__PX4_POSIX)
	PRINT_MODULE_USAGE_PARAM_INT('u', 14556, 0, 65536, "Select UDP Network Port (local)", true);
	PRINT_MODULE_USAGE_PARAM_INT('o', 14550, 0, 65536, "Select UDP Network Port (remote)", true);
	PRINT_MODULE_USAGE_PARAM_STRING('t', "127.0.0.1", nullptr,
					"Partner IP (broadcasting can be enabled via MAV_BROADCAST param)", true);
#endif
	PRINT_MODULE_USAGE_PARAM_STRING('m', "normal", "custom|camera|onboard|osd|magic|config|iridium|minimal",
					"Mode: sets default streams and rates", true);
	PRINT_MODULE_USAGE_PARAM_STRING('n', nullptr, "<interface_name>", "wifi/ethernet interface name", true);
#if defined(CONFIG_NET_IGMP) && defined(CONFIG_NET_ROUTE)
	PRINT_MODULE_USAGE_PARAM_STRING('c', nullptr, "Multicast address in the range [239.0.0.0,239.255.255.255]", "Multicast address (multicasting can be enabled via MAV_BROADCAST param)", true);
#endif
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Enable message forwarding to other Mavlink instances", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('w', "Wait to send, until first message received", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('x', "Enable FTP", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('z', "Force flow control always on", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("stop-all", "Stop all instances");

	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print status for all instances");
	PRINT_MODULE_USAGE_ARG("streams", "Print all enabled streams", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("stream", "Configure the sending rate of a stream for a running instance");
#if defined(CONFIG_NET) || defined(__PX4_POSIX)
	PRINT_MODULE_USAGE_PARAM_INT('u', 0, 0, 65536, "Select Mavlink instance via local Network Port", true);
#endif
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Select Mavlink instance via Serial Device", true);
	PRINT_MODULE_USAGE_PARAM_STRING('s', nullptr, nullptr, "Mavlink stream to configure", false);
	PRINT_MODULE_USAGE_PARAM_FLOAT('r', -1.f, 0.f, 2000.f, "Rate in Hz (0 = turn off, -1 = set to default)", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("boot_complete",
					 "Enable sending of messages. (Must be) called as last step in startup script.");

}

int mavlink_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		return Mavlink::start(argc, argv);

	} else if (!strcmp(argv[1], "stop")) {
		PX4_WARN("mavlink stop is deprecated, use stop-all instead");
		usage();
		return 1;

	} else if (!strcmp(argv[1], "stop-all")) {
		return Mavlink::destroy_all_instances();

	} else if (!strcmp(argv[1], "status")) {
		bool show_streams_status = argc > 2 && strcmp(argv[2], "streams") == 0;
		return Mavlink::get_status_all_instances(show_streams_status);

	} else if (!strcmp(argv[1], "stream")) {
		return Mavlink::stream_command(argc, argv);

	} else if (!strcmp(argv[1], "boot_complete")) {
		Mavlink::set_boot_complete();
		return 0;

	} else {
		usage();
		return 1;
	}

	return 0;
}
/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file mavlink_messages.cpp
 * MAVLink 2.0 message formatters implementation.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "mavlink_main.h"
#include "mavlink_messages.h"
#include "mavlink_command_sender.h"
#include "mavlink_simple_analyzer.h"
#include "mavlink_high_latency2.h"

#include <commander/px4_custom_mode.h>
#include <drivers/drv_pwm_output.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <modules/vtol_att_control/euler_zxy.h>
#include <px4_time.h>
#include <systemlib/mavlink_log.h>

#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/camera_capture.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/orbit_status.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/mount_orientation.h>
#include <uORB/topics/collision_report.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/uORB.h>

using matrix::wrap_2pi;

static uint16_t cm_uint16_from_m_float(float m);

static void get_mavlink_mode_state(const struct vehicle_status_s *const status, uint8_t *mavlink_state,
				   uint8_t *mavlink_base_mode, uint32_t *mavlink_custom_mode);

uint16_t
cm_uint16_from_m_float(float m)
{
	if (m < 0.0f) {
		return 0;

	} else if (m > 655.35f) {
		return 65535;
	}

	return (uint16_t)(m * 100.0f);
}

void get_mavlink_navigation_mode(const struct vehicle_status_s *const status, uint8_t *mavlink_base_mode,
				 union px4_custom_mode *custom_mode)
{
	custom_mode->data = 0;
	*mavlink_base_mode = 0;

	/* HIL */
	if (status->hil_state == vehicle_status_s::HIL_STATE_ON) {
		*mavlink_base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
	}

	/* arming state */
	if (status->arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		*mavlink_base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}

	/* main state */
	*mavlink_base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	const uint8_t auto_mode_flags	= MAV_MODE_FLAG_AUTO_ENABLED
					  | MAV_MODE_FLAG_STABILIZE_ENABLED
					  | MAV_MODE_FLAG_GUIDED_ENABLED;

	switch (status->nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | (status->is_rotary_wing ? MAV_MODE_FLAG_STABILIZE_ENABLED : 0);
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_ACRO;
		break;

	case vehicle_status_s::NAVIGATION_STATE_RATTITUDE:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_RATTITUDE;
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED
					   | MAV_MODE_FLAG_GUIDED_ENABLED; // TODO: is POSCTL GUIDED?
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ORBIT:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
				      | MAV_MODE_FLAG_STABILIZE_ENABLED
				      | MAV_MODE_FLAG_GUIDED_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_POSCTL_ORBIT;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:

	/* fallthrough */
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL:

	/* fallthrough */
	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTGS;
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
		break;

	case vehicle_status_s::NAVIGATION_STATE_MAX:
		/* this is an unused case, ignore */
		break;

	}
}

void get_mavlink_mode_state(const struct vehicle_status_s *const status, uint8_t *mavlink_state,
			    uint8_t *mavlink_base_mode, uint32_t *mavlink_custom_mode)
{
	*mavlink_state = 0;
	*mavlink_base_mode = 0;
	*mavlink_custom_mode = 0;

	union px4_custom_mode custom_mode;
	get_mavlink_navigation_mode(status, mavlink_base_mode, &custom_mode);
	*mavlink_custom_mode = custom_mode.data;

	/* set system state */
	if (status->arming_state == vehicle_status_s::ARMING_STATE_INIT
	    || status->arming_state == vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE
	    || status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY_ERROR) {	// TODO review
		*mavlink_state = MAV_STATE_UNINIT;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		*mavlink_state = MAV_STATE_ACTIVE;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
		*mavlink_state = MAV_STATE_STANDBY;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_REBOOT) {
		*mavlink_state = MAV_STATE_POWEROFF;

	} else {
		*mavlink_state = MAV_STATE_CRITICAL;
	}
}


class MavlinkStreamHeartbeat : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamHeartbeat::get_name_static();
	}

	static const char *get_name_static()
	{
		return "HEARTBEAT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HEARTBEAT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHeartbeat(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_HEARTBEAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool const_rate()
	{
		return true;
	}

private:
	MavlinkOrbSubscription *_status_sub;

	/* do not allow top copying this class */
	MavlinkStreamHeartbeat(MavlinkStreamHeartbeat &) = delete;
	MavlinkStreamHeartbeat &operator = (const MavlinkStreamHeartbeat &) = delete;

protected:
	explicit MavlinkStreamHeartbeat(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status)))
	{}

	bool send(const hrt_abstime t)
	{
		struct vehicle_status_s status = {};

		/* always send the heartbeat, independent of the update status of the topics */
		if (!_status_sub->update(&status)) {
			/* if topic update failed fill it with defaults */
			memset(&status, 0, sizeof(status));
		}

		uint8_t base_mode = 0;
		uint32_t custom_mode = 0;
		uint8_t system_status = 0;
		get_mavlink_mode_state(&status, &system_status, &base_mode, &custom_mode);

		mavlink_msg_heartbeat_send(_mavlink->get_channel(), _mavlink->get_system_type(), MAV_AUTOPILOT_PX4,
					   base_mode, custom_mode, system_status);

		return true;
	}
};

class MavlinkStreamStatustext : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamStatustext::get_name_static();
	}

	static const char *get_name_static()
	{
		return "STATUSTEXT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_STATUSTEXT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamStatustext(mavlink);
	}

	unsigned get_size()
	{
		return _mavlink->get_logbuffer()->empty() ? 0 : (MAVLINK_MSG_ID_STATUSTEXT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

private:
	/* do not allow top copying this class */
	MavlinkStreamStatustext(MavlinkStreamStatustext &) = delete;
	MavlinkStreamStatustext &operator = (const MavlinkStreamStatustext &) = delete;

protected:
	explicit MavlinkStreamStatustext(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t)
	{
		if (!_mavlink->get_logbuffer()->empty() && _mavlink->is_connected()) {

			struct mavlink_log_s mavlink_log = {};

			if (_mavlink->get_logbuffer()->get(&mavlink_log)) {

				mavlink_statustext_t msg;
				msg.severity = mavlink_log.severity;
				strncpy(msg.text, (const char *)mavlink_log.text, sizeof(msg.text));
				msg.text[sizeof(msg.text) - 1] = '\0';

				mavlink_msg_statustext_send_struct(_mavlink->get_channel(), &msg);

				return true;
			}
		}

		return false;
	}
};

class MavlinkStreamCommandLong : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamCommandLong::get_name_static();
	}

	static const char *get_name_static()
	{
		return "COMMAND_LONG";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_COMMAND_LONG;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCommandLong(mavlink);
	}

	unsigned get_size()
	{
		return 0;	// commands stream is not regular and not predictable
	}

private:
	MavlinkOrbSubscription *_cmd_sub;

	/* do not allow top copying this class */
	MavlinkStreamCommandLong(MavlinkStreamCommandLong &) = delete;
	MavlinkStreamCommandLong &operator = (const MavlinkStreamCommandLong &) = delete;

protected:
	explicit MavlinkStreamCommandLong(Mavlink *mavlink) : MavlinkStream(mavlink),
		_cmd_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_command), 0, true))
	{}

	bool send(const hrt_abstime t)
	{
		struct vehicle_command_s cmd;
		bool sent = false;

		if (_cmd_sub->update_if_changed(&cmd)) {

			if (!cmd.from_external) {
				PX4_DEBUG("sending command %d to %d/%d", cmd.command, cmd.target_system, cmd.target_component);

				MavlinkCommandSender::instance().handle_vehicle_command(cmd, _mavlink->get_channel());
				sent = true;

			} else {
				PX4_DEBUG("not forwarding command %d to %d/%d", cmd.command, cmd.target_system, cmd.target_component);
			}
		}

		MavlinkCommandSender::instance().check_timeout(_mavlink->get_channel());

		return sent;
	}
};

class MavlinkStreamSysStatus : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamSysStatus::get_name_static();
	}

	static const char *get_name_static()
	{
		return "SYS_STATUS";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SYS_STATUS;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamSysStatus(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_SYS_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_status_sub;
	MavlinkOrbSubscription *_cpuload_sub;
	MavlinkOrbSubscription *_battery_status_sub;

	uint64_t _status_timestamp{0};
	uint64_t _cpuload_timestamp{0};
	uint64_t _battery_status_timestamp{0};

	/* do not allow top copying this class */
	MavlinkStreamSysStatus(MavlinkStreamSysStatus &) = delete;
	MavlinkStreamSysStatus &operator = (const MavlinkStreamSysStatus &) = delete;

protected:
	explicit MavlinkStreamSysStatus(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status))),
		_cpuload_sub(_mavlink->add_orb_subscription(ORB_ID(cpuload))),
		_battery_status_sub(_mavlink->add_orb_subscription(ORB_ID(battery_status)))
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_status_s status = {};
		cpuload_s cpuload = {};
		battery_status_s battery_status = {};

		const bool updated_status = _status_sub->update(&_status_timestamp, &status);
		const bool updated_cpuload = _cpuload_sub->update(&_cpuload_timestamp, &cpuload);
		const bool updated_battery = _battery_status_sub->update(&_battery_status_timestamp, &battery_status);

		if (updated_status || updated_battery || updated_cpuload) {
			mavlink_sys_status_t msg = {};

			msg.onboard_control_sensors_present = status.onboard_control_sensors_present;
			msg.onboard_control_sensors_enabled = status.onboard_control_sensors_enabled;
			msg.onboard_control_sensors_health = status.onboard_control_sensors_health;
			msg.load = cpuload.load * 1000.0f;
			msg.voltage_battery = (battery_status.connected) ? battery_status.voltage_filtered_v * 1000.0f : UINT16_MAX;
			msg.current_battery = (battery_status.connected) ? battery_status.current_filtered_a * 100.0f : -1;
			msg.battery_remaining = (battery_status.connected) ? ceilf(battery_status.remaining * 100.0f) : -1;
			// TODO: fill in something useful in the fields below
			msg.drop_rate_comm = 0;
			msg.errors_comm = 0;
			msg.errors_count1 = 0;
			msg.errors_count2 = 0;
			msg.errors_count3 = 0;
			msg.errors_count4 = 0;

			mavlink_msg_sys_status_send_struct(_mavlink->get_channel(), &msg);

			/* battery status message with higher resolution */
			mavlink_battery_status_t bat_msg = {};
			bat_msg.id = 0;
			bat_msg.battery_function = MAV_BATTERY_FUNCTION_ALL;
			bat_msg.type = MAV_BATTERY_TYPE_LIPO;
			bat_msg.current_consumed = (battery_status.connected) ? battery_status.discharged_mah : -1;
			bat_msg.energy_consumed = -1;
			bat_msg.current_battery = (battery_status.connected) ? battery_status.current_filtered_a * 100 : -1;
			bat_msg.battery_remaining = (battery_status.connected) ? ceilf(battery_status.remaining * 100.0f) : -1;
			bat_msg.temperature = (battery_status.connected) ? (int16_t)battery_status.temperature : INT16_MAX;
			//bat_msg.average_current_battery = (battery_status.connected) ? battery_status.average_current_a * 100.0f : -1;
			//bat_msg.serial_number = (battery_status.connected) ? battery_status.serial_number : 0;
			//bat_msg.capacity = (battery_status.connected) ? battery_status.capacity : 0;
			//bat_msg.cycle_count = (battery_status.connected) ? battery_status.cycle_count : UINT16_MAX;
			//bat_msg.run_time_to_empty = (battery_status.connected) ? battery_status.run_time_to_empty * 60 : 0;
			//bat_msg.average_time_to_empty = (battery_status.connected) ? battery_status.average_time_to_empty * 60 : 0;

			for (unsigned int i = 0; i < (sizeof(bat_msg.voltages) / sizeof(bat_msg.voltages[0])); i++) {
				if ((int)i < battery_status.cell_count && battery_status.connected) {
					bat_msg.voltages[i] = (battery_status.voltage_v / battery_status.cell_count) * 1000.0f;

				} else {
					bat_msg.voltages[i] = UINT16_MAX;
				}
			}

			mavlink_msg_battery_status_send_struct(_mavlink->get_channel(), &bat_msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamHighresIMU : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamHighresIMU::get_name_static();
	}

	static const char *get_name_static()
	{
		return "HIGHRES_IMU";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HIGHRES_IMU;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHighresIMU(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_HIGHRES_IMU_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_sensor_sub;
	uint64_t _sensor_time;

	MavlinkOrbSubscription *_bias_sub;
	MavlinkOrbSubscription *_differential_pressure_sub;
	MavlinkOrbSubscription *_magnetometer_sub;
	MavlinkOrbSubscription *_air_data_sub;

	uint64_t _accel_timestamp;
	uint64_t _gyro_timestamp;
	uint64_t _mag_timestamp;
	uint64_t _baro_timestamp;
	uint64_t _dpres_timestamp;

	/* do not allow top copying this class */
	MavlinkStreamHighresIMU(MavlinkStreamHighresIMU &) = delete;
	MavlinkStreamHighresIMU &operator = (const MavlinkStreamHighresIMU &) = delete;

protected:
	explicit MavlinkStreamHighresIMU(Mavlink *mavlink) : MavlinkStream(mavlink),
		_sensor_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_combined))),
		_sensor_time(0),
		_bias_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_bias))),
		_differential_pressure_sub(_mavlink->add_orb_subscription(ORB_ID(differential_pressure))),
		_magnetometer_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_magnetometer))),
		_air_data_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_air_data))),
		_accel_timestamp(0),
		_gyro_timestamp(0),
		_mag_timestamp(0),
		_baro_timestamp(0),
		_dpres_timestamp(0)
	{}

	bool send(const hrt_abstime t)
	{
		sensor_combined_s sensor;

		if (_sensor_sub->update(&_sensor_time, &sensor)) {
			uint16_t fields_updated = 0;

			if (_accel_timestamp != sensor.timestamp + sensor.accelerometer_timestamp_relative) {
				/* mark first three dimensions as changed */
				fields_updated |= (1 << 0) | (1 << 1) | (1 << 2);
				_accel_timestamp = sensor.timestamp + sensor.accelerometer_timestamp_relative;
			}

			if (_gyro_timestamp != sensor.timestamp) {
				/* mark second group dimensions as changed */
				fields_updated |= (1 << 3) | (1 << 4) | (1 << 5);
				_gyro_timestamp = sensor.timestamp;
			}

			vehicle_magnetometer_s magnetometer = {};
			_magnetometer_sub->update(&magnetometer);

			if (_mag_timestamp != magnetometer.timestamp) {
				/* mark third group dimensions as changed */
				fields_updated |= (1 << 6) | (1 << 7) | (1 << 8);
				_mag_timestamp = magnetometer.timestamp;
			}

			vehicle_air_data_s air_data = {};
			_air_data_sub->update(&air_data);

			if (_baro_timestamp != air_data.timestamp) {
				/* mark fourth group (baro fields) dimensions as changed */
				fields_updated |= (1 << 9) | (1 << 11) | (1 << 12);
				_baro_timestamp = air_data.timestamp;
			}

			sensor_bias_s bias = {};
			_bias_sub->update(&bias);

			differential_pressure_s differential_pressure = {};
			_differential_pressure_sub->update(&differential_pressure);

			if (_dpres_timestamp != differential_pressure.timestamp) {
				/* mark fourth group (dpres field) dimensions as changed */
				fields_updated |= (1 << 10);
				_dpres_timestamp = differential_pressure.timestamp;
			}

			mavlink_highres_imu_t msg = {};

			msg.time_usec = sensor.timestamp;
			msg.xacc = sensor.accelerometer_m_s2[0] - bias.accel_x_bias;
			msg.yacc = sensor.accelerometer_m_s2[1] - bias.accel_y_bias;
			msg.zacc = sensor.accelerometer_m_s2[2] - bias.accel_z_bias;
			msg.xgyro = sensor.gyro_rad[0] - bias.gyro_x_bias;
			msg.ygyro = sensor.gyro_rad[1] - bias.gyro_y_bias;
			msg.zgyro = sensor.gyro_rad[2] - bias.gyro_z_bias;
			msg.xmag = magnetometer.magnetometer_ga[0] - bias.mag_x_bias;
			msg.ymag = magnetometer.magnetometer_ga[1] - bias.mag_y_bias;
			msg.zmag = magnetometer.magnetometer_ga[2] - bias.mag_z_bias;
			msg.abs_pressure = air_data.baro_pressure_pa;
			msg.diff_pressure = differential_pressure.differential_pressure_raw_pa;
			msg.pressure_alt = air_data.baro_alt_meter;
			msg.temperature = air_data.baro_temp_celcius;
			msg.fields_updated = fields_updated;

			mavlink_msg_highres_imu_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamScaledIMU : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamScaledIMU::get_name_static();
	}

	static const char *get_name_static()
	{
		return "SCALED_IMU";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SCALED_IMU;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamScaledIMU(mavlink);
	}

	unsigned get_size()
	{
		return _raw_accel_sub->is_published() ? (MAVLINK_MSG_ID_SCALED_IMU_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_raw_accel_sub;
	MavlinkOrbSubscription *_raw_gyro_sub;
	MavlinkOrbSubscription *_raw_mag_sub;

	uint64_t _raw_accel_time;
	uint64_t _raw_gyro_time;
	uint64_t _raw_mag_time;

	// do not allow top copy this class
	MavlinkStreamScaledIMU(MavlinkStreamScaledIMU &) = delete;
	MavlinkStreamScaledIMU &operator = (const MavlinkStreamScaledIMU &) = delete;

protected:
	explicit MavlinkStreamScaledIMU(Mavlink *mavlink) : MavlinkStream(mavlink),
		_raw_accel_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_accel), 0)),
		_raw_gyro_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_gyro), 0)),
		_raw_mag_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_mag), 0)),
		_raw_accel_time(0),
		_raw_gyro_time(0),
		_raw_mag_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		sensor_accel_s sensor_accel = {};
		sensor_gyro_s sensor_gyro = {};
		sensor_mag_s sensor_mag = {};

		bool updated = false;
		updated |= _raw_accel_sub->update(&_raw_accel_time, &sensor_accel);
		updated |= _raw_gyro_sub->update(&_raw_gyro_time, &sensor_gyro);
		updated |= _raw_mag_sub->update(&_raw_mag_time, &sensor_mag);

		if (updated) {

			mavlink_scaled_imu_t msg = {};

			msg.time_boot_ms = sensor_accel.timestamp / 1000;
			msg.xacc = (int16_t)(sensor_accel.x_raw / CONSTANTS_ONE_G); 	// [milli g]
			msg.yacc = (int16_t)(sensor_accel.y_raw / CONSTANTS_ONE_G); 	// [milli g]
			msg.zacc = (int16_t)(sensor_accel.z_raw / CONSTANTS_ONE_G); 	// [milli g]
			msg.xgyro = sensor_gyro.x_raw;					// [milli rad/s]
			msg.ygyro = sensor_gyro.y_raw;					// [milli rad/s]
			msg.zgyro = sensor_gyro.z_raw;					// [milli rad/s]
			msg.xmag = sensor_mag.x_raw;					// [milli tesla]
			msg.ymag = sensor_mag.y_raw;					// [milli tesla]
			msg.zmag = sensor_mag.z_raw;					// [milli tesla]

			mavlink_msg_scaled_imu_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamScaledIMU2 : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamScaledIMU2::get_name_static();
	}

	static const char *get_name_static()
	{
		return "SCALED_IMU2";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SCALED_IMU2;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamScaledIMU2(mavlink);
	}

	unsigned get_size()
	{
		return _raw_accel_sub->is_published() ? (MAVLINK_MSG_ID_SCALED_IMU2_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_raw_accel_sub;
	MavlinkOrbSubscription *_raw_gyro_sub;
	MavlinkOrbSubscription *_raw_mag_sub;

	uint64_t _raw_accel_time;
	uint64_t _raw_gyro_time;
	uint64_t _raw_mag_time;

	// do not allow top copy this class
	MavlinkStreamScaledIMU2(MavlinkStreamScaledIMU2 &) = delete;
	MavlinkStreamScaledIMU2 &operator = (const MavlinkStreamScaledIMU2 &) = delete;

protected:
	explicit MavlinkStreamScaledIMU2(Mavlink *mavlink) : MavlinkStream(mavlink),
		_raw_accel_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_accel), 1)),
		_raw_gyro_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_gyro), 1)),
		_raw_mag_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_mag), 1)),
		_raw_accel_time(0),
		_raw_gyro_time(0),
		_raw_mag_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		sensor_accel_s sensor_accel = {};
		sensor_gyro_s sensor_gyro = {};
		sensor_mag_s sensor_mag = {};

		bool updated = false;
		updated |= _raw_accel_sub->update(&_raw_accel_time, &sensor_accel);
		updated |= _raw_gyro_sub->update(&_raw_gyro_time, &sensor_gyro);
		updated |= _raw_mag_sub->update(&_raw_mag_time, &sensor_mag);

		if (updated) {

			mavlink_scaled_imu2_t msg = {};

			msg.time_boot_ms = sensor_accel.timestamp / 1000;
			msg.xacc = (int16_t)(sensor_accel.x_raw / CONSTANTS_ONE_G); 	// [milli g]
			msg.yacc = (int16_t)(sensor_accel.y_raw / CONSTANTS_ONE_G); 	// [milli g]
			msg.zacc = (int16_t)(sensor_accel.z_raw / CONSTANTS_ONE_G); 	// [milli g]
			msg.xgyro = sensor_gyro.x_raw;					// [milli rad/s]
			msg.ygyro = sensor_gyro.y_raw;					// [milli rad/s]
			msg.zgyro = sensor_gyro.z_raw;					// [milli rad/s]
			msg.xmag = sensor_mag.x_raw;					// [milli tesla]
			msg.ymag = sensor_mag.y_raw;					// [milli tesla]
			msg.zmag = sensor_mag.z_raw;					// [milli tesla]

			mavlink_msg_scaled_imu2_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamScaledIMU3 : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamScaledIMU3::get_name_static();
	}

	static const char *get_name_static()
	{
		return "SCALED_IMU3";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SCALED_IMU3;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamScaledIMU3(mavlink);
	}

	unsigned get_size()
	{
		return _raw_accel_sub->is_published() ? (MAVLINK_MSG_ID_SCALED_IMU3_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_raw_accel_sub;
	MavlinkOrbSubscription *_raw_gyro_sub;
	MavlinkOrbSubscription *_raw_mag_sub;

	uint64_t _raw_accel_time;
	uint64_t _raw_gyro_time;
	uint64_t _raw_mag_time;

	// do not allow top copy this class
	MavlinkStreamScaledIMU3(MavlinkStreamScaledIMU3 &) = delete;
	MavlinkStreamScaledIMU3 &operator = (const MavlinkStreamScaledIMU3 &) = delete;

protected:
	explicit MavlinkStreamScaledIMU3(Mavlink *mavlink) : MavlinkStream(mavlink),
		_raw_accel_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_accel), 2)),
		_raw_gyro_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_gyro), 2)),
		_raw_mag_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_mag), 2)),
		_raw_accel_time(0),
		_raw_gyro_time(0),
		_raw_mag_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		sensor_accel_s sensor_accel = {};
		sensor_gyro_s sensor_gyro = {};
		sensor_mag_s sensor_mag = {};

		bool updated = false;
		updated |= _raw_accel_sub->update(&_raw_accel_time, &sensor_accel);
		updated |= _raw_gyro_sub->update(&_raw_gyro_time, &sensor_gyro);
		updated |= _raw_mag_sub->update(&_raw_mag_time, &sensor_mag);

		if (updated) {

			mavlink_scaled_imu3_t msg = {};

			msg.time_boot_ms = sensor_accel.timestamp / 1000;
			msg.xacc = (int16_t)(sensor_accel.x_raw / CONSTANTS_ONE_G); 	// [milli g]
			msg.yacc = (int16_t)(sensor_accel.y_raw / CONSTANTS_ONE_G); 	// [milli g]
			msg.zacc = (int16_t)(sensor_accel.z_raw / CONSTANTS_ONE_G); 	// [milli g]
			msg.xgyro = sensor_gyro.x_raw;					// [milli rad/s]
			msg.ygyro = sensor_gyro.y_raw;					// [milli rad/s]
			msg.zgyro = sensor_gyro.z_raw;					// [milli rad/s]
			msg.xmag = sensor_mag.x_raw;					// [milli tesla]
			msg.ymag = sensor_mag.y_raw;					// [milli tesla]
			msg.zmag = sensor_mag.z_raw;					// [milli tesla]

			mavlink_msg_scaled_imu3_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamAttitude : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamAttitude::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ATTITUDE";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ATTITUDE;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAttitude(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_ATTITUDE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_att_sub;
	uint64_t _att_time;

	/* do not allow top copying this class */
	MavlinkStreamAttitude(MavlinkStreamAttitude &) = delete;
	MavlinkStreamAttitude &operator = (const MavlinkStreamAttitude &) = delete;


protected:
	explicit MavlinkStreamAttitude(Mavlink *mavlink) : MavlinkStream(mavlink),
		_att_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_attitude))),
		_att_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_attitude_s att;

		if (_att_sub->update(&_att_time, &att)) {
			mavlink_attitude_t msg = {};
			//matrix::Eulerf euler = matrix::Quatf(att.q);
			matrix::Eulerf_zxy euler = matrix::Eulerf_zxy(matrix::Quatf(att.q));
			msg.time_boot_ms = att.timestamp / 1000;
			msg.roll = euler.phi();
			msg.pitch = euler.theta();
			msg.yaw = euler.psi();
			msg.rollspeed = att.rollspeed;
			msg.pitchspeed = att.pitchspeed;
			msg.yawspeed = att.yawspeed;

			mavlink_msg_attitude_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamAttitudeQuaternion : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamAttitudeQuaternion::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ATTITUDE_QUATERNION";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ATTITUDE_QUATERNION;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAttitudeQuaternion(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_att_sub;
	uint64_t _att_time;

	/* do not allow top copying this class */
	MavlinkStreamAttitudeQuaternion(MavlinkStreamAttitudeQuaternion &) = delete;
	MavlinkStreamAttitudeQuaternion &operator = (const MavlinkStreamAttitudeQuaternion &) = delete;

protected:
	explicit MavlinkStreamAttitudeQuaternion(Mavlink *mavlink) : MavlinkStream(mavlink),
		_att_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_attitude))),
		_att_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_attitude_s att;

		if (_att_sub->update(&_att_time, &att)) {
			mavlink_attitude_quaternion_t msg = {};

			msg.time_boot_ms = att.timestamp / 1000;
			msg.q1 = att.q[0];
			msg.q2 = att.q[1];
			msg.q3 = att.q[2];
			msg.q4 = att.q[3];
			msg.rollspeed = att.rollspeed;
			msg.pitchspeed = att.pitchspeed;
			msg.yawspeed = att.yawspeed;

			mavlink_msg_attitude_quaternion_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamVFRHUD : public MavlinkStream
{
public:

	const char *get_name() const
	{
		return MavlinkStreamVFRHUD::get_name_static();
	}

	static const char *get_name_static()
	{
		return "VFR_HUD";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_VFR_HUD;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamVFRHUD(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_VFR_HUD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_pos_sub;
	uint64_t _pos_time;

	MavlinkOrbSubscription *_armed_sub;
	uint64_t _armed_time;

	MavlinkOrbSubscription *_act0_sub;
	MavlinkOrbSubscription *_act1_sub;

	MavlinkOrbSubscription *_airspeed_sub;
	uint64_t _airspeed_time;

	MavlinkOrbSubscription *_air_data_sub;

	/* do not allow top copying this class */
	MavlinkStreamVFRHUD(MavlinkStreamVFRHUD &) = delete;
	MavlinkStreamVFRHUD &operator = (const MavlinkStreamVFRHUD &) = delete;

protected:
	explicit MavlinkStreamVFRHUD(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position))),
		_pos_time(0),
		_armed_sub(_mavlink->add_orb_subscription(ORB_ID(actuator_armed))),
		_armed_time(0),
		_act0_sub(_mavlink->add_orb_subscription(ORB_ID(actuator_controls_0))),
		_act1_sub(_mavlink->add_orb_subscription(ORB_ID(actuator_controls_1))),
		_airspeed_sub(_mavlink->add_orb_subscription(ORB_ID(airspeed))),
		_airspeed_time(0),
		_air_data_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_air_data)))
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_local_position_s pos = {};
		actuator_armed_s armed = {};
		airspeed_s airspeed = {};

		bool updated = false;
		updated |= _pos_sub->update(&_pos_time, &pos);
		updated |= _armed_sub->update(&_armed_time, &armed);
		updated |= _airspeed_sub->update(&_airspeed_time, &airspeed);

		if (updated) {
			mavlink_vfr_hud_t msg = {};
			msg.airspeed = airspeed.indicated_airspeed_m_s;
			msg.groundspeed = sqrtf(pos.vx * pos.vx + pos.vy * pos.vy);
			msg.heading = math::degrees(wrap_2pi(pos.yaw));

			if (armed.armed) {
				actuator_controls_s act0 = {};
				actuator_controls_s act1 = {};
				_act0_sub->update(&act0);
				_act1_sub->update(&act1);

				// VFR_HUD throttle should only be used for operator feedback.
				// VTOLs switch between actuator_controls_0 and actuator_controls_1. During transition there isn't a
				// a single throttle value, but this should still be a useful heuristic for operator awareness.
				//
				// Use ACTUATOR_CONTROL_TARGET if accurate states are needed.
				msg.throttle = 100 * math::max(
						       act0.control[actuator_controls_s::INDEX_THROTTLE],
						       act1.control[actuator_controls_s::INDEX_THROTTLE]);

			} else {
				msg.throttle = 0.0f;
			}

			if (pos.z_valid && pos.z_global) {
				/* use local position estimate */
				msg.alt = -pos.z + pos.ref_alt;

			} else {
				vehicle_air_data_s air_data = {};
				_air_data_sub->update(&air_data);

				/* fall back to baro altitude */
				if (air_data.timestamp > 0) {
					msg.alt = air_data.baro_alt_meter;
				}
			}

			if (pos.v_z_valid) {
				msg.climb = -pos.vz;
			}

			mavlink_msg_vfr_hud_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamGPSRawInt : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamGPSRawInt::get_name_static();
	}

	static const char *get_name_static()
	{
		return "GPS_RAW_INT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GPS_RAW_INT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGPSRawInt(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_GPS_RAW_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_gps_sub;
	uint64_t _gps_time;

	/* do not allow top copying this class */
	MavlinkStreamGPSRawInt(MavlinkStreamGPSRawInt &) = delete;
	MavlinkStreamGPSRawInt &operator = (const MavlinkStreamGPSRawInt &) = delete;

protected:
	explicit MavlinkStreamGPSRawInt(Mavlink *mavlink) : MavlinkStream(mavlink),
		_gps_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_gps_position))),
		_gps_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_gps_position_s gps;

		if (_gps_sub->update(&_gps_time, &gps)) {
			mavlink_gps_raw_int_t msg = {};

			msg.time_usec = gps.timestamp;
			msg.fix_type = gps.fix_type;
			msg.lat = gps.lat;
			msg.lon = gps.lon;
			msg.alt = gps.alt;
			msg.alt_ellipsoid = gps.alt_ellipsoid;
			msg.eph = gps.hdop * 100;
			msg.epv = gps.vdop * 100;
			msg.h_acc = gps.eph * 1e3f;
			msg.v_acc = gps.epv * 1e3f;
			msg.vel_acc = gps.s_variance_m_s * 1e3f;
			msg.hdg_acc = gps.c_variance_rad * 1e5f / M_DEG_TO_RAD_F;
			msg.vel = cm_uint16_from_m_float(gps.vel_m_s);
			msg.cog = math::degrees(wrap_2pi(gps.cog_rad)) * 1e2f;
			msg.satellites_visible = gps.satellites_used;

			mavlink_msg_gps_raw_int_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamGPS2Raw : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamGPS2Raw::get_name_static();
	}

	static const char *get_name_static()
	{
		return "GPS2_RAW";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GPS2_RAW;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGPS2Raw(mavlink);
	}

	unsigned get_size()
	{
		return (_gps_time > 0) ? (MAVLINK_MSG_ID_GPS2_RAW_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_gps_sub;
	uint64_t _gps_time;

	/* do not allow top copying this class */
	MavlinkStreamGPS2Raw(MavlinkStreamGPS2Raw &) = delete;
	MavlinkStreamGPS2Raw &operator = (const MavlinkStreamGPS2Raw &) = delete;

protected:
	explicit MavlinkStreamGPS2Raw(Mavlink *mavlink) : MavlinkStream(mavlink),
		_gps_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_gps_position), 1)),
		_gps_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_gps_position_s gps;

		if (_gps_sub->update(&_gps_time, &gps)) {
			mavlink_gps2_raw_t msg = {};

			msg.time_usec = gps.timestamp;
			msg.fix_type = gps.fix_type;
			msg.lat = gps.lat;
			msg.lon = gps.lon;
			msg.alt = gps.alt;
			msg.eph = gps.eph * 1e3f;
			msg.epv = gps.epv * 1e3f;
			msg.vel = cm_uint16_from_m_float(gps.vel_m_s);
			msg.cog = math::degrees(wrap_2pi(gps.cog_rad)) * 1e2f;
			msg.satellites_visible = gps.satellites_used;
			//msg.dgps_numch = // Number of DGPS satellites
			//msg.dgps_age = // Age of DGPS info

			mavlink_msg_gps2_raw_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamSystemTime : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamSystemTime::get_name_static();
	}

	static const char *get_name_static()
	{
		return "SYSTEM_TIME";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SYSTEM_TIME;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamSystemTime(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_SYSTEM_TIME_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	/* do not allow top copying this class */
	MavlinkStreamSystemTime(MavlinkStreamSystemTime &) = delete;
	MavlinkStreamSystemTime &operator = (const MavlinkStreamSystemTime &) = delete;

protected:
	explicit MavlinkStreamSystemTime(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t)
	{
		mavlink_system_time_t msg = {};
		timespec tv;

		px4_clock_gettime(CLOCK_REALTIME, &tv);

		msg.time_boot_ms = hrt_absolute_time() / 1000;
		msg.time_unix_usec = (uint64_t)tv.tv_sec * 1000000 + tv.tv_nsec / 1000;

		// If the time is before 2001-01-01, it's probably the default 2000
		// and we don't need to bother sending it because it's definitely wrong.
		if (msg.time_unix_usec > 978307200000000) {
			mavlink_msg_system_time_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

class MavlinkStreamTimesync : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamTimesync::get_name_static();
	}

	static const char *get_name_static()
	{
		return "TIMESYNC";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_TIMESYNC;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamTimesync(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_TIMESYNC_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	/* do not allow top copying this class */
	MavlinkStreamTimesync(MavlinkStreamTimesync &) = delete;
	MavlinkStreamTimesync &operator = (const MavlinkStreamTimesync &) = delete;

protected:
	explicit MavlinkStreamTimesync(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t)
	{
		mavlink_timesync_t msg = {};

		msg.tc1 = 0;
		msg.ts1 = hrt_absolute_time() * 1000; // boot time in nanoseconds

		mavlink_msg_timesync_send_struct(_mavlink->get_channel(), &msg);

		return true;
	}
};

class MavlinkStreamADSBVehicle : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamADSBVehicle::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ADSB_VEHICLE";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ADSB_VEHICLE;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamADSBVehicle(mavlink);
	}

	bool const_rate()
	{
		return true;
	}

	unsigned get_size()
	{
		return (_pos_time > 0) ? MAVLINK_MSG_ID_ADSB_VEHICLE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_pos_sub;
	uint64_t _pos_time;

	/* do not allow top copying this class */
	MavlinkStreamADSBVehicle(MavlinkStreamADSBVehicle &) = delete;
	MavlinkStreamADSBVehicle &operator = (const MavlinkStreamADSBVehicle &) = delete;

protected:
	explicit MavlinkStreamADSBVehicle(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_sub(_mavlink->add_orb_subscription(ORB_ID(transponder_report))),
		_pos_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct transponder_report_s pos;
		bool sent = false;

		while (_pos_sub->update(&_pos_time, &pos)) {
			mavlink_adsb_vehicle_t msg = {};

			if (!(pos.flags & transponder_report_s::PX4_ADSB_FLAGS_RETRANSLATE)) { continue; }

			msg.ICAO_address = pos.icao_address;
			msg.lat = pos.lat * 1e7;
			msg.lon = pos.lon * 1e7;
			msg.altitude_type = pos.altitude_type;
			msg.altitude = pos.altitude * 1e3f;
			msg.heading = (pos.heading + M_PI_F) / M_PI_F * 180.0f * 100.0f;
			msg.hor_velocity = pos.hor_velocity * 100.0f;
			msg.ver_velocity = pos.ver_velocity * 100.0f;
			memcpy(&msg.callsign[0], &pos.callsign[0], sizeof(msg.callsign));
			msg.emitter_type = pos.emitter_type;
			msg.tslc = pos.tslc;
			msg.squawk = pos.squawk;

			msg.flags = 0;

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS) { msg.flags |= ADSB_FLAGS_VALID_COORDS; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE) { msg.flags |= ADSB_FLAGS_VALID_ALTITUDE; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING) { msg.flags |= ADSB_FLAGS_VALID_HEADING; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY) { msg.flags |= ADSB_FLAGS_VALID_VELOCITY; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN) { msg.flags |= ADSB_FLAGS_VALID_CALLSIGN; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_SQUAWK) { msg.flags |= ADSB_FLAGS_VALID_SQUAWK; }

			mavlink_msg_adsb_vehicle_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

		return sent;
	}
};

class MavlinkStreamUTMGlobalPosition : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamUTMGlobalPosition::get_name_static();
	}

	static const char *get_name_static()
	{
		return "UTM_GLOBAL_POSITION";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_UTM_GLOBAL_POSITION;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamUTMGlobalPosition(mavlink);
	}

	bool const_rate()
	{
		return true;
	}

	unsigned get_size()
	{
		return _local_pos_time > 0 ? MAVLINK_MSG_ID_UTM_GLOBAL_POSITION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_local_pos_sub;
	uint64_t _local_pos_time = 0;
	vehicle_local_position_s _local_position   = {};

	MavlinkOrbSubscription *_global_pos_sub;
	uint64_t _global_pos_time = 0;
	vehicle_global_position_s _global_position = {};

	MavlinkOrbSubscription *_position_setpoint_triplet_sub;
	uint64_t _setpoint_triplet_time = 0;
	position_setpoint_triplet_s _setpoint_triplet = {};

	MavlinkOrbSubscription *_vehicle_status_sub;
	uint64_t _vehicle_status_time = 0;
	vehicle_status_s _vehicle_status = {};

	MavlinkOrbSubscription *_land_detected_sub;
	uint64_t _land_detected_time = 0;
	vehicle_land_detected_s _land_detected = {};

	/* do not allow top copying this class */
	MavlinkStreamUTMGlobalPosition(MavlinkStreamUTMGlobalPosition &) = delete;
	MavlinkStreamUTMGlobalPosition &operator = (const MavlinkStreamUTMGlobalPosition &) = delete;

protected:
	explicit MavlinkStreamUTMGlobalPosition(Mavlink *mavlink) : MavlinkStream(mavlink),
		_local_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position))),
		_global_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_global_position))),
		_position_setpoint_triplet_sub(_mavlink->add_orb_subscription(ORB_ID(position_setpoint_triplet))),
		_vehicle_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status))),
		_land_detected_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_land_detected)))
	{}

	bool send(const hrt_abstime t)
	{

		// Check if new uORB messages are available otherwise use the last received
		_local_pos_sub->update(&_local_pos_time, &_local_position);
		_global_pos_sub->update(&_global_pos_time, &_global_position);
		_position_setpoint_triplet_sub->update(&_setpoint_triplet_time, &_setpoint_triplet);
		_vehicle_status_sub->update(&_vehicle_status_time, &_vehicle_status);
		_land_detected_sub->update(&_land_detected_time, &_land_detected);

		mavlink_utm_global_position_t msg = {};

		// Compute Unix epoch and set time field
		timespec tv;
		px4_clock_gettime(CLOCK_REALTIME, &tv);
		uint64_t unix_epoch = (uint64_t)tv.tv_sec * 1000000 + tv.tv_nsec / 1000;

		// If the time is before 2001-01-01, it's probably the default 2000
		if (unix_epoch > 978307200000000) {
			msg.time = unix_epoch;
			msg.flags |= UTM_DATA_AVAIL_FLAGS_TIME_VALID;
		}

#ifndef BOARD_HAS_NO_UUID
		px4_guid_t px4_guid;
		board_get_px4_guid(px4_guid);
		static_assert(sizeof(px4_guid_t) == sizeof(msg.uas_id), "GUID byte length mismatch");
		memcpy(&msg.uas_id, &px4_guid, sizeof(msg.uas_id));
		msg.flags |= UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE;
#else
		// TODO Fill ID with something reasonable
		memset(&msg.uas_id[0], 0, sizeof(msg.uas_id));
#endif /* BOARD_HAS_NO_UUID */

		// Handle global position
		if (_global_pos_time > 0) {
			msg.lat = _global_position.lat * 1e7;
			msg.lon = _global_position.lon * 1e7;
			msg.alt = _global_position.alt_ellipsoid * 1000.0f;

			msg.h_acc = _global_position.eph * 1000.0f;
			msg.v_acc = _global_position.epv * 1000.0f;

			msg.flags |= UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE;
			msg.flags |= UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE;
		}

		// Handle local position
		if (_local_pos_time > 0) {
			float evh = 0.0f;
			float evv = 0.0f;

			if (_local_position.v_xy_valid) {
				msg.vx = _local_position.vx * 100.0f;
				msg.vy = _local_position.vy * 100.0f;
				evh = _local_position.evh;
				msg.flags |= UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE;
			}

			if (_local_position.v_z_valid) {
				msg.vz = _local_position.vz * 100.0f;
				evv = _local_position.evv;
				msg.flags |= UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE;
			}

			msg.vel_acc = sqrtf(evh * evh + evv * evv) * 100.0f;

			if (_local_position.dist_bottom_valid) {
				msg.relative_alt = _local_position.dist_bottom * 1000.0f;
				msg.flags |= UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE;
			}
		}

		bool vehicle_in_auto_mode = _vehicle_status_time > 0
					    && (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET
						|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS
						|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND
						|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL
						|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND
						|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION
						|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER
						|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF
						|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL
						|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER);

		// Handle next waypoint if it is valid
		if (vehicle_in_auto_mode && _setpoint_triplet_time > 0 && _setpoint_triplet.current.valid) {
			msg.next_lat = _setpoint_triplet.current.lat * 1e7;
			msg.next_lon = _setpoint_triplet.current.lon * 1e7;
			// HACK We assume that the offset between AMSL and WGS84 is constant between the current
			// vehicle position and the the target waypoint.
			msg.next_alt = (_setpoint_triplet.current.alt + (_global_position.alt_ellipsoid - _global_position.alt)) * 1000.0f;
			msg.flags |= UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE;
		}

		// Handle flight state
		if (_vehicle_status_time > 0 && _land_detected_time > 0
		    && _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			if (_land_detected.landed) {
				msg.flight_state |= UTM_FLIGHT_STATE_GROUND;

			} else {
				msg.flight_state |= UTM_FLIGHT_STATE_AIRBORNE;
			}

		} else {
			msg.flight_state |= UTM_FLIGHT_STATE_UNKNOWN;
		}

		msg.update_rate = 0; // Data driven mode

		mavlink_msg_utm_global_position_send_struct(_mavlink->get_channel(), &msg);

		return true;
	}
};

class MavlinkStreamCollision : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamCollision::get_name_static();
	}

	static const char *get_name_static()
	{
		return "COLLISION";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_COLLISION;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCollision(mavlink);
	}

	unsigned get_size()
	{
		return (_collision_time > 0) ? MAVLINK_MSG_ID_COLLISION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_collision_sub;
	uint64_t _collision_time;

	/* do not allow top copying this class */
	MavlinkStreamCollision(MavlinkStreamCollision &) = delete;
	MavlinkStreamCollision &operator = (const MavlinkStreamCollision &) = delete;

protected:
	explicit MavlinkStreamCollision(Mavlink *mavlink) : MavlinkStream(mavlink),
		_collision_sub(_mavlink->add_orb_subscription(ORB_ID(collision_report))),
		_collision_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct collision_report_s report;
		bool sent = false;

		while (_collision_sub->update(&_collision_time, &report)) {
			mavlink_collision_t msg = {};

			msg.src = report.src;
			msg.id = report.id;
			msg.action = report.action;
			msg.threat_level = report.threat_level;
			msg.time_to_minimum_delta = report.time_to_minimum_delta;
			msg.altitude_minimum_delta = report.altitude_minimum_delta;
			msg.horizontal_minimum_delta = report.horizontal_minimum_delta;

			mavlink_msg_collision_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

		return sent;
	}
};

class MavlinkStreamCameraTrigger : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamCameraTrigger::get_name_static();
	}

	static const char *get_name_static()
	{
		return "CAMERA_TRIGGER";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_CAMERA_TRIGGER;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCameraTrigger(mavlink);
	}

	bool const_rate()
	{
		return true;
	}

	unsigned get_size()
	{
		return (_trigger_time > 0) ? MAVLINK_MSG_ID_CAMERA_TRIGGER_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_trigger_sub;
	uint64_t _trigger_time;

	/* do not allow top copying this class */
	MavlinkStreamCameraTrigger(MavlinkStreamCameraTrigger &) = delete;
	MavlinkStreamCameraTrigger &operator = (const MavlinkStreamCameraTrigger &) = delete;

protected:
	explicit MavlinkStreamCameraTrigger(Mavlink *mavlink) : MavlinkStream(mavlink),
		_trigger_sub(_mavlink->add_orb_subscription(ORB_ID(camera_trigger))),
		_trigger_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct camera_trigger_s trigger;

		if (_trigger_sub->update(&_trigger_time, &trigger)) {
			mavlink_camera_trigger_t msg = {};

			msg.time_usec = trigger.timestamp;
			msg.seq = trigger.seq;

			/* ensure that only active trigger events are sent */
			if (trigger.timestamp > 0) {

				mavlink_msg_camera_trigger_send_struct(_mavlink->get_channel(), &msg);

				vehicle_command_s vcmd = {};
				vcmd.timestamp = hrt_absolute_time();
				vcmd.param1 = 0.0f; // all cameras
				vcmd.param2 = 0.0f; // duration 0 because only taking one picture
				vcmd.param3 = 1.0f; // only take one
				vcmd.param4 = NAN;
				vcmd.param5 = (double)NAN;
				vcmd.param6 = (double)NAN;
				vcmd.param7 = NAN;
				vcmd.command = MAV_CMD_IMAGE_START_CAPTURE;
				vcmd.target_system = mavlink_system.sysid;
				vcmd.target_component = MAV_COMP_ID_CAMERA;

				MavlinkCommandSender::instance().handle_vehicle_command(vcmd, _mavlink->get_channel());

				// TODO: move this camera_trigger and publish as a vehicle_command
				/* send MAV_CMD_DO_DIGICAM_CONTROL*/
				mavlink_command_long_t digicam_ctrl_cmd = {};

				digicam_ctrl_cmd.target_system = 0; // 0 for broadcast
				digicam_ctrl_cmd.target_component = MAV_COMP_ID_CAMERA;
				digicam_ctrl_cmd.command = MAV_CMD_DO_DIGICAM_CONTROL;
				digicam_ctrl_cmd.confirmation = 0;
				digicam_ctrl_cmd.param1 = NAN;
				digicam_ctrl_cmd.param2 = NAN;
				digicam_ctrl_cmd.param3 = NAN;
				digicam_ctrl_cmd.param4 = NAN;
				digicam_ctrl_cmd.param5 = 1;   // take 1 picture
				digicam_ctrl_cmd.param6 = NAN;
				digicam_ctrl_cmd.param7 = NAN;

				mavlink_msg_command_long_send_struct(_mavlink->get_channel(), &digicam_ctrl_cmd);

				return true;
			}
		}

		return false;
	}
};

class MavlinkStreamCameraImageCaptured : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamCameraImageCaptured::get_name_static();
	}

	static const char *get_name_static()
	{
		return "CAMERA_IMAGE_CAPTURED";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	bool const_rate()
	{
		return true;
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCameraImageCaptured(mavlink);
	}

	unsigned get_size()
	{
		return (_capture_time > 0) ? MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_capture_sub;
	uint64_t _capture_time;

	/* do not allow top copying this class */
	MavlinkStreamCameraImageCaptured(MavlinkStreamCameraImageCaptured &) = delete;
	MavlinkStreamCameraImageCaptured &operator = (const MavlinkStreamCameraImageCaptured &) = delete;

protected:
	explicit MavlinkStreamCameraImageCaptured(Mavlink *mavlink) : MavlinkStream(mavlink),
		_capture_sub(_mavlink->add_orb_subscription(ORB_ID(camera_capture))),
		_capture_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct camera_capture_s capture;

		if (_capture_sub->update(&_capture_time, &capture)) {

			mavlink_camera_image_captured_t msg;

			msg.time_boot_ms = capture.timestamp / 1000;
			msg.time_utc = capture.timestamp_utc;
			msg.camera_id = 1;	// FIXME : get this from uORB
			msg.lat = capture.lat * 1e7;
			msg.lon = capture.lon * 1e7;
			msg.alt = capture.alt * 1e3f;
			msg.relative_alt = capture.ground_distance * 1e3f;
			msg.q[0] = capture.q[0];
			msg.q[1] = capture.q[1];
			msg.q[2] = capture.q[2];
			msg.q[3] = capture.q[3];
			msg.image_index = capture.seq;
			msg.capture_result = capture.result;
			msg.file_url[0] = '\0';

			mavlink_msg_camera_image_captured_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamGlobalPositionInt : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamGlobalPositionInt::get_name_static();
	}

	static const char *get_name_static()
	{
		return "GLOBAL_POSITION_INT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGlobalPositionInt(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_gpos_sub;
	uint64_t _gpos_time;

	MavlinkOrbSubscription *_lpos_sub;
	uint64_t _lpos_time;

	MavlinkOrbSubscription *_home_sub;
	MavlinkOrbSubscription *_air_data_sub;

	/* do not allow top copying this class */
	MavlinkStreamGlobalPositionInt(MavlinkStreamGlobalPositionInt &) = delete;
	MavlinkStreamGlobalPositionInt &operator = (const MavlinkStreamGlobalPositionInt &) = delete;

protected:
	explicit MavlinkStreamGlobalPositionInt(Mavlink *mavlink) : MavlinkStream(mavlink),
		_gpos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_global_position))),
		_gpos_time(0),
		_lpos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position))),
		_lpos_time(0),
		_home_sub(_mavlink->add_orb_subscription(ORB_ID(home_position))),
		_air_data_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_air_data)))
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_global_position_s gpos = {};
		vehicle_local_position_s lpos = {};

		bool gpos_updated = _gpos_sub->update(&_gpos_time, &gpos);
		bool lpos_updated = _lpos_sub->update(&_lpos_time, &lpos);

		if (gpos_updated && lpos_updated) {
			mavlink_global_position_int_t msg = {};

			if (lpos.z_valid && lpos.z_global) {
				msg.alt = (-lpos.z + lpos.ref_alt) * 1000.0f;

			} else {
				// fall back to baro altitude
				vehicle_air_data_s air_data = {};
				_air_data_sub->update(&air_data);

				if (air_data.timestamp > 0) {
					msg.alt = air_data.baro_alt_meter * 1000.0f;
				}
			}

			home_position_s home = {};
			_home_sub->update(&home);

			if ((home.timestamp > 0) && home.valid_alt) {
				if (lpos.z_valid) {
					msg.relative_alt = -(lpos.z - home.z) * 1000.0f;

				} else {
					msg.relative_alt = msg.alt - (home.alt * 1000.0f);
				}

			} else {
				if (lpos.z_valid) {
					msg.relative_alt = -lpos.z * 1000.0f;
				}
			}

			msg.time_boot_ms = gpos.timestamp / 1000;
			msg.lat = gpos.lat * 1e7;
			msg.lon = gpos.lon * 1e7;

			msg.vx = lpos.vx * 100.0f;
			msg.vy = lpos.vy * 100.0f;
			msg.vz = lpos.vz * 100.0f;

			msg.hdg = math::degrees(wrap_2pi(lpos.yaw)) * 100.0f;

			mavlink_msg_global_position_int_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamOdometry : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamOdometry::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ODOMETRY";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ODOMETRY;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamOdometry(mavlink);
	}

	unsigned get_size()
	{
		return (_odom_time > 0) ? MAVLINK_MSG_ID_ODOMETRY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_odom_sub;
	uint64_t _odom_time;

	MavlinkOrbSubscription *_vodom_sub;
	uint64_t _vodom_time;

	/* do not allow top copying this class */
	MavlinkStreamOdometry(MavlinkStreamOdometry &) = delete;
	MavlinkStreamOdometry &operator = (const MavlinkStreamOdometry &) = delete;

protected:
	explicit MavlinkStreamOdometry(Mavlink *mavlink) : MavlinkStream(mavlink),
		_odom_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_odometry))),
		_odom_time(0),
		_vodom_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_visual_odometry))),
		_vodom_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_odometry_s odom;
		// check if it is to send visual odometry loopback or not
		bool odom_updated = false;

		mavlink_odometry_t msg = {};

		if (_send_odom_loopback.get()) {
			odom_updated = _vodom_sub->update(&_vodom_time, &odom);
			// frame matches the external vision system
			msg.frame_id = MAV_FRAME_VISION_NED;

		} else {
			odom_updated = _odom_sub->update(&_odom_time, &odom);
			// frame matches the PX4 local NED frame
			msg.frame_id = MAV_FRAME_ESTIM_NED;
		}

		if (odom_updated) {
			msg.time_usec = odom.timestamp;
			msg.child_frame_id = MAV_FRAME_BODY_NED;

			// Current position
			msg.x = odom.x;
			msg.y = odom.y;
			msg.z = odom.z;

			// Current orientation
			msg.q[0] = odom.q[0];
			msg.q[1] = odom.q[1];
			msg.q[2] = odom.q[2];
			msg.q[3] = odom.q[3];

			// Local NED to body-NED Dcm matrix
			matrix::Dcmf Rlb(matrix::Quatf(odom.q));

			// Rotate linear and angular velocity from local NED to body-NED frame
			matrix::Vector3f linvel_body(Rlb * matrix::Vector3f(odom.vx, odom.vy, odom.vz));

			// Current linear velocity
			msg.vx = linvel_body(0);
			msg.vy = linvel_body(1);
			msg.vz = linvel_body(2);

			// Current body rates
			msg.rollspeed = odom.rollspeed;
			msg.pitchspeed = odom.pitchspeed;
			msg.yawspeed = odom.yawspeed;

			// get the covariance matrix size
			const size_t POS_URT_SIZE = sizeof(odom.pose_covariance) / sizeof(odom.pose_covariance[0]);
			const size_t VEL_URT_SIZE = sizeof(odom.velocity_covariance) / sizeof(odom.velocity_covariance[0]);
			static_assert(POS_URT_SIZE == (sizeof(msg.pose_covariance) / sizeof(msg.pose_covariance[0])),
				      "Odometry Pose Covariance matrix URT array size mismatch");
			static_assert(VEL_URT_SIZE == (sizeof(msg.twist_covariance) / sizeof(msg.twist_covariance[0])),
				      "Odometry Velocity Covariance matrix URT array size mismatch");

			// copy pose covariances
			for (size_t i = 0; i < POS_URT_SIZE; i++) {
				msg.pose_covariance[i] = odom.pose_covariance[i];
			}

			// copy velocity covariances
			//TODO: Apply rotation matrix to transform from body-fixed NED to earth-fixed NED frame
			for (size_t i = 0; i < VEL_URT_SIZE; i++) {
				msg.twist_covariance[i] = odom.velocity_covariance[i];
			}

			mavlink_msg_odometry_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;

	}
};

class MavlinkStreamLocalPositionNED : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamLocalPositionNED::get_name_static();
	}

	static const char *get_name_static()
	{
		return "LOCAL_POSITION_NED";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_LOCAL_POSITION_NED;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamLocalPositionNED(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_pos_sub;
	uint64_t _pos_time;

	/* do not allow top copying this class */
	MavlinkStreamLocalPositionNED(MavlinkStreamLocalPositionNED &) = delete;
	MavlinkStreamLocalPositionNED &operator = (const MavlinkStreamLocalPositionNED &) = delete;

protected:
	explicit MavlinkStreamLocalPositionNED(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position))),
		_pos_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_local_position_s pos;

		if (_pos_sub->update(&_pos_time, &pos)) {
			mavlink_local_position_ned_t msg = {};

			msg.time_boot_ms = pos.timestamp / 1000;
			msg.x = pos.x;
			msg.y = pos.y;
			msg.z = pos.z;
			msg.vx = pos.vx;
			msg.vy = pos.vy;
			msg.vz = pos.vz;

			mavlink_msg_local_position_ned_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamEstimatorStatus : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamEstimatorStatus::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ESTIMATOR_STATUS";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_VIBRATION;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamEstimatorStatus(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_VIBRATION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_est_sub;
	uint64_t _est_time;

	/* do not allow top copying this class */
	MavlinkStreamEstimatorStatus(MavlinkStreamEstimatorStatus &) = delete;
	MavlinkStreamEstimatorStatus &operator = (const MavlinkStreamEstimatorStatus &) = delete;

protected:
	explicit MavlinkStreamEstimatorStatus(Mavlink *mavlink) : MavlinkStream(mavlink),
		_est_sub(_mavlink->add_orb_subscription(ORB_ID(estimator_status))),
		_est_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		estimator_status_s est;

		if (_est_sub->update(&_est_time, &est)) {
			// ESTIMATOR_STATUS
			mavlink_estimator_status_t est_msg = {};
			est_msg.time_usec = est.timestamp;
			est_msg.vel_ratio = est.vel_test_ratio;
			est_msg.pos_horiz_ratio = est.pos_test_ratio;
			est_msg.pos_vert_ratio = est.hgt_test_ratio;
			est_msg.mag_ratio = est.mag_test_ratio;
			est_msg.hagl_ratio = est.hagl_test_ratio;
			est_msg.tas_ratio = est.tas_test_ratio;
			est_msg.pos_horiz_accuracy = est.pos_horiz_accuracy;
			est_msg.pos_vert_accuracy = est.pos_vert_accuracy;
			est_msg.flags = est.solution_status_flags;
			mavlink_msg_estimator_status_send_struct(_mavlink->get_channel(), &est_msg);

			// VIBRATION
			mavlink_vibration_t msg = {};
			msg.time_usec = est.timestamp;
			msg.vibration_x = est.vibe[0];
			msg.vibration_y = est.vibe[1];
			msg.vibration_z = est.vibe[2];
			mavlink_msg_vibration_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamAttPosMocap : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamAttPosMocap::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ATT_POS_MOCAP";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ATT_POS_MOCAP;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAttPosMocap(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_mocap_sub;
	uint64_t _mocap_time;

	/* do not allow top copying this class */
	MavlinkStreamAttPosMocap(MavlinkStreamAttPosMocap &) = delete;
	MavlinkStreamAttPosMocap &operator = (const MavlinkStreamAttPosMocap &) = delete;

protected:
	explicit MavlinkStreamAttPosMocap(Mavlink *mavlink) : MavlinkStream(mavlink),
		_mocap_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_mocap_odometry))),
		_mocap_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_odometry_s mocap;

		if (_mocap_sub->update(&_mocap_time, &mocap)) {
			mavlink_att_pos_mocap_t msg = {};

			msg.time_usec = mocap.timestamp;
			msg.q[0] = mocap.q[0];
			msg.q[1] = mocap.q[1];
			msg.q[2] = mocap.q[2];
			msg.q[3] = mocap.q[3];
			msg.x = mocap.x;
			msg.y = mocap.y;
			msg.z = mocap.z;

			mavlink_msg_att_pos_mocap_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamHomePosition : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamHomePosition::get_name_static();
	}

	static const char *get_name_static()
	{
		return "HOME_POSITION";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HOME_POSITION;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHomePosition(mavlink);
	}

	unsigned get_size()
	{
		return _home_sub->is_published() ? (MAVLINK_MSG_ID_HOME_POSITION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_home_sub;

	/* do not allow top copying this class */
	MavlinkStreamHomePosition(MavlinkStreamHomePosition &) = delete;
	MavlinkStreamHomePosition &operator = (const MavlinkStreamHomePosition &) = delete;

protected:
	explicit MavlinkStreamHomePosition(Mavlink *mavlink) : MavlinkStream(mavlink),
		_home_sub(_mavlink->add_orb_subscription(ORB_ID(home_position)))
	{}

	bool send(const hrt_abstime t)
	{
		/* we're sending the GPS home periodically to ensure the
		 * the GCS does pick it up at one point */
		if (_home_sub->is_published()) {
			home_position_s home;

			if (_home_sub->update(&home)) {
				if (home.valid_hpos) {
					mavlink_home_position_t msg;

					msg.latitude = home.lat * 1e7;
					msg.longitude = home.lon * 1e7;
					msg.altitude = home.alt * 1e3f;

					msg.x = home.x;
					msg.y = home.y;
					msg.z = home.z;

					matrix::Quatf q(matrix::Eulerf(0.0f, 0.0f, home.yaw));
					msg.q[0] = q(0);
					msg.q[1] = q(1);
					msg.q[2] = q(2);
					msg.q[3] = q(3);

					msg.approach_x = 0.0f;
					msg.approach_y = 0.0f;
					msg.approach_z = 0.0f;

					mavlink_msg_home_position_send_struct(_mavlink->get_channel(), &msg);

					return true;
				}
			}
		}

		return false;
	}
};


template <int N>
class MavlinkStreamServoOutputRaw : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamServoOutputRaw<N>::get_name_static();
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SERVO_OUTPUT_RAW;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static const char *get_name_static()
	{
		switch (N) {
		case 0:
			return "SERVO_OUTPUT_RAW_0";

		case 1:
			return "SERVO_OUTPUT_RAW_1";

		case 2:
			return "SERVO_OUTPUT_RAW_2";

		case 3:
			return "SERVO_OUTPUT_RAW_3";
		}
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamServoOutputRaw<N>(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_act_sub;
	uint64_t _act_time;

	/* do not allow top copying this class */
	MavlinkStreamServoOutputRaw(MavlinkStreamServoOutputRaw &) = delete;
	MavlinkStreamServoOutputRaw &operator = (const MavlinkStreamServoOutputRaw &) = delete;

protected:
	explicit MavlinkStreamServoOutputRaw(Mavlink *mavlink) : MavlinkStream(mavlink),
		_act_sub(_mavlink->add_orb_subscription(ORB_ID(actuator_outputs), N)),
		_act_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		actuator_outputs_s act;

		if (_act_sub->update(&_act_time, &act)) {
			mavlink_servo_output_raw_t msg = {};

			static_assert(sizeof(act.output) / sizeof(act.output[0]) >= 16, "mavlink message requires at least 16 outputs");

			msg.time_usec = act.timestamp;
			msg.port = N;
			msg.servo1_raw = act.output[0];
			msg.servo2_raw = act.output[1];
			msg.servo3_raw = act.output[2];
			msg.servo4_raw = act.output[3];
			msg.servo5_raw = act.output[4];
			msg.servo6_raw = act.output[5];
			msg.servo7_raw = act.output[6];
			msg.servo8_raw = act.output[7];
			msg.servo9_raw = act.output[8];
			msg.servo10_raw = act.output[9];
			msg.servo11_raw = act.output[10];
			msg.servo12_raw = act.output[11];
			msg.servo13_raw = act.output[12];
			msg.servo14_raw = act.output[13];
			msg.servo15_raw = act.output[14];
			msg.servo16_raw = act.output[15];

			mavlink_msg_servo_output_raw_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

template <int N>
class MavlinkStreamActuatorControlTarget : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamActuatorControlTarget<N>::get_name_static();
	}

	static const char *get_name_static()
	{
		switch (N) {
		case 0:
			return "ACTUATOR_CONTROL_TARGET0";

		case 1:
			return "ACTUATOR_CONTROL_TARGET1";

		case 2:
			return "ACTUATOR_CONTROL_TARGET2";

		case 3:
			return "ACTUATOR_CONTROL_TARGET3";
		}
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamActuatorControlTarget<N>(mavlink);
	}

	unsigned get_size()
	{
		return _att_ctrl_sub->is_published() ? (MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_att_ctrl_sub;
	uint64_t _att_ctrl_time;

	/* do not allow top copying this class */
	MavlinkStreamActuatorControlTarget(MavlinkStreamActuatorControlTarget &) = delete;
	MavlinkStreamActuatorControlTarget &operator = (const MavlinkStreamActuatorControlTarget &) = delete;

protected:
	explicit MavlinkStreamActuatorControlTarget(Mavlink *mavlink) : MavlinkStream(mavlink),
		_att_ctrl_sub(nullptr),
		_att_ctrl_time(0)
	{
		// XXX this can be removed once the multiplatform system remaps topics
		switch (N) {
		case 0:
			_att_ctrl_sub = _mavlink->add_orb_subscription(ORB_ID(actuator_controls_0));
			break;

		case 1:
			_att_ctrl_sub = _mavlink->add_orb_subscription(ORB_ID(actuator_controls_1));
			break;

		case 2:
			_att_ctrl_sub = _mavlink->add_orb_subscription(ORB_ID(actuator_controls_2));
			break;

		case 3:
			_att_ctrl_sub = _mavlink->add_orb_subscription(ORB_ID(actuator_controls_3));
			break;
		}
	}

	bool send(const hrt_abstime t)
	{
		actuator_controls_s att_ctrl;

		if (_att_ctrl_sub->update(&_att_ctrl_time, &att_ctrl)) {
			mavlink_actuator_control_target_t msg = {};

			msg.time_usec = att_ctrl.timestamp;
			msg.group_mlx = N;

			for (unsigned i = 0; i < sizeof(msg.controls) / sizeof(msg.controls[0]); i++) {
				msg.controls[i] = att_ctrl.control[i];
			}

			mavlink_msg_actuator_control_target_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamHILActuatorControls : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamHILActuatorControls::get_name_static();
	}

	static const char *get_name_static()
	{
		return "HIL_ACTUATOR_CONTROLS";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHILActuatorControls(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_status_sub;

	MavlinkOrbSubscription *_act_sub;
	uint64_t _act_time;

	/* do not allow top copying this class */
	MavlinkStreamHILActuatorControls(MavlinkStreamHILActuatorControls &) = delete;
	MavlinkStreamHILActuatorControls &operator = (const MavlinkStreamHILActuatorControls &) = delete;

protected:
	explicit MavlinkStreamHILActuatorControls(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status))),
		_act_sub(_mavlink->add_orb_subscription(ORB_ID(actuator_outputs))),
		_act_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		actuator_outputs_s act;

		if (_act_sub->update(&_act_time, &act)) {
			vehicle_status_s status = {};
			_status_sub->update(&status);

			if ((status.timestamp > 0) && (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)) {
				/* translate the current system state to mavlink state and mode */
				uint8_t mavlink_state;
				uint8_t mavlink_base_mode;
				uint32_t mavlink_custom_mode;
				mavlink_hil_actuator_controls_t msg = {};

				get_mavlink_mode_state(&status, &mavlink_state, &mavlink_base_mode, &mavlink_custom_mode);

				const float pwm_center = (PWM_DEFAULT_MAX + PWM_DEFAULT_MIN) / 2;

				unsigned system_type = _mavlink->get_system_type();

				/* scale outputs depending on system type */
				if (system_type == MAV_TYPE_QUADROTOR ||
				    system_type == MAV_TYPE_HEXAROTOR ||
				    system_type == MAV_TYPE_OCTOROTOR ||
				    system_type == MAV_TYPE_VTOL_DUOROTOR ||
				    system_type == MAV_TYPE_VTOL_QUADROTOR ||
				    system_type == MAV_TYPE_VTOL_RESERVED2) {

					/* multirotors: set number of rotor outputs depending on type */

					unsigned n;

					switch (system_type) {
					case MAV_TYPE_QUADROTOR:
						n = 4;
						break;

					case MAV_TYPE_HEXAROTOR:
						n = 6;
						break;

					case MAV_TYPE_VTOL_DUOROTOR:
						n = 2;
						break;

					case MAV_TYPE_VTOL_QUADROTOR:
						n = 4;
						break;

					case MAV_TYPE_VTOL_RESERVED2:
						n = 8;
						break;

					default:
						n = 8;
						break;
					}

					for (unsigned i = 0; i < 16; i++) {
						if (act.output[i] > PWM_DEFAULT_MIN / 2) {
							if (i < n) {
								/* scale PWM out 900..2100 us to 0..1 for rotors */
								msg.controls[i] = (act.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);

							} else {
								/* scale PWM out 900..2100 us to -1..1 for other channels */
								msg.controls[i] = (act.output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);
							}

						} else {
							/* send 0 when disarmed and for disabled channels */
							msg.controls[i] = 0.0f;
						}
					}

				} else {
					/* fixed wing: scale throttle to 0..1 and other channels to -1..1 */

					for (unsigned i = 0; i < 16; i++) {
						if (act.output[i] > PWM_DEFAULT_MIN / 2) {
							if (i != 3) {
								/* scale PWM out 900..2100 us to -1..1 for normal channels */
								msg.controls[i] = (act.output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);

							} else {
								/* scale PWM out 900..2100 us to 0..1 for throttle */
								msg.controls[i] = (act.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);
							}

						} else {
							/* set 0 for disabled channels */
							msg.controls[i] = 0.0f;
						}
					}
				}

				msg.time_usec = hrt_absolute_time();
				msg.mode = mavlink_base_mode;
				msg.flags = 0;

				mavlink_msg_hil_actuator_controls_send_struct(_mavlink->get_channel(), &msg);

				return true;
			}
		}

		return false;
	}
};

class MavlinkStreamPositionTargetGlobalInt : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamPositionTargetGlobalInt::get_name_static();
	}

	static const char *get_name_static()
	{
		return "POSITION_TARGET_GLOBAL_INT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamPositionTargetGlobalInt(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_control_mode_sub;
	MavlinkOrbSubscription *_lpos_sp_sub;
	MavlinkOrbSubscription *_pos_sp_triplet_sub;

	/* do not allow top copying this class */
	MavlinkStreamPositionTargetGlobalInt(MavlinkStreamPositionTargetGlobalInt &) = delete;
	MavlinkStreamPositionTargetGlobalInt &operator = (const MavlinkStreamPositionTargetGlobalInt &) = delete;

protected:
	explicit MavlinkStreamPositionTargetGlobalInt(Mavlink *mavlink) : MavlinkStream(mavlink),
		_control_mode_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_control_mode))),
		_lpos_sp_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position_setpoint))),
		_pos_sp_triplet_sub(_mavlink->add_orb_subscription(ORB_ID(position_setpoint_triplet)))
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_control_mode_s control_mode = {};
		_control_mode_sub->update(&control_mode);

		if (control_mode.flag_control_position_enabled) {

			position_setpoint_triplet_s pos_sp_triplet;
			_pos_sp_triplet_sub->update(&pos_sp_triplet);

			if (pos_sp_triplet.timestamp > 0 && pos_sp_triplet.current.valid
			    && PX4_ISFINITE(pos_sp_triplet.current.lat) && PX4_ISFINITE(pos_sp_triplet.current.lon)) {

				mavlink_position_target_global_int_t msg = {};

				msg.time_boot_ms = hrt_absolute_time() / 1000;
				msg.coordinate_frame = MAV_FRAME_GLOBAL_INT;
				msg.lat_int = pos_sp_triplet.current.lat * 1e7;
				msg.lon_int = pos_sp_triplet.current.lon * 1e7;
				msg.alt = pos_sp_triplet.current.alt;

				vehicle_local_position_setpoint_s lpos_sp;

				if (_lpos_sp_sub->update(&lpos_sp)) {
					// velocity
					msg.vx = lpos_sp.vx;
					msg.vy = lpos_sp.vy;
					msg.vz = lpos_sp.vz;

					// acceleration
					msg.afx = lpos_sp.acc_x;
					msg.afy = lpos_sp.acc_y;
					msg.afz = lpos_sp.acc_z;

					// yaw
					msg.yaw = lpos_sp.yaw;
					msg.yaw_rate = lpos_sp.yawspeed;
				}

				mavlink_msg_position_target_global_int_send_struct(_mavlink->get_channel(), &msg);

				return true;
			}
		}

		return false;
	}
};


class MavlinkStreamLocalPositionSetpoint : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamLocalPositionSetpoint::get_name_static();
	}

	static const char *get_name_static()
	{
		return "POSITION_TARGET_LOCAL_NED";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamLocalPositionSetpoint(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_pos_sp_sub;
	uint64_t _pos_sp_time;

	/* do not allow top copying this class */
	MavlinkStreamLocalPositionSetpoint(MavlinkStreamLocalPositionSetpoint &) = delete;
	MavlinkStreamLocalPositionSetpoint &operator = (const MavlinkStreamLocalPositionSetpoint &) = delete;

protected:
	explicit MavlinkStreamLocalPositionSetpoint(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_sp_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position_setpoint))),
		_pos_sp_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_local_position_setpoint_s pos_sp;

		if (_pos_sp_sub->update(&_pos_sp_time, &pos_sp)) {
			mavlink_position_target_local_ned_t msg = {};

			msg.time_boot_ms = pos_sp.timestamp / 1000;
			msg.coordinate_frame = MAV_FRAME_LOCAL_NED;
			msg.x = pos_sp.x;
			msg.y = pos_sp.y;
			msg.z = pos_sp.z;
			msg.yaw = pos_sp.yaw;
			msg.yaw_rate = pos_sp.yawspeed;
			msg.vx = pos_sp.vx;
			msg.vy = pos_sp.vy;
			msg.vz = pos_sp.vz;
			msg.afx = pos_sp.acc_x;
			msg.afy = pos_sp.acc_y;
			msg.afz = pos_sp.acc_z;

			mavlink_msg_position_target_local_ned_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamAttitudeTarget : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamAttitudeTarget::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ATTITUDE_TARGET";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ATTITUDE_TARGET;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAttitudeTarget(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_att_sp_sub;
	MavlinkOrbSubscription *_att_rates_sp_sub;

	uint64_t _att_sp_time;

	/* do not allow top copying this class */
	MavlinkStreamAttitudeTarget(MavlinkStreamAttitudeTarget &) = delete;
	MavlinkStreamAttitudeTarget &operator = (const MavlinkStreamAttitudeTarget &) = delete;

protected:
	explicit MavlinkStreamAttitudeTarget(Mavlink *mavlink) : MavlinkStream(mavlink),
		_att_sp_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_attitude_setpoint))),
		_att_rates_sp_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_rates_setpoint))),
		_att_sp_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_attitude_setpoint_s att_sp;

		if (_att_sp_sub->update(&_att_sp_time, &att_sp)) {

			vehicle_rates_setpoint_s att_rates_sp = {};
			_att_rates_sp_sub->update(&att_rates_sp);

			mavlink_attitude_target_t msg = {};

			msg.time_boot_ms = att_sp.timestamp / 1000;

			if (att_sp.q_d_valid) {
				memcpy(&msg.q[0], &att_sp.q_d[0], sizeof(msg.q));

			} else {
				matrix::Quatf q = matrix::Eulerf(att_sp.roll_body, att_sp.pitch_body, att_sp.yaw_body);
				memcpy(&msg.q[0], q.data(), sizeof(msg.q));
			}

			msg.body_roll_rate = att_rates_sp.roll;
			msg.body_pitch_rate = att_rates_sp.pitch;
			msg.body_yaw_rate = att_rates_sp.yaw;

			msg.thrust = att_sp.thrust_body[0];

			mavlink_msg_attitude_target_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamRCChannels : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamRCChannels::get_name_static();
	}

	static const char *get_name_static()
	{
		return "RC_CHANNELS";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_RC_CHANNELS;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamRCChannels(mavlink);
	}

	unsigned get_size()
	{
		return _rc_sub->is_published() ? (MAVLINK_MSG_ID_RC_CHANNELS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_rc_sub;
	uint64_t _rc_time;

	/* do not allow top copying this class */
	MavlinkStreamRCChannels(MavlinkStreamRCChannels &) = delete;
	MavlinkStreamRCChannels &operator = (const MavlinkStreamRCChannels &) = delete;

protected:
	explicit MavlinkStreamRCChannels(Mavlink *mavlink) : MavlinkStream(mavlink),
		_rc_sub(_mavlink->add_orb_subscription(ORB_ID(input_rc))),
		_rc_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		input_rc_s rc;

		if (_rc_sub->update(&_rc_time, &rc)) {

			/* send RC channel data and RSSI */
			mavlink_rc_channels_t msg = {};

			msg.time_boot_ms = rc.timestamp / 1000;
			msg.chancount = rc.channel_count;
			msg.chan1_raw = (rc.channel_count > 0) ? rc.values[0] : UINT16_MAX;
			msg.chan2_raw = (rc.channel_count > 1) ? rc.values[1] : UINT16_MAX;
			msg.chan3_raw = (rc.channel_count > 2) ? rc.values[2] : UINT16_MAX;
			msg.chan4_raw = (rc.channel_count > 3) ? rc.values[3] : UINT16_MAX;
			msg.chan5_raw = (rc.channel_count > 4) ? rc.values[4] : UINT16_MAX;
			msg.chan6_raw = (rc.channel_count > 5) ? rc.values[5] : UINT16_MAX;
			msg.chan7_raw = (rc.channel_count > 6) ? rc.values[6] : UINT16_MAX;
			msg.chan8_raw = (rc.channel_count > 7) ? rc.values[7] : UINT16_MAX;
			msg.chan9_raw = (rc.channel_count > 8) ? rc.values[8] : UINT16_MAX;
			msg.chan10_raw = (rc.channel_count > 9) ? rc.values[9] : UINT16_MAX;
			msg.chan11_raw = (rc.channel_count > 10) ? rc.values[10] : UINT16_MAX;
			msg.chan12_raw = (rc.channel_count > 11) ? rc.values[11] : UINT16_MAX;
			msg.chan13_raw = (rc.channel_count > 12) ? rc.values[12] : UINT16_MAX;
			msg.chan14_raw = (rc.channel_count > 13) ? rc.values[13] : UINT16_MAX;
			msg.chan15_raw = (rc.channel_count > 14) ? rc.values[14] : UINT16_MAX;
			msg.chan16_raw = (rc.channel_count > 15) ? rc.values[15] : UINT16_MAX;
			msg.chan17_raw = (rc.channel_count > 16) ? rc.values[16] : UINT16_MAX;
			msg.chan18_raw = (rc.channel_count > 17) ? rc.values[17] : UINT16_MAX;

			msg.rssi = (rc.channel_count > 0) ? rc.rssi : 0;

			mavlink_msg_rc_channels_send_struct(_mavlink->get_channel(), &msg);

			/* send override message - harmless if connected to GCS, allows to connect a board to a Linux system */
			/* http://mavlink.org/messages/common#RC_CHANNELS_OVERRIDE */
			mavlink_rc_channels_override_t over;
			over.target_system = mavlink_system.sysid;
			over.target_component = 0;
			over.chan1_raw = msg.chan1_raw;
			over.chan2_raw = msg.chan2_raw;
			over.chan3_raw = msg.chan3_raw;
			over.chan4_raw = msg.chan4_raw;
			over.chan5_raw = msg.chan5_raw;
			over.chan6_raw = msg.chan6_raw;
			over.chan7_raw = msg.chan7_raw;
			over.chan8_raw = msg.chan8_raw;

			mavlink_msg_rc_channels_override_send_struct(_mavlink->get_channel(), &over);

			return true;
		}

		return false;
	}
};


class MavlinkStreamManualControl : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamManualControl::get_name_static();
	}

	static const char *get_name_static()
	{
		return "MANUAL_CONTROL";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_MANUAL_CONTROL;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamManualControl(mavlink);
	}

	unsigned get_size()
	{
		return _manual_sub->is_published() ? (MAVLINK_MSG_ID_MANUAL_CONTROL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_manual_sub;
	uint64_t _manual_time;

	/* do not allow top copying this class */
	MavlinkStreamManualControl(MavlinkStreamManualControl &) = delete;
	MavlinkStreamManualControl &operator = (const MavlinkStreamManualControl &) = delete;

protected:
	explicit MavlinkStreamManualControl(Mavlink *mavlink) : MavlinkStream(mavlink),
		_manual_sub(_mavlink->add_orb_subscription(ORB_ID(manual_control_setpoint))),
		_manual_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		manual_control_setpoint_s manual;

		if (_manual_sub->update(&_manual_time, &manual)) {
			mavlink_manual_control_t msg = {};

			msg.target = mavlink_system.sysid;
			msg.x = manual.x * 1000;
			msg.y = manual.y * 1000;
			msg.z = manual.z * 1000;
			msg.r = manual.r * 1000;
			unsigned shift = 2;
			msg.buttons = 0;
			msg.buttons |= (manual.mode_switch << (shift * 0));
			msg.buttons |= (manual.return_switch << (shift * 1));
			msg.buttons |= (manual.posctl_switch << (shift * 2));
			msg.buttons |= (manual.loiter_switch << (shift * 3));
			msg.buttons |= (manual.acro_switch << (shift * 4));
			msg.buttons |= (manual.offboard_switch << (shift * 5));

			mavlink_msg_manual_control_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamTrajectoryRepresentationWaypoints: public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamTrajectoryRepresentationWaypoints::get_name_static();
	}

	static const char *get_name_static()
	{
		return "TRAJECTORY_REPRESENTATION_WAYPOINTS";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamTrajectoryRepresentationWaypoints(mavlink);
	}

	unsigned get_size()
	{
		return _traj_wp_avoidance_sub->is_published() ? (MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN +
				MAVLINK_NUM_NON_PAYLOAD_BYTES)
		       : 0;
	}

private:
	MavlinkOrbSubscription *_traj_wp_avoidance_sub;
	uint64_t _traj_wp_avoidance_time;

	/* do not allow top copying this class */
	MavlinkStreamTrajectoryRepresentationWaypoints(MavlinkStreamTrajectoryRepresentationWaypoints &);
	MavlinkStreamTrajectoryRepresentationWaypoints &operator = (const MavlinkStreamTrajectoryRepresentationWaypoints &);

protected:
	explicit MavlinkStreamTrajectoryRepresentationWaypoints(Mavlink *mavlink) : MavlinkStream(mavlink),
		_traj_wp_avoidance_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_trajectory_waypoint_desired))),
		_traj_wp_avoidance_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct vehicle_trajectory_waypoint_s traj_wp_avoidance_desired;

		if (_traj_wp_avoidance_sub->update(&_traj_wp_avoidance_time, &traj_wp_avoidance_desired)) {
			mavlink_trajectory_representation_waypoints_t msg = {};

			msg.time_usec = traj_wp_avoidance_desired.timestamp;
			int number_valid_points = 0;

			for (int i = 0; i < vehicle_trajectory_waypoint_s::NUMBER_POINTS; ++i) {
				msg.pos_x[i] = traj_wp_avoidance_desired.waypoints[i].position[0];
				msg.pos_y[i] = traj_wp_avoidance_desired.waypoints[i].position[1];
				msg.pos_z[i] = traj_wp_avoidance_desired.waypoints[i].position[2];

				msg.vel_x[i] = traj_wp_avoidance_desired.waypoints[i].velocity[0];
				msg.vel_y[i] = traj_wp_avoidance_desired.waypoints[i].velocity[1];
				msg.vel_z[i] = traj_wp_avoidance_desired.waypoints[i].velocity[2];

				msg.acc_x[i] = traj_wp_avoidance_desired.waypoints[i].acceleration[0];
				msg.acc_y[i] = traj_wp_avoidance_desired.waypoints[i].acceleration[1];
				msg.acc_z[i] = traj_wp_avoidance_desired.waypoints[i].acceleration[2];

				msg.pos_yaw[i] = traj_wp_avoidance_desired.waypoints[i].yaw;
				msg.vel_yaw[i] = traj_wp_avoidance_desired.waypoints[i].yaw_speed;

				if (traj_wp_avoidance_desired.waypoints[i].point_valid) {
					number_valid_points++;
				}

			}

			msg.valid_points = number_valid_points;

			mavlink_msg_trajectory_representation_waypoints_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamOpticalFlowRad : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamOpticalFlowRad::get_name_static();
	}

	static const char *get_name_static()
	{
		return "OPTICAL_FLOW_RAD";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_OPTICAL_FLOW_RAD;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamOpticalFlowRad(mavlink);
	}

	unsigned get_size()
	{
		return _flow_sub->is_published() ? (MAVLINK_MSG_ID_OPTICAL_FLOW_RAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_flow_sub;
	uint64_t _flow_time;

	/* do not allow top copying this class */
	MavlinkStreamOpticalFlowRad(MavlinkStreamOpticalFlowRad &) = delete;
	MavlinkStreamOpticalFlowRad &operator = (const MavlinkStreamOpticalFlowRad &) = delete;

protected:
	explicit MavlinkStreamOpticalFlowRad(Mavlink *mavlink) : MavlinkStream(mavlink),
		_flow_sub(_mavlink->add_orb_subscription(ORB_ID(optical_flow))),
		_flow_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		optical_flow_s flow;

		if (_flow_sub->update(&_flow_time, &flow)) {
			mavlink_optical_flow_rad_t msg = {};

			msg.time_usec = flow.timestamp;
			msg.sensor_id = flow.sensor_id;
			msg.integrated_x = flow.pixel_flow_x_integral;
			msg.integrated_y = flow.pixel_flow_y_integral;
			msg.integrated_xgyro = flow.gyro_x_rate_integral;
			msg.integrated_ygyro = flow.gyro_y_rate_integral;
			msg.integrated_zgyro = flow.gyro_z_rate_integral;
			msg.distance = flow.ground_distance_m;
			msg.quality = flow.quality;
			msg.integration_time_us = flow.integration_timespan;
			msg.sensor_id = flow.sensor_id;
			msg.time_delta_distance_us = flow.time_since_last_sonar_update;
			msg.temperature = flow.gyro_temperature;

			mavlink_msg_optical_flow_rad_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamNamedValueFloat : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamNamedValueFloat::get_name_static();
	}

	static const char *get_name_static()
	{
		return "NAMED_VALUE_FLOAT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_NAMED_VALUE_FLOAT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamNamedValueFloat(mavlink);
	}

	unsigned get_size()
	{
		return (_debug_time > 0) ? MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_debug_sub;
	uint64_t _debug_time;

	/* do not allow top copying this class */
	MavlinkStreamNamedValueFloat(MavlinkStreamNamedValueFloat &) = delete;
	MavlinkStreamNamedValueFloat &operator = (const MavlinkStreamNamedValueFloat &) = delete;

protected:
	explicit MavlinkStreamNamedValueFloat(Mavlink *mavlink) : MavlinkStream(mavlink),
		_debug_sub(_mavlink->add_orb_subscription(ORB_ID(debug_key_value))),
		_debug_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct debug_key_value_s debug;

		if (_debug_sub->update(&_debug_time, &debug)) {
			mavlink_named_value_float_t msg = {};

			msg.time_boot_ms = debug.timestamp / 1000ULL;
			memcpy(msg.name, debug.key, sizeof(msg.name));
			/* enforce null termination */
			msg.name[sizeof(msg.name) - 1] = '\0';
			msg.value = debug.value;

			mavlink_msg_named_value_float_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamDebug : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamDebug::get_name_static();
	}

	static const char *get_name_static()
	{
		return "DEBUG";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_DEBUG;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamDebug(mavlink);
	}

	unsigned get_size()
	{
		return (_debug_time > 0) ? MAVLINK_MSG_ID_DEBUG_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_debug_sub;
	uint64_t _debug_time;

	/* do not allow top copying this class */
	MavlinkStreamDebug(MavlinkStreamDebug &) = delete;
	MavlinkStreamDebug &operator = (const MavlinkStreamDebug &) = delete;

protected:
	explicit MavlinkStreamDebug(Mavlink *mavlink) : MavlinkStream(mavlink),
		_debug_sub(_mavlink->add_orb_subscription(ORB_ID(debug_value))),
		_debug_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct debug_value_s debug = {};

		if (_debug_sub->update(&_debug_time, &debug)) {
			mavlink_debug_t msg = {};

			msg.time_boot_ms = debug.timestamp / 1000ULL;
			msg.ind = debug.ind;
			msg.value = debug.value;

			mavlink_msg_debug_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamDebugVect : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamDebugVect::get_name_static();
	}

	static const char *get_name_static()
	{
		return "DEBUG_VECT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_DEBUG_VECT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamDebugVect(mavlink);
	}

	unsigned get_size()
	{
		return (_debug_time > 0) ? MAVLINK_MSG_ID_DEBUG_VECT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_debug_sub;
	uint64_t _debug_time;

	/* do not allow top copying this class */
	MavlinkStreamDebugVect(MavlinkStreamDebugVect &) = delete;
	MavlinkStreamDebugVect &operator = (const MavlinkStreamDebugVect &) = delete;

protected:
	explicit MavlinkStreamDebugVect(Mavlink *mavlink) : MavlinkStream(mavlink),
		_debug_sub(_mavlink->add_orb_subscription(ORB_ID(debug_vect))),
		_debug_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct debug_vect_s debug = {};

		if (_debug_sub->update(&_debug_time, &debug)) {
			mavlink_debug_vect_t msg = {};

			msg.time_usec = debug.timestamp;
			memcpy(msg.name, debug.name, sizeof(msg.name));
			/* enforce null termination */
			msg.name[sizeof(msg.name) - 1] = '\0';
			msg.x = debug.x;
			msg.y = debug.y;
			msg.z = debug.z;

			mavlink_msg_debug_vect_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamDebugFloatArray : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamDebugFloatArray::get_name_static();
	}

	static const char *get_name_static()
	{
		return "DEBUG_FLOAT_ARRAY";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamDebugFloatArray(mavlink);
	}

	unsigned get_size()
	{
		return (_debug_time > 0) ? MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_debug_array_sub;
	uint64_t _debug_time;

	/* do not allow top copying this class */
	MavlinkStreamDebugFloatArray(MavlinkStreamDebugFloatArray &);
	MavlinkStreamDebugFloatArray &operator = (const MavlinkStreamDebugFloatArray &);

protected:
	explicit MavlinkStreamDebugFloatArray(Mavlink *mavlink) : MavlinkStream(mavlink),
		_debug_array_sub(_mavlink->add_orb_subscription(ORB_ID(debug_array))),
		_debug_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct debug_array_s debug = {};

		if (_debug_array_sub->update(&_debug_time, &debug)) {
			mavlink_debug_float_array_t msg = {};

			msg.time_usec = debug.timestamp;
			msg.array_id = debug.id;
			memcpy(msg.name, debug.name, sizeof(msg.name));
			/* enforce null termination */
			msg.name[sizeof(msg.name) - 1] = '\0';

			for (size_t i = 0; i < debug_array_s::ARRAY_SIZE; i++) {
				msg.data[i] = debug.data[i];
			}

			mavlink_msg_debug_float_array_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamNavControllerOutput : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamNavControllerOutput::get_name_static();
	}

	static const char *get_name_static()
	{
		return "NAV_CONTROLLER_OUTPUT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamNavControllerOutput(mavlink);
	}

	unsigned get_size()
	{
		return (_pos_ctrl_status_sub->is_published()) ?
		       MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_pos_ctrl_status_sub;
	MavlinkOrbSubscription *_tecs_status_sub;

	uint64_t _pos_ctrl_status_timestamp{0};
	uint64_t _tecs_status_timestamp{0};

	/* do not allow top copying this class */
	MavlinkStreamNavControllerOutput(MavlinkStreamNavControllerOutput &) = delete;
	MavlinkStreamNavControllerOutput &operator = (const MavlinkStreamNavControllerOutput &) = delete;

protected:
	explicit MavlinkStreamNavControllerOutput(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_ctrl_status_sub(_mavlink->add_orb_subscription(ORB_ID(position_controller_status))),
		_tecs_status_sub(_mavlink->add_orb_subscription(ORB_ID(tecs_status)))
	{}

	bool send(const hrt_abstime t)
	{
		position_controller_status_s pos_ctrl_status = {};
		tecs_status_s tecs_status = {};

		bool updated = false;
		updated |= _pos_ctrl_status_sub->update(&_pos_ctrl_status_timestamp, &pos_ctrl_status);
		updated |= _tecs_status_sub->update(&_tecs_status_timestamp, &tecs_status);

		if (updated) {
			mavlink_nav_controller_output_t msg = {};

			msg.nav_roll = math::degrees(pos_ctrl_status.nav_roll);
			msg.nav_pitch = math::degrees(pos_ctrl_status.nav_pitch);
			msg.nav_bearing = (int16_t)math::degrees(pos_ctrl_status.nav_bearing);
			msg.target_bearing = (int16_t)math::degrees(pos_ctrl_status.target_bearing);
			msg.wp_dist = (uint16_t)pos_ctrl_status.wp_dist;
			msg.xtrack_error = pos_ctrl_status.xtrack_error;
			msg.alt_error = tecs_status.altitude_filtered - tecs_status.altitude_sp;
			msg.aspd_error = tecs_status.airspeed_filtered - tecs_status.airspeed_sp;

			mavlink_msg_nav_controller_output_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamCameraCapture : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamCameraCapture::get_name_static();
	}

	static const char *get_name_static()
	{
		return "CAMERA_CAPTURE";
	}

	static uint16_t get_id_static()
	{
		return 0;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCameraCapture(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_COMMAND_LONG_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_status_sub;

	/* do not allow top copying this class */
	MavlinkStreamCameraCapture(MavlinkStreamCameraCapture &) = delete;
	MavlinkStreamCameraCapture &operator = (const MavlinkStreamCameraCapture &) = delete;

protected:
	explicit MavlinkStreamCameraCapture(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status)))
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_status_s status;

		if (_status_sub->update(&status)) {
			mavlink_command_long_t msg = {};

			msg.target_system = 0;
			msg.target_component = MAV_COMP_ID_ALL;
			msg.command = MAV_CMD_DO_CONTROL_VIDEO;
			msg.confirmation = 0;
			msg.param1 = 0;
			msg.param2 = 0;
			msg.param3 = 0;
			/* set camera capture ON/OFF depending on arming state */
			msg.param4 = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 1 : 0;
			msg.param5 = 0;
			msg.param6 = 0;
			msg.param7 = 0;

			mavlink_msg_command_long_send_struct(_mavlink->get_channel(), &msg);
		}

		return true;
	}
};

class MavlinkStreamDistanceSensor : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamDistanceSensor::get_name_static();
	}

	static const char *get_name_static()
	{
		return "DISTANCE_SENSOR";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_DISTANCE_SENSOR;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamDistanceSensor(mavlink);
	}

	unsigned get_size()
	{
		return _distance_sensor_sub->is_published() ? (MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_distance_sensor_sub;
	uint64_t _dist_sensor_time;

	/* do not allow top copying this class */
	MavlinkStreamDistanceSensor(MavlinkStreamDistanceSensor &) = delete;
	MavlinkStreamDistanceSensor &operator = (const MavlinkStreamDistanceSensor &) = delete;

protected:
	explicit MavlinkStreamDistanceSensor(Mavlink *mavlink) : MavlinkStream(mavlink),
		_distance_sensor_sub(_mavlink->add_orb_subscription(ORB_ID(distance_sensor))),
		_dist_sensor_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		distance_sensor_s dist_sensor;

		if (_distance_sensor_sub->update(&_dist_sensor_time, &dist_sensor)) {
			mavlink_distance_sensor_t msg = {};

			msg.time_boot_ms = dist_sensor.timestamp / 1000; /* us to ms */

			/* TODO: use correct ID here */
			msg.id = 0;

			switch (dist_sensor.type) {
			case MAV_DISTANCE_SENSOR_ULTRASOUND:
				msg.type = MAV_DISTANCE_SENSOR_ULTRASOUND;
				break;

			case MAV_DISTANCE_SENSOR_LASER:
				msg.type = MAV_DISTANCE_SENSOR_LASER;
				break;

			case MAV_DISTANCE_SENSOR_INFRARED:
				msg.type = MAV_DISTANCE_SENSOR_INFRARED;
				break;

			default:
				msg.type = MAV_DISTANCE_SENSOR_LASER;
				break;
			}

			msg.orientation = dist_sensor.orientation;
			msg.min_distance = dist_sensor.min_distance * 100.0f; /* m to cm */
			msg.max_distance = dist_sensor.max_distance * 100.0f; /* m to cm */
			msg.current_distance = dist_sensor.current_distance * 100.0f; /* m to cm */
			msg.covariance = dist_sensor.covariance;

			mavlink_msg_distance_sensor_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamExtendedSysState : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamExtendedSysState::get_name_static();
	}

	static const char *get_name_static()
	{
		return "EXTENDED_SYS_STATE";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_EXTENDED_SYS_STATE;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamExtendedSysState(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_EXTENDED_SYS_STATE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_status_sub;
	MavlinkOrbSubscription *_landed_sub;
	MavlinkOrbSubscription *_pos_sp_triplet_sub;
	MavlinkOrbSubscription *_control_mode_sub;
	mavlink_extended_sys_state_t _msg;

	/* do not allow top copying this class */
	MavlinkStreamExtendedSysState(MavlinkStreamExtendedSysState &) = delete;
	MavlinkStreamExtendedSysState &operator = (const MavlinkStreamExtendedSysState &) = delete;

protected:
	explicit MavlinkStreamExtendedSysState(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status))),
		_landed_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_land_detected))),
		_pos_sp_triplet_sub(_mavlink->add_orb_subscription(ORB_ID(position_setpoint_triplet))),
		_control_mode_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_control_mode))),
		_msg()
	{
		_msg.vtol_state = MAV_VTOL_STATE_UNDEFINED;
		_msg.landed_state = MAV_LANDED_STATE_ON_GROUND;
	}

	bool send(const hrt_abstime t)
	{
		bool updated = false;

		vehicle_status_s status;

		if (_status_sub->update(&status)) {
			updated = true;

			if (status.is_vtol) {
				if (!status.in_transition_mode && status.is_rotary_wing) {
					_msg.vtol_state = MAV_VTOL_STATE_MC;

				} else if (!status.in_transition_mode) {
					_msg.vtol_state = MAV_VTOL_STATE_FW;

				} else if (status.in_transition_mode && status.in_transition_to_fw) {
					_msg.vtol_state = MAV_VTOL_STATE_TRANSITION_TO_FW;

				} else if (status.in_transition_mode) {
					_msg.vtol_state = MAV_VTOL_STATE_TRANSITION_TO_MC;
				}
			}
		}

		vehicle_land_detected_s land_detected;

		if (_landed_sub->update(&land_detected)) {
			updated = true;

			if (land_detected.landed) {
				_msg.landed_state = MAV_LANDED_STATE_ON_GROUND;

			} else if (!land_detected.landed) {
				_msg.landed_state = MAV_LANDED_STATE_IN_AIR;

				vehicle_control_mode_s control_mode;
				position_setpoint_triplet_s pos_sp_triplet;

				if (_control_mode_sub->update(&control_mode) && _pos_sp_triplet_sub->update(&pos_sp_triplet)) {
					if (control_mode.flag_control_auto_enabled && pos_sp_triplet.current.valid) {
						if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
							_msg.landed_state = MAV_LANDED_STATE_TAKEOFF;

						} else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
							_msg.landed_state = MAV_LANDED_STATE_LANDING;
						}
					}
				}
			}
		}

		if (updated) {
			mavlink_msg_extended_sys_state_send_struct(_mavlink->get_channel(), &_msg);
		}

		return updated;
	}
};

class MavlinkStreamAltitude : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamAltitude::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ALTITUDE";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ALTITUDE;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAltitude(mavlink);
	}

	unsigned get_size()
	{
		return (_local_pos_time > 0) ? MAVLINK_MSG_ID_ALTITUDE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_local_pos_sub;
	MavlinkOrbSubscription *_home_sub;
	MavlinkOrbSubscription *_air_data_sub;

	uint64_t _local_pos_time{0};

	/* do not allow top copying this class */
	MavlinkStreamAltitude(MavlinkStreamAltitude &) = delete;
	MavlinkStreamAltitude &operator = (const MavlinkStreamAltitude &) = delete;

protected:
	explicit MavlinkStreamAltitude(Mavlink *mavlink) : MavlinkStream(mavlink),
		_local_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position))),
		_home_sub(_mavlink->add_orb_subscription(ORB_ID(home_position))),
		_air_data_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_air_data)))
	{}

	bool send(const hrt_abstime t)
	{
		mavlink_altitude_t msg = {};

		msg.altitude_monotonic = NAN;
		msg.altitude_amsl = NAN;
		msg.altitude_local = NAN;
		msg.altitude_relative = NAN;
		msg.altitude_terrain = NAN;
		msg.bottom_clearance = NAN;

		// always update monotonic altitude
		bool air_data_updated = false;
		vehicle_air_data_s air_data = {};
		_air_data_sub->update(&air_data);

		if (air_data.timestamp > 0) {
			msg.altitude_monotonic = air_data.baro_alt_meter;

			air_data_updated = true;
		}

		bool lpos_updated = false;

		vehicle_local_position_s local_pos;

		if (_local_pos_sub->update(&_local_pos_time, &local_pos)) {

			if (local_pos.z_valid) {
				if (local_pos.z_global) {
					msg.altitude_amsl = -local_pos.z + local_pos.ref_alt;

				} else {
					msg.altitude_amsl = msg.altitude_monotonic;
				}

				msg.altitude_local = -local_pos.z;

				home_position_s home = {};
				_home_sub->update(&home);

				if (home.valid_alt) {
					msg.altitude_relative = -(local_pos.z - home.z);

				} else {
					msg.altitude_relative = -local_pos.z;
				}

				if (local_pos.dist_bottom_valid) {
					msg.altitude_terrain = -local_pos.z - local_pos.dist_bottom;
					msg.bottom_clearance = local_pos.dist_bottom;
				}
			}

			lpos_updated = true;
		}

		// local position timeout after 10 ms
		// avoid publishing only baro altitude_monotonic if possible
		bool lpos_timeout = (hrt_elapsed_time(&_local_pos_time) > 10000);

		if (lpos_updated || (air_data_updated && lpos_timeout)) {
			msg.time_usec = hrt_absolute_time();
			mavlink_msg_altitude_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamWind : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamWind::get_name_static();
	}

	static const char *get_name_static()
	{
		return "WIND_COV";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_WIND_COV;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamWind(mavlink);
	}

	unsigned get_size()
	{
		return (_wind_estimate_time > 0) ? MAVLINK_MSG_ID_WIND_COV_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_wind_estimate_sub;
	uint64_t _wind_estimate_time;

	MavlinkOrbSubscription *_local_pos_sub;

	/* do not allow top copying this class */
	MavlinkStreamWind(MavlinkStreamWind &) = delete;
	MavlinkStreamWind &operator = (const MavlinkStreamWind &) = delete;

protected:
	explicit MavlinkStreamWind(Mavlink *mavlink) : MavlinkStream(mavlink),
		_wind_estimate_sub(_mavlink->add_orb_subscription(ORB_ID(wind_estimate))),
		_wind_estimate_time(0),
		_local_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position)))
	{}

	bool send(const hrt_abstime t)
	{
		wind_estimate_s wind_estimate;

		if (_wind_estimate_sub->update(&_wind_estimate_time, &wind_estimate)) {
			mavlink_wind_cov_t msg = {};

			msg.time_usec = wind_estimate.timestamp;

			msg.wind_x = wind_estimate.windspeed_north;
			msg.wind_y = wind_estimate.windspeed_east;
			msg.wind_z = 0.0f;

			msg.var_horiz = wind_estimate.variance_north + wind_estimate.variance_east;
			msg.var_vert = 0.0f;

			vehicle_local_position_s lpos = {};
			_local_pos_sub->update(&lpos);
			msg.wind_alt = (lpos.z_valid && lpos.z_global) ? (-lpos.z + lpos.ref_alt) : NAN;

			msg.horiz_accuracy = 0.0f;
			msg.vert_accuracy = 0.0f;

			mavlink_msg_wind_cov_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamMountOrientation : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamMountOrientation::get_name_static();
	}

	static const char *get_name_static()
	{
		return "MOUNT_ORIENTATION";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_MOUNT_ORIENTATION;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamMountOrientation(mavlink);
	}

	unsigned get_size()
	{
		return (_mount_orientation_time > 0) ? MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_mount_orientation_sub;
	uint64_t _mount_orientation_time;

	/* do not allow top copying this class */
	MavlinkStreamMountOrientation(MavlinkStreamMountOrientation &) = delete;
	MavlinkStreamMountOrientation &operator = (const MavlinkStreamMountOrientation &) = delete;

protected:
	explicit MavlinkStreamMountOrientation(Mavlink *mavlink) : MavlinkStream(mavlink),
		_mount_orientation_sub(_mavlink->add_orb_subscription(ORB_ID(mount_orientation))),
		_mount_orientation_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct mount_orientation_s mount_orientation;

		if (_mount_orientation_sub->update(&_mount_orientation_time, &mount_orientation)) {
			mavlink_mount_orientation_t msg = {};

			msg.roll = math::degrees(mount_orientation.attitude_euler_angle[0]);
			msg.pitch = math::degrees(mount_orientation.attitude_euler_angle[1]);
			msg.yaw = math::degrees(mount_orientation.attitude_euler_angle[2]);

			mavlink_msg_mount_orientation_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamGroundTruth : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamGroundTruth::get_name_static();
	}

	static const char *get_name_static()
	{
		return "GROUND_TRUTH";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HIL_STATE_QUATERNION;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGroundTruth(mavlink);
	}

	unsigned get_size()
	{
		return (_att_time > 0 || _gpos_time > 0) ? MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN +
		       MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_att_sub;
	MavlinkOrbSubscription *_gpos_sub;
	uint64_t _att_time;
	uint64_t _gpos_time;

	/* do not allow top copying this class */
	MavlinkStreamGroundTruth(MavlinkStreamGroundTruth &) = delete;
	MavlinkStreamGroundTruth &operator = (const MavlinkStreamGroundTruth &) = delete;

protected:
	explicit MavlinkStreamGroundTruth(Mavlink *mavlink) : MavlinkStream(mavlink),
		_att_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_attitude_groundtruth))),
		_gpos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_global_position_groundtruth))),
		_att_time(0),
		_gpos_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_attitude_s att = {};
		vehicle_global_position_s gpos = {};
		bool att_updated = _att_sub->update(&_att_time, &att);
		bool gpos_updated = _gpos_sub->update(&_gpos_time, &gpos);

		if (att_updated || gpos_updated) {
			mavlink_hil_state_quaternion_t msg = {};

			// vehicle_attitude -> hil_state_quaternion
			msg.attitude_quaternion[0] = att.q[0];
			msg.attitude_quaternion[1] = att.q[1];
			msg.attitude_quaternion[2] = att.q[2];
			msg.attitude_quaternion[3] = att.q[3];
			msg.rollspeed = att.rollspeed;
			msg.pitchspeed = att.pitchspeed;
			msg.yawspeed = att.yawspeed;

			// vehicle_global_position -> hil_state_quaternion
			msg.lat = gpos.lat;
			msg.lon = gpos.lon;
			msg.alt = gpos.alt;
			msg.vx = gpos.vel_n;
			msg.vy = gpos.vel_e;
			msg.vz = gpos.vel_d;
			msg.ind_airspeed = 0;
			msg.true_airspeed = 0;
			msg.xacc = 0;
			msg.yacc = 0;
			msg.zacc = 0;

			mavlink_msg_hil_state_quaternion_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamPing : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamPing::get_name_static();
	}

	static const char *get_name_static()
	{
		return "PING";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_PING;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamPing(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_PING_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool const_rate()
	{
		return true;
	}

private:
	uint32_t _sequence;

	/* do not allow top copying this class */
	MavlinkStreamPing(MavlinkStreamPing &) = delete;
	MavlinkStreamPing &operator = (const MavlinkStreamPing &) = delete;

protected:
	explicit MavlinkStreamPing(Mavlink *mavlink) : MavlinkStream(mavlink),
		_sequence(0)
	{}

	bool send(const hrt_abstime t)
	{
		mavlink_ping_t msg = {};

		msg.time_usec = hrt_absolute_time();
		msg.seq = _sequence++;
		msg.target_system = 0; // All systems
		msg.target_component = 0; // All components

		mavlink_msg_ping_send_struct(_mavlink->get_channel(), &msg);

		return true;
	}
};

class MavlinkStreamOrbitStatus : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamOrbitStatus::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ORBIT_EXECUTION_STATUS";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamOrbitStatus(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN +
		       MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_sub;
	uint64_t _orbit_status_time;

	/* do not allow top copying this class */
	MavlinkStreamOrbitStatus(MavlinkStreamOrbitStatus &);
	MavlinkStreamOrbitStatus &operator = (const MavlinkStreamOrbitStatus &);

protected:
	explicit MavlinkStreamOrbitStatus(Mavlink *mavlink) : MavlinkStream(mavlink),
		_sub(_mavlink->add_orb_subscription(ORB_ID(orbit_status))),
		_orbit_status_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct orbit_status_s _orbit_status;

		if (_sub->update(&_orbit_status_time, &_orbit_status)) {
			mavlink_orbit_execution_status_t _msg_orbit_execution_status = {};

			_msg_orbit_execution_status.time_usec = _orbit_status.timestamp;
			_msg_orbit_execution_status.radius = _orbit_status.radius;
			_msg_orbit_execution_status.frame  = _orbit_status.frame;
			_msg_orbit_execution_status.x = _orbit_status.x * 1e7;
			_msg_orbit_execution_status.y = _orbit_status.y * 1e7;
			_msg_orbit_execution_status.z = _orbit_status.z;

			mavlink_msg_orbit_execution_status_send_struct(_mavlink->get_channel(), &_msg_orbit_execution_status);
		}

		return true;
	}
};

static const StreamListItem streams_list[] = {
	StreamListItem(&MavlinkStreamHeartbeat::new_instance, &MavlinkStreamHeartbeat::get_name_static, &MavlinkStreamHeartbeat::get_id_static),
	StreamListItem(&MavlinkStreamStatustext::new_instance, &MavlinkStreamStatustext::get_name_static, &MavlinkStreamStatustext::get_id_static),
	StreamListItem(&MavlinkStreamCommandLong::new_instance, &MavlinkStreamCommandLong::get_name_static, &MavlinkStreamCommandLong::get_id_static),
	StreamListItem(&MavlinkStreamSysStatus::new_instance, &MavlinkStreamSysStatus::get_name_static, &MavlinkStreamSysStatus::get_id_static),
	StreamListItem(&MavlinkStreamHighresIMU::new_instance, &MavlinkStreamHighresIMU::get_name_static, &MavlinkStreamHighresIMU::get_id_static),
	StreamListItem(&MavlinkStreamScaledIMU::new_instance, &MavlinkStreamScaledIMU::get_name_static, &MavlinkStreamScaledIMU::get_id_static),
	StreamListItem(&MavlinkStreamScaledIMU2::new_instance, &MavlinkStreamScaledIMU2::get_name_static, &MavlinkStreamScaledIMU2::get_id_static),
	StreamListItem(&MavlinkStreamScaledIMU3::new_instance, &MavlinkStreamScaledIMU3::get_name_static, &MavlinkStreamScaledIMU3::get_id_static),
	StreamListItem(&MavlinkStreamAttitude::new_instance, &MavlinkStreamAttitude::get_name_static, &MavlinkStreamAttitude::get_id_static),
	StreamListItem(&MavlinkStreamAttitudeQuaternion::new_instance, &MavlinkStreamAttitudeQuaternion::get_name_static, &MavlinkStreamAttitudeQuaternion::get_id_static),
	StreamListItem(&MavlinkStreamVFRHUD::new_instance, &MavlinkStreamVFRHUD::get_name_static, &MavlinkStreamVFRHUD::get_id_static),
	StreamListItem(&MavlinkStreamGPSRawInt::new_instance, &MavlinkStreamGPSRawInt::get_name_static, &MavlinkStreamGPSRawInt::get_id_static),
	StreamListItem(&MavlinkStreamGPS2Raw::new_instance, &MavlinkStreamGPS2Raw::get_name_static, &MavlinkStreamGPS2Raw::get_id_static),
	StreamListItem(&MavlinkStreamSystemTime::new_instance, &MavlinkStreamSystemTime::get_name_static, &MavlinkStreamSystemTime::get_id_static),
	StreamListItem(&MavlinkStreamTimesync::new_instance, &MavlinkStreamTimesync::get_name_static, &MavlinkStreamTimesync::get_id_static),
	StreamListItem(&MavlinkStreamGlobalPositionInt::new_instance, &MavlinkStreamGlobalPositionInt::get_name_static, &MavlinkStreamGlobalPositionInt::get_id_static),
	StreamListItem(&MavlinkStreamLocalPositionNED::new_instance, &MavlinkStreamLocalPositionNED::get_name_static, &MavlinkStreamLocalPositionNED::get_id_static),
	StreamListItem(&MavlinkStreamOdometry::new_instance, &MavlinkStreamOdometry::get_name_static, &MavlinkStreamOdometry::get_id_static),
	StreamListItem(&MavlinkStreamEstimatorStatus::new_instance, &MavlinkStreamEstimatorStatus::get_name_static, &MavlinkStreamEstimatorStatus::get_id_static),
	StreamListItem(&MavlinkStreamAttPosMocap::new_instance, &MavlinkStreamAttPosMocap::get_name_static, &MavlinkStreamAttPosMocap::get_id_static),
	StreamListItem(&MavlinkStreamHomePosition::new_instance, &MavlinkStreamHomePosition::get_name_static, &MavlinkStreamHomePosition::get_id_static),
	StreamListItem(&MavlinkStreamServoOutputRaw<0>::new_instance, &MavlinkStreamServoOutputRaw<0>::get_name_static, &MavlinkStreamServoOutputRaw<0>::get_id_static),
	StreamListItem(&MavlinkStreamServoOutputRaw<1>::new_instance, &MavlinkStreamServoOutputRaw<1>::get_name_static, &MavlinkStreamServoOutputRaw<1>::get_id_static),
	StreamListItem(&MavlinkStreamServoOutputRaw<2>::new_instance, &MavlinkStreamServoOutputRaw<2>::get_name_static, &MavlinkStreamServoOutputRaw<2>::get_id_static),
	StreamListItem(&MavlinkStreamServoOutputRaw<3>::new_instance, &MavlinkStreamServoOutputRaw<3>::get_name_static, &MavlinkStreamServoOutputRaw<3>::get_id_static),
	StreamListItem(&MavlinkStreamHILActuatorControls::new_instance, &MavlinkStreamHILActuatorControls::get_name_static, &MavlinkStreamHILActuatorControls::get_id_static),
	StreamListItem(&MavlinkStreamPositionTargetGlobalInt::new_instance, &MavlinkStreamPositionTargetGlobalInt::get_name_static, &MavlinkStreamPositionTargetGlobalInt::get_id_static),
	StreamListItem(&MavlinkStreamLocalPositionSetpoint::new_instance, &MavlinkStreamLocalPositionSetpoint::get_name_static, &MavlinkStreamLocalPositionSetpoint::get_id_static),
	StreamListItem(&MavlinkStreamAttitudeTarget::new_instance, &MavlinkStreamAttitudeTarget::get_name_static, &MavlinkStreamAttitudeTarget::get_id_static),
	StreamListItem(&MavlinkStreamRCChannels::new_instance, &MavlinkStreamRCChannels::get_name_static, &MavlinkStreamRCChannels::get_id_static),
	StreamListItem(&MavlinkStreamManualControl::new_instance, &MavlinkStreamManualControl::get_name_static, &MavlinkStreamManualControl::get_id_static),
	StreamListItem(&MavlinkStreamTrajectoryRepresentationWaypoints::new_instance, &MavlinkStreamTrajectoryRepresentationWaypoints::get_name_static, &MavlinkStreamTrajectoryRepresentationWaypoints::get_id_static),
	StreamListItem(&MavlinkStreamOpticalFlowRad::new_instance, &MavlinkStreamOpticalFlowRad::get_name_static, &MavlinkStreamOpticalFlowRad::get_id_static),
	StreamListItem(&MavlinkStreamActuatorControlTarget<0>::new_instance, &MavlinkStreamActuatorControlTarget<0>::get_name_static, &MavlinkStreamActuatorControlTarget<0>::get_id_static),
	StreamListItem(&MavlinkStreamActuatorControlTarget<1>::new_instance, &MavlinkStreamActuatorControlTarget<1>::get_name_static, &MavlinkStreamActuatorControlTarget<1>::get_id_static),
	StreamListItem(&MavlinkStreamActuatorControlTarget<2>::new_instance, &MavlinkStreamActuatorControlTarget<2>::get_name_static, &MavlinkStreamActuatorControlTarget<2>::get_id_static),
	StreamListItem(&MavlinkStreamActuatorControlTarget<3>::new_instance, &MavlinkStreamActuatorControlTarget<3>::get_name_static, &MavlinkStreamActuatorControlTarget<3>::get_id_static),
	StreamListItem(&MavlinkStreamNamedValueFloat::new_instance, &MavlinkStreamNamedValueFloat::get_name_static, &MavlinkStreamNamedValueFloat::get_id_static),
	StreamListItem(&MavlinkStreamDebug::new_instance, &MavlinkStreamDebug::get_name_static, &MavlinkStreamDebug::get_id_static),
	StreamListItem(&MavlinkStreamDebugVect::new_instance, &MavlinkStreamDebugVect::get_name_static, &MavlinkStreamDebugVect::get_id_static),
	StreamListItem(&MavlinkStreamDebugFloatArray::new_instance, &MavlinkStreamDebugFloatArray::get_name_static, &MavlinkStreamDebugFloatArray::get_id_static),
	StreamListItem(&MavlinkStreamNavControllerOutput::new_instance, &MavlinkStreamNavControllerOutput::get_name_static, &MavlinkStreamNavControllerOutput::get_id_static),
	StreamListItem(&MavlinkStreamCameraCapture::new_instance, &MavlinkStreamCameraCapture::get_name_static, &MavlinkStreamCameraCapture::get_id_static),
	StreamListItem(&MavlinkStreamCameraTrigger::new_instance, &MavlinkStreamCameraTrigger::get_name_static, &MavlinkStreamCameraTrigger::get_id_static),
	StreamListItem(&MavlinkStreamCameraImageCaptured::new_instance, &MavlinkStreamCameraImageCaptured::get_name_static, &MavlinkStreamCameraImageCaptured::get_id_static),
	StreamListItem(&MavlinkStreamDistanceSensor::new_instance, &MavlinkStreamDistanceSensor::get_name_static, &MavlinkStreamDistanceSensor::get_id_static),
	StreamListItem(&MavlinkStreamExtendedSysState::new_instance, &MavlinkStreamExtendedSysState::get_name_static, &MavlinkStreamExtendedSysState::get_id_static),
	StreamListItem(&MavlinkStreamAltitude::new_instance, &MavlinkStreamAltitude::get_name_static, &MavlinkStreamAltitude::get_id_static),
	StreamListItem(&MavlinkStreamADSBVehicle::new_instance, &MavlinkStreamADSBVehicle::get_name_static, &MavlinkStreamADSBVehicle::get_id_static),
	StreamListItem(&MavlinkStreamUTMGlobalPosition::new_instance, &MavlinkStreamUTMGlobalPosition::get_name_static, &MavlinkStreamUTMGlobalPosition::get_id_static),
	StreamListItem(&MavlinkStreamCollision::new_instance, &MavlinkStreamCollision::get_name_static, &MavlinkStreamCollision::get_id_static),
	StreamListItem(&MavlinkStreamWind::new_instance, &MavlinkStreamWind::get_name_static, &MavlinkStreamWind::get_id_static),
	StreamListItem(&MavlinkStreamMountOrientation::new_instance, &MavlinkStreamMountOrientation::get_name_static, &MavlinkStreamMountOrientation::get_id_static),
	StreamListItem(&MavlinkStreamHighLatency2::new_instance, &MavlinkStreamHighLatency2::get_name_static, &MavlinkStreamHighLatency2::get_id_static),
	StreamListItem(&MavlinkStreamGroundTruth::new_instance, &MavlinkStreamGroundTruth::get_name_static, &MavlinkStreamGroundTruth::get_id_static),
	StreamListItem(&MavlinkStreamPing::new_instance, &MavlinkStreamPing::get_name_static, &MavlinkStreamPing::get_id_static),
	StreamListItem(&MavlinkStreamOrbitStatus::new_instance, &MavlinkStreamOrbitStatus::get_name_static, &MavlinkStreamOrbitStatus::get_id_static)
};

const char *get_stream_name(const uint16_t msg_id)
{
	// search for stream with specified msg id in supported streams list
	for (const auto &stream : streams_list) {
		if (msg_id == stream.get_id()) {
			return stream.get_name();
		}
	}

	return nullptr;
}

MavlinkStream *create_mavlink_stream(const char *stream_name, Mavlink *mavlink)
{
	// search for stream with specified name in supported streams list
	if (stream_name != nullptr) {
		for (const auto &stream : streams_list) {
			if (strcmp(stream_name, stream.get_name()) == 0) {
				return stream.new_instance(mavlink);
			}
		}
	}

	return nullptr;
}
/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file mavlink_mission.cpp
 * MAVLink mission manager implementation.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

#include "mavlink_mission.h"
#include "mavlink_main.h"

#include <lib/ecl/geo/geo.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <px4_defines.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <navigator/navigation.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>

using matrix::wrap_2pi;

dm_item_t MavlinkMissionManager::_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
bool MavlinkMissionManager::_dataman_init = false;
uint16_t MavlinkMissionManager::_count[3] = { 0, 0, 0 };
int32_t MavlinkMissionManager::_current_seq = 0;
bool MavlinkMissionManager::_transfer_in_progress = false;
constexpr uint16_t MavlinkMissionManager::MAX_COUNT[];
uint16_t MavlinkMissionManager::_geofence_update_counter = 0;

#define CHECK_SYSID_COMPID_MISSION(_msg)		(_msg.target_system == mavlink_system.sysid && \
		((_msg.target_component == mavlink_system.compid) || \
		 (_msg.target_component == MAV_COMP_ID_MISSIONPLANNER) || \
		 (_msg.target_component == MAV_COMP_ID_ALL)))

MavlinkMissionManager::MavlinkMissionManager(Mavlink *mavlink) :
	_mavlink(mavlink)
{
	_offboard_mission_sub = orb_subscribe(ORB_ID(mission));
	_mission_result_sub = orb_subscribe(ORB_ID(mission_result));

	init_offboard_mission();
}

MavlinkMissionManager::~MavlinkMissionManager()
{
	orb_unsubscribe(_mission_result_sub);
	orb_unadvertise(_offboard_mission_pub);
}

void
MavlinkMissionManager::init_offboard_mission()
{
	if (!_dataman_init) {
		_dataman_init = true;

		/* lock MISSION_STATE item */
		int dm_lock_ret = dm_lock(DM_KEY_MISSION_STATE);

		if (dm_lock_ret != 0) {
			PX4_ERR("DM_KEY_MISSION_STATE lock failed");
		}

		mission_s mission_state;
		int ret = dm_read(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_s));

		/* unlock MISSION_STATE item */
		if (dm_lock_ret == 0) {
			dm_unlock(DM_KEY_MISSION_STATE);
		}

		if (ret > 0) {
			_dataman_id = (dm_item_t)mission_state.dataman_id;
			_count[MAV_MISSION_TYPE_MISSION] = mission_state.count;
			_current_seq = mission_state.current_seq;

		} else if (ret < 0) {
			PX4_ERR("offboard mission init failed (%i)", ret);
		}

		load_geofence_stats();

		load_safepoint_stats();
	}

	_my_dataman_id = _dataman_id;
}

int
MavlinkMissionManager::load_geofence_stats()
{
	mission_stats_entry_s stats;
	// initialize fence points count
	int ret = dm_read(DM_KEY_FENCE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));

	if (ret == sizeof(mission_stats_entry_s)) {
		_count[MAV_MISSION_TYPE_FENCE] = stats.num_items;
		_geofence_update_counter = stats.update_counter;
	}

	return ret;
}

int
MavlinkMissionManager::load_safepoint_stats()
{
	mission_stats_entry_s stats;
	// initialize safe points count
	int ret = dm_read(DM_KEY_SAFE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));

	if (ret == sizeof(mission_stats_entry_s)) {
		_count[MAV_MISSION_TYPE_RALLY] = stats.num_items;
	}

	return ret;
}

/**
 * Publish mission topic to notify navigator about changes.
 */
int
MavlinkMissionManager::update_active_mission(dm_item_t dataman_id, uint16_t count, int32_t seq)
{
	mission_s mission;
	mission.timestamp = hrt_absolute_time();
	mission.dataman_id = dataman_id;
	mission.count = count;
	mission.current_seq = seq;

	/* update mission state in dataman */

	/* lock MISSION_STATE item */
	int dm_lock_ret = dm_lock(DM_KEY_MISSION_STATE);

	if (dm_lock_ret != 0) {
		PX4_ERR("DM_KEY_MISSION_STATE lock failed");
	}

	int res = dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission, sizeof(mission_s));

	/* unlock MISSION_STATE item */
	if (dm_lock_ret == 0) {
		dm_unlock(DM_KEY_MISSION_STATE);
	}

	if (res == sizeof(mission_s)) {
		/* update active mission state */
		_dataman_id = dataman_id;
		_count[MAV_MISSION_TYPE_MISSION] = count;
		_current_seq = seq;
		_my_dataman_id = _dataman_id;

		/* mission state saved successfully, publish offboard_mission topic */
		if (_offboard_mission_pub == nullptr) {
			_offboard_mission_pub = orb_advertise(ORB_ID(mission), &mission);

		} else {
			orb_publish(ORB_ID(mission), _offboard_mission_pub, &mission);
		}

		return PX4_OK;

	} else {
		PX4_ERR("WPM: can't save mission state");

		if (_filesystem_errcount++ < FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT) {
			_mavlink->send_statustext_critical("Mission storage: Unable to write to microSD");
		}

		return PX4_ERROR;
	}
}
int
MavlinkMissionManager::update_geofence_count(unsigned count)
{
	mission_stats_entry_s stats;
	stats.num_items = count;
	stats.update_counter = ++_geofence_update_counter; // this makes sure navigator will reload the fence data

	/* update stats in dataman */
	int res = dm_write(DM_KEY_FENCE_POINTS, 0, DM_PERSIST_POWER_ON_RESET, &stats, sizeof(mission_stats_entry_s));

	if (res == sizeof(mission_stats_entry_s)) {
		_count[MAV_MISSION_TYPE_FENCE] = count;

	} else {

		if (_filesystem_errcount++ < FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT) {
			_mavlink->send_statustext_critical("Mission storage: Unable to write to microSD");
		}

		return PX4_ERROR;
	}

	return PX4_OK;

}

int
MavlinkMissionManager::update_safepoint_count(unsigned count)
{
	mission_stats_entry_s stats;
	stats.num_items = count;

	/* update stats in dataman */
	int res = dm_write(DM_KEY_SAFE_POINTS, 0, DM_PERSIST_POWER_ON_RESET, &stats, sizeof(mission_stats_entry_s));

	if (res == sizeof(mission_stats_entry_s)) {
		_count[MAV_MISSION_TYPE_RALLY] = count;

	} else {

		if (_filesystem_errcount++ < FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT) {
			_mavlink->send_statustext_critical("Mission storage: Unable to write to microSD");
		}

		return PX4_ERROR;
	}

	return PX4_OK;
}

void
MavlinkMissionManager::send_mission_ack(uint8_t sysid, uint8_t compid, uint8_t type)
{
	mavlink_mission_ack_t wpa;

	wpa.target_system = sysid;
	wpa.target_component = compid;
	wpa.type = type;
	wpa.mission_type = _mission_type;

	mavlink_msg_mission_ack_send_struct(_mavlink->get_channel(), &wpa);

	PX4_DEBUG("WPM: Send MISSION_ACK type %u to ID %u", wpa.type, wpa.target_system);
}


void
MavlinkMissionManager::send_mission_current(uint16_t seq)
{
	unsigned item_count = _count[MAV_MISSION_TYPE_MISSION];

	if (seq < item_count) {
		mavlink_mission_current_t wpc;

		wpc.seq = seq;

		mavlink_msg_mission_current_send_struct(_mavlink->get_channel(), &wpc);

	} else if (seq == 0 && item_count == 0) {
		/* don't broadcast if no WPs */

	} else {
		PX4_DEBUG("WPM: Send MISSION_CURRENT ERROR: seq %u out of bounds", seq);

		_mavlink->send_statustext_critical("ERROR: wp index out of bounds");
	}
}


void
MavlinkMissionManager::send_mission_count(uint8_t sysid, uint8_t compid, uint16_t count, MAV_MISSION_TYPE mission_type)
{
	_time_last_sent = hrt_absolute_time();

	mavlink_mission_count_t wpc;

	wpc.target_system = sysid;
	wpc.target_component = compid;
	wpc.count = count;
	wpc.mission_type = mission_type;

	mavlink_msg_mission_count_send_struct(_mavlink->get_channel(), &wpc);

	PX4_DEBUG("WPM: Send MISSION_COUNT %u to ID %u, mission type=%i", wpc.count, wpc.target_system, mission_type);
}


void
MavlinkMissionManager::send_mission_item(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	mission_item_s mission_item = {};
	bool read_success = false;

	switch (_mission_type) {

	case MAV_MISSION_TYPE_MISSION: {
			read_success = dm_read(_dataman_id, seq, &mission_item, sizeof(mission_item_s)) == sizeof(mission_item_s);
		}
		break;

	case MAV_MISSION_TYPE_FENCE: { // Read a geofence point
			mission_fence_point_s mission_fence_point;
			read_success = dm_read(DM_KEY_FENCE_POINTS, seq + 1, &mission_fence_point, sizeof(mission_fence_point_s)) ==
				       sizeof(mission_fence_point_s);

			mission_item.nav_cmd = mission_fence_point.nav_cmd;
			mission_item.frame = mission_fence_point.frame;
			mission_item.lat = mission_fence_point.lat;
			mission_item.lon = mission_fence_point.lon;
			mission_item.altitude = mission_fence_point.alt;

			if (mission_fence_point.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION ||
			    mission_fence_point.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION) {
				mission_item.vertex_count = mission_fence_point.vertex_count;

			} else {
				mission_item.circle_radius = mission_fence_point.circle_radius;
			}
		}
		break;

	case MAV_MISSION_TYPE_RALLY: { // Read a safe point / rally point
			mission_save_point_s mission_save_point;
			read_success = dm_read(DM_KEY_SAFE_POINTS, seq + 1, &mission_save_point, sizeof(mission_save_point_s)) ==
				       sizeof(mission_save_point_s);

			mission_item.nav_cmd = MAV_CMD_NAV_RALLY_POINT;
			mission_item.frame = mission_save_point.frame;
			mission_item.lat = mission_save_point.lat;
			mission_item.lon = mission_save_point.lon;
			mission_item.altitude = mission_save_point.alt;
		}
		break;

	default:
		_mavlink->send_statustext_critical("Received unknown mission type, abort.");
		break;
	}

	if (read_success) {
		_time_last_sent = hrt_absolute_time();

		if (_int_mode) {
			mavlink_mission_item_int_t wp;
			format_mavlink_mission_item(&mission_item, reinterpret_cast<mavlink_mission_item_t *>(&wp));

			wp.target_system = sysid;
			wp.target_component = compid;
			wp.seq = seq;
			wp.current = (_current_seq == seq) ? 1 : 0;

			mavlink_msg_mission_item_int_send_struct(_mavlink->get_channel(), &wp);

			PX4_DEBUG("WPM: Send MISSION_ITEM_INT seq %u to ID %u", wp.seq, wp.target_system);

		} else {
			mavlink_mission_item_t wp;
			format_mavlink_mission_item(&mission_item, &wp);

			wp.target_system = sysid;
			wp.target_component = compid;
			wp.seq = seq;
			wp.current = (_current_seq == seq) ? 1 : 0;

			mavlink_msg_mission_item_send_struct(_mavlink->get_channel(), &wp);

			PX4_DEBUG("WPM: Send MISSION_ITEM seq %u to ID %u", wp.seq, wp.target_system);
		}

	} else {
		send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);

		if (_filesystem_errcount++ < FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT) {
			_mavlink->send_statustext_critical("Mission storage: Unable to read from microSD");
		}

		PX4_DEBUG("WPM: Send MISSION_ITEM ERROR: could not read seq %u from dataman ID %i", seq, _dataman_id);
	}
}

uint16_t
MavlinkMissionManager::current_max_item_count()
{
	if (_mission_type >= sizeof(MAX_COUNT) / sizeof(MAX_COUNT[0])) {
		PX4_ERR("WPM: MAX_COUNT out of bounds (%u)", _mission_type);
		return 0;
	}

	return MAX_COUNT[_mission_type];
}

uint16_t
MavlinkMissionManager::current_item_count()
{
	if (_mission_type >= sizeof(_count) / sizeof(_count[0])) {
		PX4_ERR("WPM: _count out of bounds (%u)", _mission_type);
		return 0;
	}

	return _count[_mission_type];
}

void
MavlinkMissionManager::send_mission_request(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	if (seq < current_max_item_count()) {
		_time_last_sent = hrt_absolute_time();

		if (_int_mode) {
			mavlink_mission_request_int_t wpr;
			wpr.target_system = sysid;
			wpr.target_component = compid;
			wpr.seq = seq;
			wpr.mission_type = _mission_type;
			mavlink_msg_mission_request_int_send_struct(_mavlink->get_channel(), &wpr);

			PX4_DEBUG("WPM: Send MISSION_REQUEST_INT seq %u to ID %u", wpr.seq, wpr.target_system);

		} else {

			mavlink_mission_request_t wpr;
			wpr.target_system = sysid;
			wpr.target_component = compid;
			wpr.seq = seq;
			wpr.mission_type = _mission_type;

			mavlink_msg_mission_request_send_struct(_mavlink->get_channel(), &wpr);

			PX4_DEBUG("WPM: Send MISSION_REQUEST seq %u to ID %u", wpr.seq, wpr.target_system);
		}

	} else {
		_mavlink->send_statustext_critical("ERROR: Waypoint index exceeds list capacity");

		PX4_DEBUG("WPM: Send MISSION_REQUEST ERROR: seq %u exceeds list capacity", seq);
	}
}


void
MavlinkMissionManager::send_mission_item_reached(uint16_t seq)
{
	mavlink_mission_item_reached_t wp_reached;

	wp_reached.seq = seq;

	mavlink_msg_mission_item_reached_send_struct(_mavlink->get_channel(), &wp_reached);

	PX4_DEBUG("WPM: Send MISSION_ITEM_REACHED reached_seq %u", wp_reached.seq);
}


void
MavlinkMissionManager::send(const hrt_abstime now)
{
	// do not send anything over high latency communication
	if (_mavlink->get_mode() == Mavlink::MAVLINK_MODE_IRIDIUM) {
		return;
	}

	bool updated = false;
	orb_check(_mission_result_sub, &updated);

	if (updated) {
		mission_result_s mission_result;
		orb_copy(ORB_ID(mission_result), _mission_result_sub, &mission_result);

		if (_current_seq != mission_result.seq_current) {
			_current_seq = mission_result.seq_current;

			PX4_DEBUG("WPM: got mission result, new current_seq: %u", _current_seq);
		}

		if (_last_reached != mission_result.seq_reached) {
			_last_reached = mission_result.seq_reached;
			_reached_sent_count = 0;

			if (_last_reached >= 0) {
				send_mission_item_reached((uint16_t)mission_result.seq_reached);
			}

			PX4_DEBUG("WPM: got mission result, new seq_reached: %d", _last_reached);
		}

		send_mission_current(_current_seq);

		if (mission_result.item_do_jump_changed) {
			/* send a mission item again if the remaining DO_JUMPs has changed */
			send_mission_item(_transfer_partner_sysid, _transfer_partner_compid,
					  (uint16_t)mission_result.item_changed_index);
		}

	} else {
		if (_slow_rate_limiter.check(now)) {
			send_mission_current(_current_seq);

			// send the reached message another 10 times
			if (_last_reached >= 0 && (_reached_sent_count < 10)) {
				send_mission_item_reached((uint16_t)_last_reached);
				_reached_sent_count++;
			}
		}
	}

	/* check for timed-out operations */
	if (_state == MAVLINK_WPM_STATE_GETLIST && (_time_last_sent > 0)
	    && hrt_elapsed_time(&_time_last_sent) > MAVLINK_MISSION_RETRY_TIMEOUT_DEFAULT) {

		// try to request item again after timeout
		send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);

	} else if (_state != MAVLINK_WPM_STATE_IDLE && (_time_last_recv > 0)
		   && hrt_elapsed_time(&_time_last_recv) > MAVLINK_MISSION_PROTOCOL_TIMEOUT_DEFAULT) {

		_mavlink->send_statustext_critical("Operation timeout");

		PX4_DEBUG("WPM: Last operation (state=%u) timed out, changing state to MAVLINK_WPM_STATE_IDLE", _state);

		switch_to_idle_state();

		// since we are giving up, reset this state also, so another request can be started.
		_transfer_in_progress = false;

	} else if (_state == MAVLINK_WPM_STATE_IDLE) {
		// reset flags
		_time_last_sent = 0;
		_time_last_recv = 0;
	}
}


void
MavlinkMissionManager::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_MISSION_ACK:
		handle_mission_ack(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
		handle_mission_set_current(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
		handle_mission_request_list(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_REQUEST:
		handle_mission_request(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
		handle_mission_request_int(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_COUNT:
		handle_mission_count(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_ITEM:
		handle_mission_item(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_ITEM_INT:
		handle_mission_item_int(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
		handle_mission_clear_all(msg);
		break;

	default:
		break;
	}
}


void
MavlinkMissionManager::handle_mission_ack(const mavlink_message_t *msg)
{
	mavlink_mission_ack_t wpa;
	mavlink_msg_mission_ack_decode(msg, &wpa);

	if (CHECK_SYSID_COMPID_MISSION(wpa)) {
		if ((msg->sysid == _transfer_partner_sysid && msg->compid == _transfer_partner_compid)) {
			if (_state == MAVLINK_WPM_STATE_SENDLIST && _mission_type == wpa.mission_type) {
				_time_last_recv = hrt_absolute_time();

				if (_transfer_seq == current_item_count()) {
					PX4_DEBUG("WPM: MISSION_ACK OK all items sent, switch to state IDLE");

				} else {
					_mavlink->send_statustext_critical("WPM: ERR: not all items sent -> IDLE");

					PX4_DEBUG("WPM: MISSION_ACK ERROR: not all items sent, switch to state IDLE anyway");
				}

				switch_to_idle_state();

			} else if (_state == MAVLINK_WPM_STATE_GETLIST) {

				// INT or float mode is not supported
				if (wpa.type == MAV_MISSION_UNSUPPORTED) {

					if (_int_mode) {
						_int_mode = false;
						send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);

					} else {
						_int_mode = true;
						send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
					}

				} else if (wpa.type != MAV_MISSION_ACCEPTED) {
					PX4_WARN("Mission ack result was %d", wpa.type);
				}
			}

		} else {
			_mavlink->send_statustext_critical("REJ. WP CMD: partner id mismatch");

			PX4_DEBUG("WPM: MISSION_ACK ERR: ID mismatch");
		}
	}
}


void
MavlinkMissionManager::handle_mission_set_current(const mavlink_message_t *msg)
{
	mavlink_mission_set_current_t wpc;
	mavlink_msg_mission_set_current_decode(msg, &wpc);

	if (CHECK_SYSID_COMPID_MISSION(wpc)) {
		if (_state == MAVLINK_WPM_STATE_IDLE) {
			_time_last_recv = hrt_absolute_time();

			if (wpc.seq < _count[MAV_MISSION_TYPE_MISSION]) {
				if (update_active_mission(_dataman_id, _count[MAV_MISSION_TYPE_MISSION], wpc.seq) == PX4_OK) {
					PX4_DEBUG("WPM: MISSION_SET_CURRENT seq=%d OK", wpc.seq);

				} else {
					PX4_DEBUG("WPM: MISSION_SET_CURRENT seq=%d ERROR", wpc.seq);

					_mavlink->send_statustext_critical("WPM: WP CURR CMD: Error setting ID");
				}

			} else {
				PX4_ERR("WPM: MISSION_SET_CURRENT seq=%d ERROR: not in list", wpc.seq);

				_mavlink->send_statustext_critical("WPM: WP CURR CMD: Not in list");
			}

		} else {
			PX4_DEBUG("WPM: MISSION_SET_CURRENT ERROR: busy");

			_mavlink->send_statustext_critical("WPM: IGN WP CURR CMD: Busy");
		}
	}
}


void
MavlinkMissionManager::handle_mission_request_list(const mavlink_message_t *msg)
{
	mavlink_mission_request_list_t wprl;
	mavlink_msg_mission_request_list_decode(msg, &wprl);

	if (CHECK_SYSID_COMPID_MISSION(wprl)) {
		if (_state == MAVLINK_WPM_STATE_IDLE || (_state == MAVLINK_WPM_STATE_SENDLIST
				&& (uint8_t)_mission_type == wprl.mission_type)) {
			_time_last_recv = hrt_absolute_time();

			_state = MAVLINK_WPM_STATE_SENDLIST;
			_mission_type = (MAV_MISSION_TYPE)wprl.mission_type;

			// make sure our item counts are up-to-date
			switch (_mission_type) {
			case MAV_MISSION_TYPE_FENCE:
				load_geofence_stats();
				break;

			case MAV_MISSION_TYPE_RALLY:
				load_safepoint_stats();
				break;

			default:
				break;
			}

			_transfer_seq = 0;
			_transfer_count = current_item_count();
			_transfer_partner_sysid = msg->sysid;
			_transfer_partner_compid = msg->compid;

			if (_transfer_count > 0) {
				PX4_DEBUG("WPM: MISSION_REQUEST_LIST OK, %u mission items to send, mission type=%i", _transfer_count, _mission_type);

			} else {
				PX4_DEBUG("WPM: MISSION_REQUEST_LIST OK nothing to send, mission is empty, mission type=%i", _mission_type);
			}

			send_mission_count(msg->sysid, msg->compid, _transfer_count, _mission_type);

		} else {
			PX4_DEBUG("WPM: MISSION_REQUEST_LIST ERROR: busy");

			_mavlink->send_statustext_critical("IGN REQUEST LIST: Busy");
		}
	}
}


void
MavlinkMissionManager::handle_mission_request(const mavlink_message_t *msg)
{
	// The request comes in the old float mode, so we switch to it.
	if (_int_mode) {
		_int_mode = false;
	}

	handle_mission_request_both(msg);
}

void
MavlinkMissionManager::handle_mission_request_int(const mavlink_message_t *msg)
{
	// The request comes in the new int mode, so we switch to it.
	if (!_int_mode) {
		_int_mode = true;
	}

	handle_mission_request_both(msg);
}

void
MavlinkMissionManager::handle_mission_request_both(const mavlink_message_t *msg)
{
	/* The mavlink_message_t could also be a mavlink_mission_request_int_t, however the structs
	 * are basically the same, so we can ignore it. */
	mavlink_mission_request_t wpr;
	mavlink_msg_mission_request_decode(msg, &wpr);

	if (CHECK_SYSID_COMPID_MISSION(wpr)) {
		if (msg->sysid == _transfer_partner_sysid && msg->compid == _transfer_partner_compid) {
			if (_state == MAVLINK_WPM_STATE_SENDLIST) {

				if (_mission_type != wpr.mission_type) {
					PX4_WARN("WPM: Unexpected mission type (%u %u)", wpr.mission_type, _mission_type);
					return;
				}

				_time_last_recv = hrt_absolute_time();

				/* _transfer_seq contains sequence of expected request */
				if (wpr.seq == _transfer_seq && _transfer_seq < _transfer_count) {
					PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) seq %u from ID %u", wpr.seq, msg->sysid);

					_transfer_seq++;

				} else if (wpr.seq == _transfer_seq - 1) {
					PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) seq %u from ID %u (again)", wpr.seq, msg->sysid);

				} else {
					if (_transfer_seq > 0 && _transfer_seq < _transfer_count) {
						PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: seq %u from ID %u unexpected, must be %i or %i", wpr.seq, msg->sysid,
							  _transfer_seq - 1, _transfer_seq);

					} else if (_transfer_seq <= 0) {
						PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: seq %u from ID %u unexpected, must be %i", wpr.seq, msg->sysid,
							  _transfer_seq);

					} else {
						PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: seq %u from ID %u unexpected, must be %i", wpr.seq, msg->sysid,
							  _transfer_seq - 1);
					}

					switch_to_idle_state();

					send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
					_mavlink->send_statustext_critical("WPM: REJ. CMD: Req. WP was unexpected");
					return;
				}

				/* double check bounds in case of items count changed */
				if (wpr.seq < current_item_count()) {
					send_mission_item(_transfer_partner_sysid, _transfer_partner_compid, wpr.seq);

				} else {
					PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: seq %u out of bound [%u, %u]", wpr.seq, wpr.seq,
						  current_item_count() - 1);

					switch_to_idle_state();

					send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
					_mavlink->send_statustext_critical("WPM: REJ. CMD: Req. WP was unexpected");
				}

			} else if (_state == MAVLINK_WPM_STATE_IDLE) {
				PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: no transfer");

				// Silently ignore this as some OSDs have buggy mission protocol implementations
				//_mavlink->send_statustext_critical("IGN MISSION_ITEM_REQUEST(_INT): No active transfer");

			} else {
				PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: busy (state %d).", _state);

				_mavlink->send_statustext_critical("WPM: REJ. CMD: Busy");
			}

		} else {
			_mavlink->send_statustext_critical("WPM: REJ. CMD: partner id mismatch");

			PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: rejected, partner ID mismatch");
		}
	}
}


void
MavlinkMissionManager::handle_mission_count(const mavlink_message_t *msg)
{
	mavlink_mission_count_t wpc;
	mavlink_msg_mission_count_decode(msg, &wpc);

	if (CHECK_SYSID_COMPID_MISSION(wpc)) {
		if (_state == MAVLINK_WPM_STATE_IDLE) {
			_time_last_recv = hrt_absolute_time();

			if (_transfer_in_progress) {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
				return;
			}

			_transfer_in_progress = true;
			_mission_type = (MAV_MISSION_TYPE)wpc.mission_type;

			if (wpc.count > current_max_item_count()) {
				PX4_DEBUG("WPM: MISSION_COUNT ERROR: too many waypoints (%d), supported: %d", wpc.count, current_max_item_count());

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_NO_SPACE);
				_transfer_in_progress = false;
				return;
			}

			if (wpc.count == 0) {
				PX4_DEBUG("WPM: MISSION_COUNT 0, clearing waypoints list and staying in state MAVLINK_WPM_STATE_IDLE");

				switch (_mission_type) {
				case MAV_MISSION_TYPE_MISSION:

					/* alternate dataman ID anyway to let navigator know about changes */

					if (_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0) {
						update_active_mission(DM_KEY_WAYPOINTS_OFFBOARD_1, 0, 0);

					} else {
						update_active_mission(DM_KEY_WAYPOINTS_OFFBOARD_0, 0, 0);
					}

					break;

				case MAV_MISSION_TYPE_FENCE:
					update_geofence_count(0);
					break;

				case MAV_MISSION_TYPE_RALLY:
					update_safepoint_count(0);
					break;

				default:
					PX4_ERR("mission type %u not handled", _mission_type);
					break;
				}

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED);
				_transfer_in_progress = false;
				return;
			}

			PX4_DEBUG("WPM: MISSION_COUNT %u from ID %u, changing state to MAVLINK_WPM_STATE_GETLIST", wpc.count, msg->sysid);

			_state = MAVLINK_WPM_STATE_GETLIST;
			_transfer_seq = 0;
			_transfer_partner_sysid = msg->sysid;
			_transfer_partner_compid = msg->compid;
			_transfer_count = wpc.count;
			_transfer_dataman_id = (_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 ? DM_KEY_WAYPOINTS_OFFBOARD_1 :
						DM_KEY_WAYPOINTS_OFFBOARD_0);	// use inactive storage for transmission
			_transfer_current_seq = -1;

			if (_mission_type == MAV_MISSION_TYPE_FENCE) {
				// We're about to write new geofence items, so take the lock. It will be released when
				// switching back to idle
				PX4_DEBUG("locking fence dataman items");

				int ret = dm_lock(DM_KEY_FENCE_POINTS);

				if (ret == 0) {
					_geofence_locked = true;

				} else {
					PX4_ERR("locking failed (%i)", errno);
				}
			}

		} else if (_state == MAVLINK_WPM_STATE_GETLIST) {
			_time_last_recv = hrt_absolute_time();

			if (_transfer_seq == 0) {
				/* looks like our MISSION_REQUEST was lost, try again */
				PX4_DEBUG("WPM: MISSION_COUNT %u from ID %u (again)", wpc.count, msg->sysid);

			} else {
				PX4_DEBUG("WPM: MISSION_COUNT ERROR: busy, already receiving seq %u", _transfer_seq);

				_mavlink->send_statustext_critical("WPM: REJ. CMD: Busy");

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
				return;
			}

		} else {
			PX4_DEBUG("WPM: MISSION_COUNT ERROR: busy, state %i", _state);

			_mavlink->send_statustext_critical("WPM: IGN MISSION_COUNT: Busy");
			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			return;
		}

		send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
	}
}

void
MavlinkMissionManager::switch_to_idle_state()
{
	// when switching to idle, we *always* check if the lock was held and release it.
	// This is to ensure we don't end up in a state where we forget to release it.
	if (_geofence_locked) {
		dm_unlock(DM_KEY_FENCE_POINTS);
		_geofence_locked = false;

		PX4_DEBUG("unlocking geofence");
	}

	_state = MAVLINK_WPM_STATE_IDLE;
}


void
MavlinkMissionManager::handle_mission_item(const mavlink_message_t *msg)
{
	if (_int_mode) {
		// It seems that we should be using the float mode, let's switch out of int mode.
		_int_mode = false;
	}

	handle_mission_item_both(msg);
}

void
MavlinkMissionManager::handle_mission_item_int(const mavlink_message_t *msg)
{
	if (!_int_mode) {
		// It seems that we should be using the int mode, let's switch to it.
		_int_mode = true;
	}

	handle_mission_item_both(msg);
}

void
MavlinkMissionManager::handle_mission_item_both(const mavlink_message_t *msg)
{

	// The mavlink_message could also contain a mavlink_mission_item_int_t. We ignore that here
	// and take care of it later in parse_mavlink_mission_item depending on _int_mode.

	mavlink_mission_item_t wp;
	mavlink_msg_mission_item_decode(msg, &wp);

	if (CHECK_SYSID_COMPID_MISSION(wp)) {

		if (wp.mission_type != _mission_type) {
			PX4_WARN("WPM: Unexpected mission type (%u %u)", (int)wp.mission_type, (int)_mission_type);
			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			return;
		}

		if (_state == MAVLINK_WPM_STATE_GETLIST) {
			_time_last_recv = hrt_absolute_time();

			if (wp.seq != _transfer_seq) {
				PX4_DEBUG("WPM: MISSION_ITEM ERROR: seq %u was not the expected %u", wp.seq, _transfer_seq);

				/* request next item again */
				send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
				return;
			}

		} else if (_state == MAVLINK_WPM_STATE_IDLE) {
			if (_transfer_seq == wp.seq + 1) {
				// Assume this is a duplicate, where we already successfully got all mission items,
				// but the GCS did not receive the last ack and sent the same item again
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED);

			} else {
				PX4_DEBUG("WPM: MISSION_ITEM ERROR: no transfer");

				_mavlink->send_statustext_critical("IGN MISSION_ITEM: No transfer");
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			}

			return;

		} else {
			PX4_DEBUG("WPM: MISSION_ITEM ERROR: busy, state %i", _state);

			_mavlink->send_statustext_critical("IGN MISSION_ITEM: Busy");
			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			return;
		}

		struct mission_item_s mission_item = {};

		int ret = parse_mavlink_mission_item(&wp, &mission_item);

		if (ret != PX4_OK) {
			PX4_DEBUG("WPM: MISSION_ITEM ERROR: seq %u invalid item", wp.seq);

			_mavlink->send_statustext_critical("IGN MISSION_ITEM: Busy");

			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, ret);
			switch_to_idle_state();
			_transfer_in_progress = false;
			return;
		}

		bool write_failed = false;
		bool check_failed = false;

		switch (_mission_type) {

		case MAV_MISSION_TYPE_MISSION: {
				// check that we don't get a wrong item (hardening against wrong client implementations, the list here
				// does not need to be complete)
				if (mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_RALLY_POINT) {
					check_failed = true;

				} else {
					dm_item_t dm_item = _transfer_dataman_id;

					write_failed = dm_write(dm_item, wp.seq, DM_PERSIST_POWER_ON_RESET, &mission_item,
								sizeof(struct mission_item_s)) != sizeof(struct mission_item_s);

					if (!write_failed) {
						/* waypoint marked as current */
						if (wp.current) {
							_transfer_current_seq = wp.seq;
						}
					}
				}
			}
			break;

		case MAV_MISSION_TYPE_FENCE: { // Write a geofence point
				mission_fence_point_s mission_fence_point;
				mission_fence_point.nav_cmd = mission_item.nav_cmd;
				mission_fence_point.lat = mission_item.lat;
				mission_fence_point.lon = mission_item.lon;
				mission_fence_point.alt = mission_item.altitude;

				if (mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION) {
					mission_fence_point.vertex_count = mission_item.vertex_count;

					if (mission_item.vertex_count < 3) { // feasibility check
						PX4_ERR("Fence: too few vertices");
						check_failed = true;
						update_geofence_count(0);
					}

				} else {
					mission_fence_point.circle_radius = mission_item.circle_radius;
				}

				mission_fence_point.frame = mission_item.frame;

				if (!check_failed) {
					write_failed = dm_write(DM_KEY_FENCE_POINTS, wp.seq + 1, DM_PERSIST_POWER_ON_RESET, &mission_fence_point,
								sizeof(mission_fence_point_s)) != sizeof(mission_fence_point_s);
				}

			}
			break;

		case MAV_MISSION_TYPE_RALLY: { // Write a safe point / rally point
				mission_save_point_s mission_save_point;
				mission_save_point.lat = mission_item.lat;
				mission_save_point.lon = mission_item.lon;
				mission_save_point.alt = mission_item.altitude;
				mission_save_point.frame = mission_item.frame;
				write_failed = dm_write(DM_KEY_SAFE_POINTS, wp.seq + 1, DM_PERSIST_POWER_ON_RESET, &mission_save_point,
							sizeof(mission_save_point_s)) != sizeof(mission_save_point_s);
			}
			break;

		default:
			_mavlink->send_statustext_critical("Received unknown mission type, abort.");
			break;
		}

		if (write_failed || check_failed) {
			PX4_DEBUG("WPM: MISSION_ITEM ERROR: error writing seq %u to dataman ID %i", wp.seq, _transfer_dataman_id);

			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);

			if (write_failed) {
				_mavlink->send_statustext_critical("Unable to write on micro SD");
			}

			switch_to_idle_state();
			_transfer_in_progress = false;
			return;
		}

		/* waypoint marked as current */
		if (wp.current) {
			_transfer_current_seq = wp.seq;
		}

		PX4_DEBUG("WPM: MISSION_ITEM seq %u received", wp.seq);

		_transfer_seq = wp.seq + 1;

		if (_transfer_seq == _transfer_count) {
			/* got all new mission items successfully */
			PX4_DEBUG("WPM: MISSION_ITEM got all %u items, current_seq=%u, changing state to MAVLINK_WPM_STATE_IDLE",
				  _transfer_count, _transfer_current_seq);

			ret = 0;

			switch (_mission_type) {
			case MAV_MISSION_TYPE_MISSION:
				ret = update_active_mission(_transfer_dataman_id, _transfer_count, _transfer_current_seq);
				break;

			case MAV_MISSION_TYPE_FENCE:
				ret = update_geofence_count(_transfer_count);
				break;

			case MAV_MISSION_TYPE_RALLY:
				ret = update_safepoint_count(_transfer_count);
				break;

			default:
				PX4_ERR("mission type %u not handled", _mission_type);
				break;
			}

			// Note: the switch to idle needs to happen after update_geofence_count is called, for proper unlocking order
			switch_to_idle_state();


			if (ret == PX4_OK) {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED);

			} else {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			}

			_transfer_in_progress = false;

		} else {
			/* request next item */
			send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
		}
	}
}


void
MavlinkMissionManager::handle_mission_clear_all(const mavlink_message_t *msg)
{
	mavlink_mission_clear_all_t wpca;
	mavlink_msg_mission_clear_all_decode(msg, &wpca);

	if (CHECK_SYSID_COMPID_MISSION(wpca)) {

		if (_state == MAVLINK_WPM_STATE_IDLE) {
			/* don't touch mission items storage itself, but only items count in mission state */
			_time_last_recv = hrt_absolute_time();

			_mission_type = (MAV_MISSION_TYPE)wpca.mission_type; // this is needed for the returned ack
			int ret = 0;

			switch (wpca.mission_type) {
			case MAV_MISSION_TYPE_MISSION:
				ret = update_active_mission(_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 ? DM_KEY_WAYPOINTS_OFFBOARD_1 :
							    DM_KEY_WAYPOINTS_OFFBOARD_0, 0, 0);
				break;

			case MAV_MISSION_TYPE_FENCE:
				ret = update_geofence_count(0);
				break;

			case MAV_MISSION_TYPE_RALLY:
				ret = update_safepoint_count(0);
				break;

			case MAV_MISSION_TYPE_ALL:
				ret = update_active_mission(_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 ? DM_KEY_WAYPOINTS_OFFBOARD_1 :
							    DM_KEY_WAYPOINTS_OFFBOARD_0, 0, 0);
				ret = update_geofence_count(0) || ret;
				ret = update_safepoint_count(0) || ret;
				break;

			default:
				PX4_ERR("mission type %u not handled", _mission_type);
				break;
			}

			if (ret == PX4_OK) {
				PX4_DEBUG("WPM: CLEAR_ALL OK (mission_type=%i)", _mission_type);

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED);

			} else {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			}

		} else {
			_mavlink->send_statustext_critical("WPM: IGN CLEAR CMD: Busy");

			PX4_DEBUG("WPM: CLEAR_ALL IGNORED: busy");
		}
	}
}

int
MavlinkMissionManager::parse_mavlink_mission_item(const mavlink_mission_item_t *mavlink_mission_item,
		struct mission_item_s *mission_item)
{
	if (mavlink_mission_item->frame == MAV_FRAME_GLOBAL ||
	    mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT ||
	    (_int_mode && (mavlink_mission_item->frame == MAV_FRAME_GLOBAL_INT ||
			   mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT))) {

		// Switch to int mode if that is what we are receiving
		if ((mavlink_mission_item->frame == MAV_FRAME_GLOBAL_INT ||
		     mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)) {
			_int_mode = true;
		}

		if (_int_mode) {
			/* The argument is actually a mavlink_mission_item_int_t in int_mode.
			 * mavlink_mission_item_t and mavlink_mission_item_int_t have the same
			 * alignment, so we can just swap float for int32_t. */
			const mavlink_mission_item_int_t *item_int
				= reinterpret_cast<const mavlink_mission_item_int_t *>(mavlink_mission_item);
			mission_item->lat = ((double)item_int->x) * 1e-7;
			mission_item->lon = ((double)item_int->y) * 1e-7;

		} else {
			mission_item->lat = (double)mavlink_mission_item->x;
			mission_item->lon = (double)mavlink_mission_item->y;
		}

		mission_item->altitude = mavlink_mission_item->z;

		if (mavlink_mission_item->frame == MAV_FRAME_GLOBAL ||
		    mavlink_mission_item->frame == MAV_FRAME_GLOBAL_INT) {
			mission_item->altitude_is_relative = false;

		} else if (mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT ||
			   mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
			mission_item->altitude_is_relative = true;
		}

		/* this field is shared with pitch_min (and circle_radius for geofence) in memory and
		 * exclusive in the MAVLink spec. Set it to 0 first
		 * and then set minimum pitch later only for the
		 * corresponding item
		 */
		mission_item->time_inside = 0.0f;

		switch (mavlink_mission_item->command) {
		case MAV_CMD_NAV_WAYPOINT:
			mission_item->nav_cmd = NAV_CMD_WAYPOINT;
			mission_item->time_inside = mavlink_mission_item->param1;
			mission_item->acceptance_radius = mavlink_mission_item->param2;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_NAV_LOITER_UNLIM:
			mission_item->nav_cmd = NAV_CMD_LOITER_UNLIMITED;
			mission_item->loiter_radius = mavlink_mission_item->param3;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_NAV_LOITER_TIME:
			mission_item->nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
			mission_item->time_inside = mavlink_mission_item->param1;
			mission_item->loiter_radius = mavlink_mission_item->param3;
			mission_item->loiter_exit_xtrack = (mavlink_mission_item->param4 > 0);
			// Yaw is only valid for multicopter but we set it always because
			// it's just ignored for fixedwing.
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_NAV_LAND:
			mission_item->nav_cmd = NAV_CMD_LAND;
			// TODO: abort alt param1
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			mission_item->land_precision = mavlink_mission_item->param2;
			break;

		case MAV_CMD_NAV_TAKEOFF:
			mission_item->nav_cmd = NAV_CMD_TAKEOFF;
			mission_item->pitch_min = mavlink_mission_item->param1;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_NAV_LOITER_TO_ALT:
			mission_item->nav_cmd = NAV_CMD_LOITER_TO_ALT;
			mission_item->force_heading = (mavlink_mission_item->param1 > 0);
			mission_item->loiter_radius = mavlink_mission_item->param2;
			mission_item->loiter_exit_xtrack = (mavlink_mission_item->param4 > 0);
			break;

		case MAV_CMD_NAV_ROI:
		case MAV_CMD_DO_SET_ROI:
			if ((int)mavlink_mission_item->param1 == MAV_ROI_LOCATION) {
				mission_item->nav_cmd = NAV_CMD_DO_SET_ROI;
				mission_item->params[0] = MAV_ROI_LOCATION;

				mission_item->params[6] = mavlink_mission_item->z;

			} else if ((int)mavlink_mission_item->param1 == MAV_ROI_NONE) {
				mission_item->nav_cmd = NAV_CMD_DO_SET_ROI;
				mission_item->params[0] = MAV_ROI_NONE;

			} else {
				return MAV_MISSION_INVALID_PARAM1;
			}

			break;

		case MAV_CMD_DO_SET_ROI_LOCATION:
			mission_item->nav_cmd = NAV_CMD_DO_SET_ROI_LOCATION;
			mission_item->params[6] = mavlink_mission_item->z;
			break;

		case MAV_CMD_NAV_VTOL_TAKEOFF:
		case MAV_CMD_NAV_VTOL_LAND:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_NAV_FENCE_RETURN_POINT:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			break;

		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			mission_item->vertex_count = (uint16_t)(mavlink_mission_item->param1 + 0.5f);
			break;

		case MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
		case MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			mission_item->circle_radius = mavlink_mission_item->param1;
			break;

		case MAV_CMD_NAV_RALLY_POINT:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			break;

		default:
			mission_item->nav_cmd = NAV_CMD_INVALID;

			PX4_DEBUG("Unsupported command %d", mavlink_mission_item->command);

			return MAV_MISSION_UNSUPPORTED;
		}

		mission_item->frame = mavlink_mission_item->frame;

	} else if (mavlink_mission_item->frame == MAV_FRAME_MISSION) {

		// this is a mission item with no coordinates

		mission_item->params[0] = mavlink_mission_item->param1;
		mission_item->params[1] = mavlink_mission_item->param2;
		mission_item->params[2] = mavlink_mission_item->param3;
		mission_item->params[3] = mavlink_mission_item->param4;
		mission_item->params[4] = mavlink_mission_item->x;
		mission_item->params[5] = mavlink_mission_item->y;
		mission_item->params[6] = mavlink_mission_item->z;

		switch (mavlink_mission_item->command) {
		case MAV_CMD_DO_JUMP:
			mission_item->nav_cmd = NAV_CMD_DO_JUMP;
			mission_item->do_jump_mission_index = mavlink_mission_item->param1;
			mission_item->do_jump_current_count = 0;
			mission_item->do_jump_repeat_count = mavlink_mission_item->param2;
			break;

		case MAV_CMD_NAV_ROI:
		case MAV_CMD_DO_SET_ROI: {
				const int roi_mode = mavlink_mission_item->param1;

				if (roi_mode == MAV_ROI_NONE || roi_mode == MAV_ROI_WPNEXT || roi_mode == MAV_ROI_WPINDEX) {
					mission_item->nav_cmd = NAV_CMD_DO_SET_ROI;

				} else {
					return MAV_MISSION_INVALID_PARAM1;
				}
			}
			break;

		case MAV_CMD_DO_CHANGE_SPEED:
		case MAV_CMD_DO_SET_HOME:
		case MAV_CMD_DO_SET_SERVO:
		case MAV_CMD_DO_LAND_START:
		case MAV_CMD_DO_TRIGGER_CONTROL:
		case MAV_CMD_DO_DIGICAM_CONTROL:
		case MAV_CMD_DO_MOUNT_CONFIGURE:
		case MAV_CMD_DO_MOUNT_CONTROL:
		case MAV_CMD_IMAGE_START_CAPTURE:
		case MAV_CMD_IMAGE_STOP_CAPTURE:
		case MAV_CMD_VIDEO_START_CAPTURE:
		case MAV_CMD_VIDEO_STOP_CAPTURE:
		case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
		case MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
		case MAV_CMD_SET_CAMERA_MODE:
		case MAV_CMD_DO_VTOL_TRANSITION:
		case MAV_CMD_NAV_DELAY:
		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
		case MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET:
		case MAV_CMD_DO_SET_ROI_NONE:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			break;

		default:
			mission_item->nav_cmd = NAV_CMD_INVALID;

			PX4_DEBUG("Unsupported command %d", mavlink_mission_item->command);

			return MAV_MISSION_UNSUPPORTED;
		}

		mission_item->frame = MAV_FRAME_MISSION;

	} else {
		PX4_DEBUG("Unsupported frame %d", mavlink_mission_item->frame);

		return MAV_MISSION_UNSUPPORTED_FRAME;
	}

	mission_item->autocontinue = mavlink_mission_item->autocontinue;
	// mission_item->index = mavlink_mission_item->seq;

	mission_item->origin = ORIGIN_MAVLINK;

	return MAV_MISSION_ACCEPTED;
}


int
MavlinkMissionManager::format_mavlink_mission_item(const struct mission_item_s *mission_item,
		mavlink_mission_item_t *mavlink_mission_item)
{
	mavlink_mission_item->frame = mission_item->frame;
	mavlink_mission_item->command = mission_item->nav_cmd;
	mavlink_mission_item->autocontinue = mission_item->autocontinue;

	/* default mappings for generic commands */
	if (mission_item->frame == MAV_FRAME_MISSION) {
		mavlink_mission_item->param1 = mission_item->params[0];
		mavlink_mission_item->param2 = mission_item->params[1];
		mavlink_mission_item->param3 = mission_item->params[2];
		mavlink_mission_item->param4 = mission_item->params[3];
		mavlink_mission_item->x = mission_item->params[4];
		mavlink_mission_item->y = mission_item->params[5];
		mavlink_mission_item->z = mission_item->params[6];

		switch (mavlink_mission_item->command) {
		case NAV_CMD_DO_JUMP:
			mavlink_mission_item->param1 = mission_item->do_jump_mission_index;
			mavlink_mission_item->param2 = mission_item->do_jump_repeat_count;
			break;

		case NAV_CMD_DO_CHANGE_SPEED:
		case NAV_CMD_DO_SET_HOME:
		case NAV_CMD_DO_SET_SERVO:
		case NAV_CMD_DO_LAND_START:
		case NAV_CMD_DO_TRIGGER_CONTROL:
		case NAV_CMD_DO_DIGICAM_CONTROL:
		case NAV_CMD_IMAGE_START_CAPTURE:
		case NAV_CMD_IMAGE_STOP_CAPTURE:
		case NAV_CMD_VIDEO_START_CAPTURE:
		case NAV_CMD_VIDEO_STOP_CAPTURE:
		case NAV_CMD_DO_MOUNT_CONFIGURE:
		case NAV_CMD_DO_MOUNT_CONTROL:
		case NAV_CMD_DO_SET_ROI:
		case NAV_CMD_DO_SET_CAM_TRIGG_DIST:
		case NAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
		case NAV_CMD_SET_CAMERA_MODE:
		case NAV_CMD_DO_VTOL_TRANSITION:
			break;

		default:
			return PX4_ERROR;
		}

	} else {
		mavlink_mission_item->param1 = 0.0f;
		mavlink_mission_item->param2 = 0.0f;
		mavlink_mission_item->param3 = 0.0f;
		mavlink_mission_item->param4 = 0.0f;

		if (_int_mode) {
			// This function actually receives a mavlink_mission_item_int_t in _int_mode
			// which has the same alignment as mavlink_mission_item_t and the only
			// difference is int32_t vs. float for x and y.
			mavlink_mission_item_int_t *item_int =
				reinterpret_cast<mavlink_mission_item_int_t *>(mavlink_mission_item);

			item_int->x = (int32_t)(mission_item->lat * 1e7);
			item_int->y = (int32_t)(mission_item->lon * 1e7);

		} else {
			mavlink_mission_item->x = (float)mission_item->lat;
			mavlink_mission_item->y = (float)mission_item->lon;
		}

		mavlink_mission_item->z = mission_item->altitude;

		if (mission_item->altitude_is_relative) {
			if (_int_mode) {
				mavlink_mission_item->frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;

			} else {
				mavlink_mission_item->frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
			}

		} else {
			if (_int_mode) {
				mavlink_mission_item->frame = MAV_FRAME_GLOBAL_INT;

			} else {
				mavlink_mission_item->frame = MAV_FRAME_GLOBAL;
			}
		}

		switch (mission_item->nav_cmd) {
		case NAV_CMD_WAYPOINT:
			mavlink_mission_item->param1 = mission_item->time_inside;
			mavlink_mission_item->param2 = mission_item->acceptance_radius;
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case NAV_CMD_LOITER_UNLIMITED:
			mavlink_mission_item->param3 = mission_item->loiter_radius;
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case NAV_CMD_LOITER_TIME_LIMIT:
			mavlink_mission_item->param1 = mission_item->time_inside;
			mavlink_mission_item->param3 = mission_item->loiter_radius;
			mavlink_mission_item->param4 = mission_item->loiter_exit_xtrack;
			break;

		case NAV_CMD_LAND:
			// TODO: param1 abort alt
			mavlink_mission_item->param2 = mission_item->land_precision;
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case NAV_CMD_TAKEOFF:
			mavlink_mission_item->param1 = mission_item->pitch_min;
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case NAV_CMD_LOITER_TO_ALT:
			mavlink_mission_item->param1 = mission_item->force_heading;
			mavlink_mission_item->param2 = mission_item->loiter_radius;
			mavlink_mission_item->param4 = mission_item->loiter_exit_xtrack;
			break;

		case MAV_CMD_NAV_VTOL_TAKEOFF:
		case MAV_CMD_NAV_VTOL_LAND:
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case MAV_CMD_NAV_FENCE_RETURN_POINT:
			break;

		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
			mavlink_mission_item->param1 = (float)mission_item->vertex_count;
			break;

		case MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
		case MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
			mavlink_mission_item->param1 = mission_item->circle_radius;
			break;

		case MAV_CMD_NAV_RALLY_POINT:
			break;


		default:
			return PX4_ERROR;
		}
	}

	return PX4_OK;
}


void MavlinkMissionManager::check_active_mission()
{
	// do not send anything over high latency communication
	if (_mavlink->get_mode() == Mavlink::MAVLINK_MODE_IRIDIUM) {
		return;
	}

	if (!(_my_dataman_id == _dataman_id)) {
		PX4_DEBUG("WPM: New mission detected (possibly over different Mavlink instance) Updating");

		_my_dataman_id = _dataman_id;
		send_mission_count(_transfer_partner_sysid, _transfer_partner_compid, _count[MAV_MISSION_TYPE_MISSION],
				   MAV_MISSION_TYPE_MISSION);
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @file mavlink_orb_subscription.cpp
 * uORB subscription implementation.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "mavlink_orb_subscription.h"

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <px4_defines.h>
#include <uORB/uORB.h>

MavlinkOrbSubscription::MavlinkOrbSubscription(const orb_id_t topic, int instance) :
	_topic(topic),
	_instance(instance)
{
}

MavlinkOrbSubscription::~MavlinkOrbSubscription()
{
	if (_fd >= 0) {
		orb_unsubscribe(_fd);
	}
}

orb_id_t
MavlinkOrbSubscription::get_topic() const
{
	return _topic;
}

int
MavlinkOrbSubscription::get_instance() const
{
	return _instance;
}

bool
MavlinkOrbSubscription::update(uint64_t *time, void *data)
{
	// TODO this is NOT atomic operation, we can get data newer than time
	// if topic was published between orb_stat and orb_copy calls.

	if (!is_published()) {
		return false;
	}

	uint64_t time_topic;

	if (orb_stat(_fd, &time_topic)) {
		/* error getting last topic publication time */
		time_topic = 0;
	}

	if (time_topic == 0 || (time_topic != *time)) {
		if (orb_copy(_topic, _fd, data) == PX4_OK) {
			/* data copied successfully */
			*time = time_topic;
			return true;
		}
	}

	return false;
}

bool
MavlinkOrbSubscription::update(void *data)
{
	if (!is_published()) {
		return false;
	}

	if (orb_copy(_topic, _fd, data) != PX4_OK) {
		return false;
	}

	return true;
}

bool
MavlinkOrbSubscription::update_if_changed(void *data)
{
	if (!is_published()) {
		return false;
	}

	bool updated;

	if (orb_check(_fd, &updated) || !updated) {
		return false;
	}

	return update(data);
}

bool
MavlinkOrbSubscription::is_published()
{
	// If we marked it as published no need to check again
	if (_published) {
		return true;
	}

	hrt_abstime now = hrt_absolute_time();

	if (now - _last_pub_check < 300000) {
		return false;
	}

	// We are checking now
	_last_pub_check = now;

	// We don't want to subscribe to anything that does not exist
	// in order to save memory and file descriptors.
	// However, for some topics like vehicle_command_ack, we want to subscribe
	// from the beginning in order not to miss or delay the first publish respective advertise.
	if (!_subscribe_from_beginning && orb_exists(_topic, _instance)) {
		return false;
	}

	if (_fd < 0) {
		_fd = orb_subscribe_multi(_topic, _instance);
	}

	bool updated;
	orb_check(_fd, &updated);

	if (updated) {
		_published = true;
	}

	return _published;
}

void
MavlinkOrbSubscription::subscribe_from_beginning(bool from_beginning)
{
	_subscribe_from_beginning = from_beginning;
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_parameters.cpp
 * Mavlink parameters manager implementation.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Beat Kueng <beat@px4.io>
 */

#include <stdio.h>

#include <uORB/topics/uavcan_parameter_request.h>
#include <uORB/topics/uavcan_parameter_value.h>

#include "mavlink_parameters.h"
#include "mavlink_main.h"

MavlinkParametersManager::MavlinkParametersManager(Mavlink *mavlink) :
	_send_all_index(-1),
	_uavcan_open_request_list(nullptr),
	_uavcan_waiting_for_request_response(false),
	_uavcan_queued_request_items(0),
	_rc_param_map_pub(nullptr),
	_rc_param_map(),
	_uavcan_parameter_request_pub(nullptr),
	_uavcan_parameter_value_sub(-1),
	_mavlink_parameter_sub(-1),
	_param_update_time(0),
	_param_update_index(0),
	_mavlink(mavlink)
{
}
MavlinkParametersManager::~MavlinkParametersManager()
{
	if (_uavcan_parameter_value_sub >= 0) {
		orb_unsubscribe(_uavcan_parameter_value_sub);
	}

	if (_uavcan_parameter_request_pub) {
		orb_unadvertise(_uavcan_parameter_request_pub);
	}
}

unsigned
MavlinkParametersManager::get_size()
{
	return MAVLINK_MSG_ID_PARAM_VALUE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
}

void
MavlinkParametersManager::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
			/* request all parameters */
			mavlink_param_request_list_t req_list;
			mavlink_msg_param_request_list_decode(msg, &req_list);

			if (req_list.target_system == mavlink_system.sysid &&
			    (req_list.target_component == mavlink_system.compid || req_list.target_component == MAV_COMP_ID_ALL)) {
				if (_send_all_index < 0) {
					_send_all_index = PARAM_HASH;

				} else {
					/* a restart should skip the hash check on the ground */
					_send_all_index = 0;
				}
			}

			if (req_list.target_system == mavlink_system.sysid && req_list.target_component < 127 &&
			    (req_list.target_component != mavlink_system.compid || req_list.target_component == MAV_COMP_ID_ALL)) {
				// publish list request to UAVCAN driver via uORB.
				uavcan_parameter_request_s req;
				req.message_type = msg->msgid;
				req.node_id = req_list.target_component;
				req.param_index = 0;

				if (_uavcan_parameter_request_pub == nullptr) {
					_uavcan_parameter_request_pub = orb_advertise(ORB_ID(uavcan_parameter_request), &req);

				} else {
					orb_publish(ORB_ID(uavcan_parameter_request), _uavcan_parameter_request_pub, &req);
				}
			}

			break;
		}

	case MAVLINK_MSG_ID_PARAM_SET: {
			/* set parameter */
			mavlink_param_set_t set;
			mavlink_msg_param_set_decode(msg, &set);

			if (set.target_system == mavlink_system.sysid &&
			    (set.target_component == mavlink_system.compid || set.target_component == MAV_COMP_ID_ALL)) {

				/* local name buffer to enforce null-terminated string */
				char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
				strncpy(name, set.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
				/* enforce null termination */
				name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

				/* Whatever the value is, we're being told to stop sending */
				if (strncmp(name, "_HASH_CHECK", sizeof(name)) == 0) {

					if (_mavlink->hash_check_enabled()) {
						_send_all_index = -1;
					}

					/* No other action taken, return */
					return;
				}

				/* attempt to find parameter, set and send it */
				param_t param = param_find_no_notification(name);

				if (param == PARAM_INVALID) {
					char buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
					sprintf(buf, "[pm] unknown param: %s", name);
					_mavlink->send_statustext_info(buf);

				} else {
					// According to the mavlink spec we should always acknowledge a write operation.
					param_set(param, &(set.param_value));
					send_param(param);
				}
			}

			if (set.target_system == mavlink_system.sysid && set.target_component < 127 &&
			    (set.target_component != mavlink_system.compid || set.target_component == MAV_COMP_ID_ALL)) {
				// publish set request to UAVCAN driver via uORB.
				uavcan_parameter_request_s req;
				req.message_type = msg->msgid;
				req.node_id = set.target_component;
				req.param_index = -1;
				strncpy(req.param_id, set.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
				req.param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

				if (set.param_type == MAV_PARAM_TYPE_REAL32) {
					req.param_type = MAV_PARAM_TYPE_REAL32;
					req.real_value = set.param_value;

				} else {
					int32_t val;
					memcpy(&val, &set.param_value, sizeof(int32_t));
					req.param_type = MAV_PARAM_TYPE_INT64;
					req.int_value = val;
				}

				if (_uavcan_parameter_request_pub == nullptr) {
					_uavcan_parameter_request_pub = orb_advertise(ORB_ID(uavcan_parameter_request), &req);

				} else {
					orb_publish(ORB_ID(uavcan_parameter_request), _uavcan_parameter_request_pub, &req);
				}
			}

			break;
		}

	case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
			/* request one parameter */
			mavlink_param_request_read_t req_read;
			mavlink_msg_param_request_read_decode(msg, &req_read);

			if (req_read.target_system == mavlink_system.sysid &&
			    (req_read.target_component == mavlink_system.compid || req_read.target_component == MAV_COMP_ID_ALL)) {

				/* when no index is given, loop through string ids and compare them */
				if (req_read.param_index < 0) {
					/* XXX: I left this in so older versions of QGC wouldn't break */
					if (strncmp(req_read.param_id, HASH_PARAM, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN) == 0) {
						/* return hash check for cached params */
						uint32_t hash = param_hash_check();

						/* build the one-off response message */
						mavlink_param_value_t param_value;
						param_value.param_count = param_count_used();
						param_value.param_index = -1;
						strncpy(param_value.param_id, HASH_PARAM, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
						param_value.param_type = MAV_PARAM_TYPE_UINT32;
						memcpy(&param_value.param_value, &hash, sizeof(hash));
						mavlink_msg_param_value_send_struct(_mavlink->get_channel(), &param_value);

					} else {
						/* local name buffer to enforce null-terminated string */
						char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
						strncpy(name, req_read.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
						/* enforce null termination */
						name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';
						/* attempt to find parameter and send it */
						send_param(param_find_no_notification(name));
					}

				} else {
					/* when index is >= 0, send this parameter again */
					int ret = send_param(param_for_used_index(req_read.param_index));

					if (ret == 1) {
						char buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
						sprintf(buf, "[pm] unknown param ID: %u", req_read.param_index);
						_mavlink->send_statustext_info(buf);

					} else if (ret == 2) {
						char buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
						sprintf(buf, "[pm] failed loading param from storage ID: %u", req_read.param_index);
						_mavlink->send_statustext_info(buf);
					}
				}
			}

			if (req_read.target_system == mavlink_system.sysid && req_read.target_component < 127 &&
			    (req_read.target_component != mavlink_system.compid || req_read.target_component == MAV_COMP_ID_ALL)) {
				// publish set request to UAVCAN driver via uORB.
				uavcan_parameter_request_s req = {};
				req.timestamp = hrt_absolute_time();
				req.message_type = msg->msgid;
				req.node_id = req_read.target_component;
				req.param_index = req_read.param_index;
				strncpy(req.param_id, req_read.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
				req.param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

				// Enque the request and forward the first to the uavcan node
				enque_uavcan_request(&req);
				request_next_uavcan_parameter();
			}

			break;
		}

	case MAVLINK_MSG_ID_PARAM_MAP_RC: {
			/* map a rc channel to a parameter */
			mavlink_param_map_rc_t map_rc;
			mavlink_msg_param_map_rc_decode(msg, &map_rc);

			if (map_rc.target_system == mavlink_system.sysid &&
			    (map_rc.target_component == mavlink_system.compid ||
			     map_rc.target_component == MAV_COMP_ID_ALL)) {

				/* Copy values from msg to uorb using the parameter_rc_channel_index as index */
				size_t i = map_rc.parameter_rc_channel_index;
				_rc_param_map.param_index[i] = map_rc.param_index;
				strncpy(&(_rc_param_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1)]), map_rc.param_id,
					MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
				/* enforce null termination */
				_rc_param_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1) + rc_parameter_map_s::PARAM_ID_LEN] = '\0';
				_rc_param_map.scale[i] = map_rc.scale;
				_rc_param_map.value0[i] = map_rc.param_value0;
				_rc_param_map.value_min[i] = map_rc.param_value_min;
				_rc_param_map.value_max[i] = map_rc.param_value_max;

				if (map_rc.param_index == -2) { // -2 means unset map
					_rc_param_map.valid[i] = false;

				} else {
					_rc_param_map.valid[i] = true;
				}

				_rc_param_map.timestamp = hrt_absolute_time();

				if (_rc_param_map_pub == nullptr) {
					_rc_param_map_pub = orb_advertise(ORB_ID(rc_parameter_map), &_rc_param_map);

				} else {
					orb_publish(ORB_ID(rc_parameter_map), _rc_param_map_pub, &_rc_param_map);
				}

			}

			break;
		}

	default:
		break;
	}
}

void
MavlinkParametersManager::send(const hrt_abstime t)
{
	int max_num_to_send;

	if (_mavlink->get_protocol() == SERIAL && !_mavlink->is_usb_uart()) {
		max_num_to_send = 3;

	} else {
		// speed up parameter loading via UDP, TCP or USB: try to send 20 at once
		max_num_to_send = 20;
	}

	int i = 0;

	// Send while burst is not exceeded, we still have buffer space and still something to send
	while ((i++ < max_num_to_send) && (_mavlink->get_free_tx_buf() >= get_size()) && send_params());
}

bool
MavlinkParametersManager::send_params()
{
	if (send_uavcan()) {
		return true;

	} else if (send_one()) {
		return true;

	} else if (send_untransmitted()) {
		return true;

	} else {
		return false;
	}
}

bool
MavlinkParametersManager::send_untransmitted()
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
MavlinkParametersManager::send_uavcan()
{
	/* Send parameter values received from the UAVCAN topic */
	if (_uavcan_parameter_value_sub < 0) {
		_uavcan_parameter_value_sub = orb_subscribe(ORB_ID(uavcan_parameter_value));
	}

	bool param_value_ready;
	orb_check(_uavcan_parameter_value_sub, &param_value_ready);

	if (param_value_ready) {
		struct uavcan_parameter_value_s value;
		orb_copy(ORB_ID(uavcan_parameter_value), _uavcan_parameter_value_sub, &value);

		// Check if we received a matching parameter, drop it from the list and request the next
		if (_uavcan_open_request_list != nullptr
		    && value.param_index == _uavcan_open_request_list->req.param_index
		    && value.node_id == _uavcan_open_request_list->req.node_id) {
			dequeue_uavcan_request();
			request_next_uavcan_parameter();
		}

		mavlink_param_value_t msg;
		msg.param_count = value.param_count;
		msg.param_index = value.param_index;
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
		strncpy(msg.param_id, value.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
#if defined(__GNUC__) && __GNUC__ >= 8
#pragma GCC diagnostic pop
#endif

		if (value.param_type == MAV_PARAM_TYPE_REAL32) {
			msg.param_type = MAVLINK_TYPE_FLOAT;
			msg.param_value = value.real_value;

		} else {
			int32_t val;
			val = (int32_t)value.int_value;
			memcpy(&msg.param_value, &val, sizeof(int32_t));
			msg.param_type = MAVLINK_TYPE_INT32_T;
		}

		// Re-pack the message with the UAVCAN node ID
		mavlink_message_t mavlink_packet;
		mavlink_msg_param_value_encode_chan(mavlink_system.sysid, value.node_id, _mavlink->get_channel(), &mavlink_packet,
						    &msg);
		_mavlink_resend_uart(_mavlink->get_channel(), &mavlink_packet);

		return true;
	}

	return false;
}

bool
MavlinkParametersManager::send_one()
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
MavlinkParametersManager::send_param(param_t param, int component_id)
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

void MavlinkParametersManager::request_next_uavcan_parameter()
{
	// Request a parameter if we are not already waiting on a response and if the list is not empty
	if (!_uavcan_waiting_for_request_response && _uavcan_open_request_list != nullptr) {
		uavcan_parameter_request_s req = _uavcan_open_request_list->req;

		if (_uavcan_parameter_request_pub == nullptr) {
			_uavcan_parameter_request_pub = orb_advertise_queue(ORB_ID(uavcan_parameter_request), &req, 5);

		} else {
			orb_publish(ORB_ID(uavcan_parameter_request), _uavcan_parameter_request_pub, &req);
		}

		_uavcan_waiting_for_request_response = true;
	}
}

void MavlinkParametersManager::enque_uavcan_request(uavcan_parameter_request_s *req)
{
	// We store at max 10 requests to keep memory consumption low.
	// Dropped requests will be repeated by the ground station
	if (_uavcan_queued_request_items >= 10) {
		return;
	}

	_uavcan_open_request_list_item *new_reqest = new _uavcan_open_request_list_item;
	new_reqest->req = *req;
	new_reqest->next = nullptr;

	_uavcan_open_request_list_item *item = _uavcan_open_request_list;
	++_uavcan_queued_request_items;

	if (item == nullptr) {
		// Add the first item to the list
		_uavcan_open_request_list = new_reqest;

	} else {
		// Find the last item and add the new request at the end
		while (item->next != nullptr) {
			item = item->next;
		}

		item->next = new_reqest;
	}
}

void MavlinkParametersManager::dequeue_uavcan_request()
{
	if (_uavcan_open_request_list != nullptr) {
		// Drop the first item in the list and free the used memory
		_uavcan_open_request_list_item *first = _uavcan_open_request_list;
		_uavcan_open_request_list = first->next;
		--_uavcan_queued_request_items;
		delete first;
		_uavcan_waiting_for_request_response = false;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file mavlink_rate_limiter.cpp
 * Message rate limiter implementation.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "mavlink_rate_limiter.h"

bool
MavlinkRateLimiter::check(const hrt_abstime &t)
{
	uint64_t dt = t - _last_sent;

	if (dt > 0 && dt >= _interval) {
		_last_sent = t;
		return true;
	}

	return false;
}
/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_receiver.cpp
 * MAVLink protocol message receive and dispatch
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

/* XXX trim includes */
#include <px4_config.h>
#include <px4_time.h>
#include <px4_tasks.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_range_finder.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_tone_alarm.h>
#include <time.h>
#include <float.h>
#include <unistd.h>
#ifndef __PX4_POSIX
#include <termios.h>
#endif

#ifdef CONFIG_NET
#include <net/if.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#endif

#include <errno.h>
#include <stdlib.h>
#include <poll.h>

#include <sys/stat.h>
#ifdef __PX4_DARWIN
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif

#include <airspeed/airspeed.h>
#include <ecl/geo/geo.h>
#include <mathlib/mathlib.h>
#include <conversion/rotation.h>
#include <parameters/param.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>

#include <commander/px4_custom_mode.h>

#include <uORB/topics/radio_status.h>
#include <uORB/topics/vehicle_command_ack.h>

#include "mavlink_bridge_header.h"
#include "mavlink_receiver.h"
#include "mavlink_main.h"
#include "mavlink_command_sender.h"

#ifdef CONFIG_NET
#define MAVLINK_RECEIVER_NET_ADDED_STACK 1360
#else
#define MAVLINK_RECEIVER_NET_ADDED_STACK 0
#endif

using matrix::wrap_2pi;

MavlinkReceiver::MavlinkReceiver(Mavlink *parent) :
	_mavlink(parent),
	_mission_manager(parent),
	_parameters_manager(parent),
	_mavlink_ftp(parent),
	_mavlink_log_handler(parent),
	_mavlink_timesync(parent),
	_status{},
	_hil_local_pos{},
	_hil_land_detector{},
	_control_mode{},
	_global_pos_pub(nullptr),
	_local_pos_pub(nullptr),
	_attitude_pub(nullptr),
	_gps_pub(nullptr),
	_gyro_pub(nullptr),
	_accel_pub(nullptr),
	_mag_pub(nullptr),
	_baro_pub(nullptr),
	_airspeed_pub(nullptr),
	_battery_pub(nullptr),
	_cmd_pub(nullptr),
	_flow_pub(nullptr),
	_hil_distance_sensor_pub(nullptr),
	_flow_distance_sensor_pub(nullptr),
	_distance_sensor_pub(nullptr),
	_offboard_control_mode_pub(nullptr),
	_actuator_controls_pubs{nullptr, nullptr, nullptr, nullptr},
	_att_sp_pub(nullptr),
	_rates_sp_pub(nullptr),
	_pos_sp_triplet_pub(nullptr),
	_mocap_odometry_pub(nullptr),
	_visual_odometry_pub(nullptr),
	_radio_status_pub(nullptr),
	_ping_pub(nullptr),
	_rc_pub(nullptr),
	_manual_pub(nullptr),
	_obstacle_distance_pub(nullptr),
	_trajectory_waypoint_pub(nullptr),
	_land_detector_pub(nullptr),
	_follow_target_pub(nullptr),
	_landing_target_pose_pub(nullptr),
	_transponder_report_pub(nullptr),
	_collision_report_pub(nullptr),
	_debug_key_value_pub(nullptr),
	_debug_value_pub(nullptr),
	_debug_vect_pub(nullptr),
	_debug_array_pub(nullptr),
	_gps_inject_data_pub(nullptr),
	_command_ack_pub(nullptr),
	_control_mode_sub(orb_subscribe(ORB_ID(vehicle_control_mode))),
	_actuator_armed_sub(orb_subscribe(ORB_ID(actuator_armed))),
	_vehicle_attitude_sub(orb_subscribe(ORB_ID(vehicle_attitude))),
	_global_ref_timestamp(0),
	_hil_frames(0),
	_old_timestamp(0),
	_hil_local_proj_inited(0),
	_hil_local_alt0(0.0f),
	_hil_local_proj_ref{},
	_offboard_control_mode{},
	_orb_class_instance(-1),
	_mom_switch_pos{},
	_mom_switch_state(0),
	_p_bat_emergen_thr(param_find("BAT_EMERGEN_THR")),
	_p_bat_crit_thr(param_find("BAT_CRIT_THR")),
	_p_bat_low_thr(param_find("BAT_LOW_THR")),
	_p_flow_rot(param_find("SENS_FLOW_ROT")),
	_p_flow_maxr(param_find("SENS_FLOW_MAXR")),
	_p_flow_minhgt(param_find("SENS_FLOW_MINHGT")),
	_p_flow_maxhgt(param_find("SENS_FLOW_MAXHGT"))
{
	/* Make the attitude quaternion valid */
	_att.q[0] = 1.0f;
}

MavlinkReceiver::~MavlinkReceiver()
{
	orb_unsubscribe(_control_mode_sub);
	orb_unsubscribe(_actuator_armed_sub);
	orb_unsubscribe(_vehicle_attitude_sub);
}

void MavlinkReceiver::acknowledge(uint8_t sysid, uint8_t compid, uint16_t command, uint8_t result)
{
	vehicle_command_ack_s command_ack = {};
	command_ack.timestamp = hrt_absolute_time();
	command_ack.command = command;
	command_ack.result = result;
	command_ack.target_system = sysid;
	command_ack.target_component = compid;

	if (_command_ack_pub == nullptr) {
		_command_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &command_ack,
						       vehicle_command_ack_s::ORB_QUEUE_LENGTH);

	} else {
		orb_publish(ORB_ID(vehicle_command_ack), _command_ack_pub, &command_ack);
	}
}

void
MavlinkReceiver::handle_message(mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_COMMAND_LONG:
		handle_message_command_long(msg);
		break;

	case MAVLINK_MSG_ID_COMMAND_INT:
		handle_message_command_int(msg);
		break;

	case MAVLINK_MSG_ID_COMMAND_ACK:
		handle_message_command_ack(msg);
		break;

	case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
		handle_message_optical_flow_rad(msg);
		break;

	case MAVLINK_MSG_ID_PING:
		handle_message_ping(msg);
		break;

	case MAVLINK_MSG_ID_SET_MODE:
		handle_message_set_mode(msg);
		break;

	case MAVLINK_MSG_ID_ATT_POS_MOCAP:
		handle_message_att_pos_mocap(msg);
		break;

	case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
		handle_message_set_position_target_local_ned(msg);
		break;

	case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
		handle_message_set_attitude_target(msg);
		break;

	case MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET:
		handle_message_set_actuator_control_target(msg);
		break;

	case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
		handle_message_vision_position_estimate(msg);
		break;

	case MAVLINK_MSG_ID_ODOMETRY:
		handle_message_odometry(msg);
		break;

	case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
		handle_message_gps_global_origin(msg);
		break;

	case MAVLINK_MSG_ID_RADIO_STATUS:
		handle_message_radio_status(msg);
		break;

	case MAVLINK_MSG_ID_MANUAL_CONTROL:
		handle_message_manual_control(msg);
		break;

	case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
		handle_message_rc_channels_override(msg);
		break;

	case MAVLINK_MSG_ID_HEARTBEAT:
		handle_message_heartbeat(msg);
		break;

	case MAVLINK_MSG_ID_DISTANCE_SENSOR:
		handle_message_distance_sensor(msg);
		break;

	case MAVLINK_MSG_ID_FOLLOW_TARGET:
		handle_message_follow_target(msg);
		break;

	case MAVLINK_MSG_ID_LANDING_TARGET:
		handle_message_landing_target(msg);
		break;

	case MAVLINK_MSG_ID_ADSB_VEHICLE:
		handle_message_adsb_vehicle(msg);
		break;

	case MAVLINK_MSG_ID_COLLISION:
		handle_message_collision(msg);
		break;

	case MAVLINK_MSG_ID_GPS_RTCM_DATA:
		handle_message_gps_rtcm_data(msg);
		break;

	case MAVLINK_MSG_ID_BATTERY_STATUS:
		handle_message_battery_status(msg);
		break;

	case MAVLINK_MSG_ID_SERIAL_CONTROL:
		handle_message_serial_control(msg);
		break;

	case MAVLINK_MSG_ID_LOGGING_ACK:
		handle_message_logging_ack(msg);
		break;

	case MAVLINK_MSG_ID_PLAY_TUNE:
		handle_message_play_tune(msg);
		break;

	case MAVLINK_MSG_ID_OBSTACLE_DISTANCE:
		handle_message_obstacle_distance(msg);
		break;

	case MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS:
		handle_message_trajectory_representation_waypoints(msg);
		break;

	case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
		handle_message_named_value_float(msg);
		break;

	case MAVLINK_MSG_ID_DEBUG:
		handle_message_debug(msg);
		break;

	case MAVLINK_MSG_ID_DEBUG_VECT:
		handle_message_debug_vect(msg);
		break;

	case MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY:
		handle_message_debug_float_array(msg);
		break;

	default:
		break;
	}

	/*
	 * Only decode hil messages in HIL mode.
	 *
	 * The HIL mode is enabled by the HIL bit flag
	 * in the system mode. Either send a set mode
	 * COMMAND_LONG message or a SET_MODE message
	 *
	 * Accept HIL GPS messages if use_hil_gps flag is true.
	 * This allows to provide fake gps measurements to the system.
	 */
	if (_mavlink->get_hil_enabled()) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_HIL_SENSOR:
			handle_message_hil_sensor(msg);
			break;

		case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
			handle_message_hil_state_quaternion(msg);
			break;

		case MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
			handle_message_hil_optical_flow(msg);
			break;

		default:
			break;
		}
	}


	if (_mavlink->get_hil_enabled() || (_mavlink->get_use_hil_gps() && msg->sysid == mavlink_system.sysid)) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_HIL_GPS:
			handle_message_hil_gps(msg);
			break;

		default:
			break;
		}

	}

	/* If we've received a valid message, mark the flag indicating so.
	   This is used in the '-w' command-line flag. */
	_mavlink->set_has_received_messages(true);
}

bool
MavlinkReceiver::evaluate_target_ok(int command, int target_system, int target_component)
{
	/* evaluate if this system should accept this command */
	bool target_ok = false;

	switch (command) {

	case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
	case MAV_CMD_REQUEST_PROTOCOL_VERSION:
		/* broadcast and ignore component */
		target_ok = (target_system == 0) || (target_system == mavlink_system.sysid);
		break;

	default:
		target_ok = (target_system == mavlink_system.sysid) && ((target_component == mavlink_system.compid)
				|| (target_component == MAV_COMP_ID_ALL));
		break;
	}

	return target_ok;
}

void
MavlinkReceiver::send_flight_information()
{
	mavlink_flight_information_t flight_info{};

	param_t param_flight_uuid = param_find("COM_FLIGHT_UUID");

	if (param_flight_uuid != PARAM_INVALID) {
		int32_t flight_uuid;
		param_get(param_flight_uuid, &flight_uuid);
		flight_info.flight_uuid = (uint64_t)flight_uuid;
	}

	actuator_armed_s actuator_armed;
	int ret = orb_copy(ORB_ID(actuator_armed), _actuator_armed_sub, &actuator_armed);

	if (ret == 0 && actuator_armed.timestamp != 0) {
		flight_info.arming_time_utc = flight_info.takeoff_time_utc = actuator_armed.armed_time_ms;
	}

	flight_info.time_boot_ms = hrt_absolute_time() / 1000;
	mavlink_msg_flight_information_send_struct(_mavlink->get_channel(), &flight_info);
}

void
MavlinkReceiver::send_storage_information(int storage_id)
{
	mavlink_storage_information_t storage_info{};
	const char *microsd_dir = PX4_STORAGEDIR;

	if (storage_id == 0 || storage_id == 1) { // request is for all or the first storage
		storage_info.storage_id = 1;

		struct statfs statfs_buf;
		uint64_t total_bytes = 0;
		uint64_t avail_bytes = 0;

		if (statfs(microsd_dir, &statfs_buf) == 0) {
			total_bytes = (uint64_t)statfs_buf.f_blocks * statfs_buf.f_bsize;
			avail_bytes = (uint64_t)statfs_buf.f_bavail * statfs_buf.f_bsize;
		}

		if (total_bytes == 0) { // on NuttX we get 0 total bytes if no SD card is inserted
			storage_info.storage_count = 0;
			storage_info.status = 0; // not available

		} else {
			storage_info.storage_count = 1;
			storage_info.status = 2; // available & formatted
			storage_info.total_capacity = total_bytes / 1024. / 1024.;
			storage_info.available_capacity = avail_bytes / 1024. / 1024.;
			storage_info.used_capacity = (total_bytes - avail_bytes) / 1024. / 1024.;
		}

	} else {
		// only one storage supported
		storage_info.storage_id = storage_id;
		storage_info.storage_count = 1;
	}

	storage_info.time_boot_ms = hrt_absolute_time() / 1000;
	mavlink_msg_storage_information_send_struct(_mavlink->get_channel(), &storage_info);
}

void
MavlinkReceiver::handle_message_command_long(mavlink_message_t *msg)
{
	/* command */
	mavlink_command_long_t cmd_mavlink;
	mavlink_msg_command_long_decode(msg, &cmd_mavlink);

	vehicle_command_s vcmd = {};
	vcmd.timestamp = hrt_absolute_time();

	/* Copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
	vcmd.param1 = cmd_mavlink.param1;
	vcmd.param2 = cmd_mavlink.param2;
	vcmd.param3 = cmd_mavlink.param3;
	vcmd.param4 = cmd_mavlink.param4;
	vcmd.param5 = (double)cmd_mavlink.param5;
	vcmd.param6 = (double)cmd_mavlink.param6;
	vcmd.param7 = cmd_mavlink.param7;
	vcmd.command = cmd_mavlink.command;
	vcmd.target_system = cmd_mavlink.target_system;
	vcmd.target_component = cmd_mavlink.target_component;
	vcmd.source_system = msg->sysid;
	vcmd.source_component = msg->compid;
	vcmd.confirmation = cmd_mavlink.confirmation;
	vcmd.from_external = true;

	handle_message_command_both(msg, cmd_mavlink, vcmd);
}

void
MavlinkReceiver::handle_message_command_int(mavlink_message_t *msg)
{
	/* command */
	mavlink_command_int_t cmd_mavlink;
	mavlink_msg_command_int_decode(msg, &cmd_mavlink);

	vehicle_command_s vcmd = {};
	vcmd.timestamp = hrt_absolute_time();

	/* Copy the content of mavlink_command_int_t cmd_mavlink into command_t cmd */
	vcmd.param1 = cmd_mavlink.param1;
	vcmd.param2 = cmd_mavlink.param2;
	vcmd.param3 = cmd_mavlink.param3;
	vcmd.param4 = cmd_mavlink.param4;
	vcmd.param5 = ((double)cmd_mavlink.x) / 1e7;
	vcmd.param6 = ((double)cmd_mavlink.y) / 1e7;
	vcmd.param7 = cmd_mavlink.z;
	vcmd.command = cmd_mavlink.command;
	vcmd.target_system = cmd_mavlink.target_system;
	vcmd.target_component = cmd_mavlink.target_component;
	vcmd.source_system = msg->sysid;
	vcmd.source_component = msg->compid;
	vcmd.confirmation = false;
	vcmd.from_external = true;

	handle_message_command_both(msg, cmd_mavlink, vcmd);
}

template <class T>
void MavlinkReceiver::handle_message_command_both(mavlink_message_t *msg, const T &cmd_mavlink,
		const vehicle_command_s &vehicle_command)
{
	bool target_ok = evaluate_target_ok(cmd_mavlink.command, cmd_mavlink.target_system, cmd_mavlink.target_component);

	bool send_ack = true;
	uint8_t result = vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED;

	if (!target_ok) {
		acknowledge(msg->sysid, msg->compid, cmd_mavlink.command, vehicle_command_ack_s::VEHICLE_RESULT_FAILED);
		return;
	}

	if (cmd_mavlink.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) {
		/* send autopilot version message */
		_mavlink->send_autopilot_capabilites();

	} else if (cmd_mavlink.command == MAV_CMD_REQUEST_PROTOCOL_VERSION) {
		/* send protocol version message */
		_mavlink->send_protocol_version();

	} else if (cmd_mavlink.command == MAV_CMD_GET_HOME_POSITION) {
		_mavlink->configure_stream_threadsafe("HOME_POSITION", 0.5f);

	} else if (cmd_mavlink.command == MAV_CMD_SET_MESSAGE_INTERVAL) {
		if (set_message_interval((int)(cmd_mavlink.param1 + 0.5f), cmd_mavlink.param2, cmd_mavlink.param3)) {
			result = vehicle_command_ack_s::VEHICLE_RESULT_FAILED;
		}

	} else if (cmd_mavlink.command == MAV_CMD_GET_MESSAGE_INTERVAL) {
		get_message_interval((int)cmd_mavlink.param1);

	} else if (cmd_mavlink.command == MAV_CMD_REQUEST_FLIGHT_INFORMATION) {
		send_flight_information();

	} else if (cmd_mavlink.command == MAV_CMD_REQUEST_STORAGE_INFORMATION) {
		if ((int)(cmd_mavlink.param2 + 0.5f) == 1) {
			send_storage_information(cmd_mavlink.param1 + 0.5f);
		}

	} else {

		send_ack = false;

		if (msg->sysid == mavlink_system.sysid && msg->compid == mavlink_system.compid) {
			PX4_WARN("ignoring CMD with same SYS/COMP (%d/%d) ID", mavlink_system.sysid, mavlink_system.compid);
			return;
		}

		if (cmd_mavlink.command == MAV_CMD_LOGGING_START) {
			// check that we have enough bandwidth available: this is given by the configured logger topics
			// and rates. The 5000 is somewhat arbitrary, but makes sure that we cannot enable log streaming
			// on a radio link
			if (_mavlink->get_data_rate() < 5000) {
				send_ack = true;
				result = vehicle_command_ack_s::VEHICLE_RESULT_DENIED;
				_mavlink->send_statustext_critical("Not enough bandwidth to enable log streaming");

			} else {
				// we already instanciate the streaming object, because at this point we know on which
				// mavlink channel streaming was requested. But in fact it's possible that the logger is
				// not even running. The main mavlink thread takes care of this by waiting for an ack
				// from the logger.
				_mavlink->try_start_ulog_streaming(msg->sysid, msg->compid);
			}

		} else if (cmd_mavlink.command == MAV_CMD_LOGGING_STOP) {
			_mavlink->request_stop_ulog_streaming();
		}

		if (!send_ack) {
			if (_cmd_pub == nullptr) {
				_cmd_pub = orb_advertise_queue(ORB_ID(vehicle_command), &vehicle_command, vehicle_command_s::ORB_QUEUE_LENGTH);

			} else {
				orb_publish(ORB_ID(vehicle_command), _cmd_pub, &vehicle_command);
			}
		}
	}

	if (send_ack) {
		acknowledge(msg->sysid, msg->compid, cmd_mavlink.command, result);
	}
}

void
MavlinkReceiver::handle_message_command_ack(mavlink_message_t *msg)
{
	mavlink_command_ack_t ack;
	mavlink_msg_command_ack_decode(msg, &ack);

	MavlinkCommandSender::instance().handle_mavlink_command_ack(ack, msg->sysid, msg->compid);

	vehicle_command_ack_s command_ack = {};
	command_ack.timestamp = hrt_absolute_time();
	command_ack.result_param2 = ack.result_param2;
	command_ack.command = ack.command;
	command_ack.result = ack.result;
	command_ack.from_external = true;
	command_ack.result_param1 = ack.progress;
	command_ack.target_system = ack.target_system;
	command_ack.target_component = ack.target_component;

	if (_command_ack_pub == nullptr) {
		_command_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &command_ack,
						       vehicle_command_ack_s::ORB_QUEUE_LENGTH);

	} else {
		orb_publish(ORB_ID(vehicle_command_ack), _command_ack_pub, &command_ack);
	}

	// TODO: move it to the same place that sent the command
	if (ack.result != MAV_RESULT_ACCEPTED && ack.result != MAV_RESULT_IN_PROGRESS) {
		if (msg->compid == MAV_COMP_ID_CAMERA) {
			PX4_WARN("Got unsuccessful result %d from camera", ack.result);
		}
	}
}

void
MavlinkReceiver::handle_message_optical_flow_rad(mavlink_message_t *msg)
{
	/* optical flow */
	mavlink_optical_flow_rad_t flow;
	mavlink_msg_optical_flow_rad_decode(msg, &flow);

	/* read flow sensor parameters */
	int32_t flow_rot_int;
	param_get(_p_flow_rot, &flow_rot_int);
	const enum Rotation flow_rot = (Rotation)flow_rot_int;

	struct optical_flow_s f = {};

	f.timestamp = _mavlink_timesync.sync_stamp(flow.time_usec);
	f.integration_timespan = flow.integration_time_us;
	f.pixel_flow_x_integral = flow.integrated_x;
	f.pixel_flow_y_integral = flow.integrated_y;
	f.gyro_x_rate_integral = flow.integrated_xgyro;
	f.gyro_y_rate_integral = flow.integrated_ygyro;
	f.gyro_z_rate_integral = flow.integrated_zgyro;
	f.time_since_last_sonar_update = flow.time_delta_distance_us;
	f.ground_distance_m = flow.distance;
	f.quality = flow.quality;
	f.sensor_id = flow.sensor_id;
	f.gyro_temperature = flow.temperature;
	param_get(_p_flow_maxr, &f.max_flow_rate);
	param_get(_p_flow_minhgt, &f.min_ground_distance);
	param_get(_p_flow_maxhgt, &f.max_ground_distance);

	/* rotate measurements according to parameter */
	float zeroval = 0.0f;
	rotate_3f(flow_rot, f.pixel_flow_x_integral, f.pixel_flow_y_integral, zeroval);
	rotate_3f(flow_rot, f.gyro_x_rate_integral, f.gyro_y_rate_integral, f.gyro_z_rate_integral);

	if (_flow_pub == nullptr) {
		_flow_pub = orb_advertise(ORB_ID(optical_flow), &f);

	} else {
		orb_publish(ORB_ID(optical_flow), _flow_pub, &f);
	}

	/* Use distance value for distance sensor topic */
	struct distance_sensor_s d = {};

	if (flow.distance > 0.0f) { // negative values signal invalid data
		d.timestamp = _mavlink_timesync.sync_stamp(flow.integration_time_us * 1000); /* ms to us */
		d.min_distance = 0.3f;
		d.max_distance = 5.0f;
		d.current_distance = flow.distance; /* both are in m */
		d.type = 1;
		d.id = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
		d.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
		d.covariance = 0.0;

		if (_flow_distance_sensor_pub == nullptr) {
			_flow_distance_sensor_pub = orb_advertise_multi(ORB_ID(distance_sensor), &d,
						    &_orb_class_instance, ORB_PRIO_HIGH);

		} else {
			orb_publish(ORB_ID(distance_sensor), _flow_distance_sensor_pub, &d);
		}
	}
}

void
MavlinkReceiver::handle_message_hil_optical_flow(mavlink_message_t *msg)
{
	/* optical flow */
	mavlink_hil_optical_flow_t flow;
	mavlink_msg_hil_optical_flow_decode(msg, &flow);

	struct optical_flow_s f;
	memset(&f, 0, sizeof(f));

	f.timestamp = hrt_absolute_time(); // XXX we rely on the system time for now and not flow.time_usec;
	f.integration_timespan = flow.integration_time_us;
	f.pixel_flow_x_integral = flow.integrated_x;
	f.pixel_flow_y_integral = flow.integrated_y;
	f.gyro_x_rate_integral = flow.integrated_xgyro;
	f.gyro_y_rate_integral = flow.integrated_ygyro;
	f.gyro_z_rate_integral = flow.integrated_zgyro;
	f.time_since_last_sonar_update = flow.time_delta_distance_us;
	f.ground_distance_m = flow.distance;
	f.quality = flow.quality;
	f.sensor_id = flow.sensor_id;
	f.gyro_temperature = flow.temperature;

	if (_flow_pub == nullptr) {
		_flow_pub = orb_advertise(ORB_ID(optical_flow), &f);

	} else {
		orb_publish(ORB_ID(optical_flow), _flow_pub, &f);
	}

	/* Use distance value for distance sensor topic */
	struct distance_sensor_s d;
	memset(&d, 0, sizeof(d));

	d.timestamp = hrt_absolute_time();
	d.min_distance = 0.3f;
	d.max_distance = 5.0f;
	d.current_distance = flow.distance; /* both are in m */
	d.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	d.id = 0;
	d.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	d.covariance = 0.0;

	if (_hil_distance_sensor_pub == nullptr) {
		_hil_distance_sensor_pub = orb_advertise_multi(ORB_ID(distance_sensor), &d,
					   &_orb_class_instance, ORB_PRIO_HIGH);

	} else {
		orb_publish(ORB_ID(distance_sensor), _hil_distance_sensor_pub, &d);
	}
}

void
MavlinkReceiver::handle_message_set_mode(mavlink_message_t *msg)
{
	mavlink_set_mode_t new_mode;
	mavlink_msg_set_mode_decode(msg, &new_mode);

	union px4_custom_mode custom_mode;
	custom_mode.data = new_mode.custom_mode;

	vehicle_command_s vcmd = {};
	vcmd.timestamp = hrt_absolute_time();

	/* copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
	vcmd.param1 = (float)new_mode.base_mode;
	vcmd.param2 = (float)custom_mode.main_mode;
	vcmd.param3 = (float)custom_mode.sub_mode;

	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	vcmd.target_system = new_mode.target_system;
	vcmd.target_component = MAV_COMP_ID_ALL;
	vcmd.source_system = msg->sysid;
	vcmd.source_component = msg->compid;
	vcmd.confirmation = true;
	vcmd.from_external = true;

	if (_cmd_pub == nullptr) {
		_cmd_pub = orb_advertise_queue(ORB_ID(vehicle_command), &vcmd, vehicle_command_s::ORB_QUEUE_LENGTH);

	} else {
		orb_publish(ORB_ID(vehicle_command), _cmd_pub, &vcmd);
	}
}

void
MavlinkReceiver::handle_message_distance_sensor(mavlink_message_t *msg)
{
	/* distance sensor */
	mavlink_distance_sensor_t dist_sensor;
	mavlink_msg_distance_sensor_decode(msg, &dist_sensor);

	struct distance_sensor_s d;
	memset(&d, 0, sizeof(d));

	d.timestamp = dist_sensor.time_boot_ms * 1000; /* ms to us */
	d.min_distance = float(dist_sensor.min_distance) * 1e-2f; /* cm to m */
	d.max_distance = float(dist_sensor.max_distance) * 1e-2f; /* cm to m */
	d.current_distance = float(dist_sensor.current_distance) * 1e-2f; /* cm to m */
	d.type = dist_sensor.type;
	d.id = 	MAV_DISTANCE_SENSOR_LASER;
	d.orientation = dist_sensor.orientation;
	d.covariance = dist_sensor.covariance;

	/// TODO Add sensor rotation according to MAV_SENSOR_ORIENTATION enum

	if (_distance_sensor_pub == nullptr) {
		_distance_sensor_pub = orb_advertise_multi(ORB_ID(distance_sensor), &d,
				       &_orb_class_instance, ORB_PRIO_HIGH);

	} else {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_pub, &d);
	}
}

void
MavlinkReceiver::handle_message_att_pos_mocap(mavlink_message_t *msg)
{
	mavlink_att_pos_mocap_t mocap;
	mavlink_msg_att_pos_mocap_decode(msg, &mocap);

	struct vehicle_odometry_s mocap_odom = {};

	mocap_odom.timestamp = _mavlink_timesync.sync_stamp(mocap.time_usec);
	mocap_odom.x = mocap.x;
	mocap_odom.y = mocap.y;
	mocap_odom.z = mocap.z;
	mocap_odom.q[0] = mocap.q[0];
	mocap_odom.q[1] = mocap.q[1];
	mocap_odom.q[2] = mocap.q[2];
	mocap_odom.q[3] = mocap.q[3];

	const size_t URT_SIZE = sizeof(mocap_odom.pose_covariance) / sizeof(mocap_odom.pose_covariance[0]);
	static_assert(URT_SIZE == (sizeof(mocap.covariance) / sizeof(mocap.covariance[0])),
		      "Odometry Pose Covariance matrix URT array size mismatch");

	for (size_t i = 0; i < URT_SIZE; i++) {
		mocap_odom.pose_covariance[i] = mocap.covariance[i];
	}

	mocap_odom.vx = NAN;
	mocap_odom.vy = NAN;
	mocap_odom.vz = NAN;
	mocap_odom.rollspeed = NAN;
	mocap_odom.pitchspeed = NAN;
	mocap_odom.yawspeed = NAN;
	mocap_odom.velocity_covariance[0] = NAN;

	int instance_id = 0;

	orb_publish_auto(ORB_ID(vehicle_mocap_odometry), &_mocap_odometry_pub, &mocap_odom, &instance_id, ORB_PRIO_HIGH);
}

void
MavlinkReceiver::handle_message_set_position_target_local_ned(mavlink_message_t *msg)
{
	mavlink_set_position_target_local_ned_t set_position_target_local_ned;
	mavlink_msg_set_position_target_local_ned_decode(msg, &set_position_target_local_ned);

	struct offboard_control_mode_s offboard_control_mode = {};

	bool values_finite =
		PX4_ISFINITE(set_position_target_local_ned.x) &&
		PX4_ISFINITE(set_position_target_local_ned.y) &&
		PX4_ISFINITE(set_position_target_local_ned.z) &&
		PX4_ISFINITE(set_position_target_local_ned.vx) &&
		PX4_ISFINITE(set_position_target_local_ned.vy) &&
		PX4_ISFINITE(set_position_target_local_ned.vz) &&
		PX4_ISFINITE(set_position_target_local_ned.afx) &&
		PX4_ISFINITE(set_position_target_local_ned.afy) &&
		PX4_ISFINITE(set_position_target_local_ned.afz) &&
		PX4_ISFINITE(set_position_target_local_ned.yaw);

	/* Only accept messages which are intended for this system */
	if ((mavlink_system.sysid == set_position_target_local_ned.target_system ||
	     set_position_target_local_ned.target_system == 0) &&
	    (mavlink_system.compid == set_position_target_local_ned.target_component ||
	     set_position_target_local_ned.target_component == 0) &&
	    values_finite) {

		/* convert mavlink type (local, NED) to uORB offboard control struct */
		offboard_control_mode.ignore_position = (bool)(set_position_target_local_ned.type_mask & 0x7);
		offboard_control_mode.ignore_alt_hold = (bool)(set_position_target_local_ned.type_mask & 0x4);
		offboard_control_mode.ignore_velocity = (bool)(set_position_target_local_ned.type_mask & 0x38);
		offboard_control_mode.ignore_acceleration_force = (bool)(set_position_target_local_ned.type_mask & 0x1C0);
		bool is_force_sp = (bool)(set_position_target_local_ned.type_mask & (1 << 9));
		/* yaw ignore flag mapps to ignore_attitude */
		offboard_control_mode.ignore_attitude = (bool)(set_position_target_local_ned.type_mask & 0x400);
		/* yawrate ignore flag mapps to ignore_bodyrate */
		offboard_control_mode.ignore_bodyrate = (bool)(set_position_target_local_ned.type_mask & 0x800);


		bool is_takeoff_sp = (bool)(set_position_target_local_ned.type_mask & 0x1000);
		bool is_land_sp = (bool)(set_position_target_local_ned.type_mask & 0x2000);
		bool is_loiter_sp = (bool)(set_position_target_local_ned.type_mask & 0x3000);
		bool is_idle_sp = (bool)(set_position_target_local_ned.type_mask & 0x4000);

		offboard_control_mode.timestamp = hrt_absolute_time();

		if (_offboard_control_mode_pub == nullptr) {
			_offboard_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &offboard_control_mode);

		} else {
			orb_publish(ORB_ID(offboard_control_mode), _offboard_control_mode_pub, &offboard_control_mode);
		}

		/* If we are in offboard control mode and offboard control loop through is enabled
		 * also publish the setpoint topic which is read by the controller */
		if (_mavlink->get_forward_externalsp()) {
			bool updated;
			orb_check(_control_mode_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
			}

			if (_control_mode.flag_control_offboard_enabled) {
				if (is_force_sp && offboard_control_mode.ignore_position &&
				    offboard_control_mode.ignore_velocity) {

					PX4_WARN("force setpoint not supported");

				} else {
					/* It's not a pure force setpoint: publish to setpoint triplet  topic */
					struct position_setpoint_triplet_s pos_sp_triplet = {};
					pos_sp_triplet.timestamp = hrt_absolute_time();
					pos_sp_triplet.previous.valid = false;
					pos_sp_triplet.next.valid = false;
					pos_sp_triplet.current.valid = true;

					/* Order of statements matters. Takeoff can override loiter.
					 * See https://github.com/mavlink/mavlink/pull/670 for a broader conversation. */
					if (is_loiter_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;

					} else if (is_takeoff_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

					} else if (is_land_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LAND;

					} else if (is_idle_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;

					} else {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
					}

					/* set the local pos values */
					if (!offboard_control_mode.ignore_position) {
						pos_sp_triplet.current.position_valid = true;
						pos_sp_triplet.current.x = set_position_target_local_ned.x;
						pos_sp_triplet.current.y = set_position_target_local_ned.y;
						pos_sp_triplet.current.z = set_position_target_local_ned.z;

					} else {
						pos_sp_triplet.current.position_valid = false;
					}

					/* set the local vel values */
					if (!offboard_control_mode.ignore_velocity) {
						pos_sp_triplet.current.velocity_valid = true;
						pos_sp_triplet.current.vx = set_position_target_local_ned.vx;
						pos_sp_triplet.current.vy = set_position_target_local_ned.vy;
						pos_sp_triplet.current.vz = set_position_target_local_ned.vz;

						pos_sp_triplet.current.velocity_frame =
							set_position_target_local_ned.coordinate_frame;

					} else {
						pos_sp_triplet.current.velocity_valid = false;
					}

					if (!offboard_control_mode.ignore_alt_hold) {
						pos_sp_triplet.current.alt_valid = true;
						pos_sp_triplet.current.z = set_position_target_local_ned.z;

					} else {
						pos_sp_triplet.current.alt_valid = false;
					}

					/* set the local acceleration values if the setpoint type is 'local pos' and none
					 * of the accelerations fields is set to 'ignore' */
					if (!offboard_control_mode.ignore_acceleration_force) {
						pos_sp_triplet.current.acceleration_valid = true;
						pos_sp_triplet.current.a_x = set_position_target_local_ned.afx;
						pos_sp_triplet.current.a_y = set_position_target_local_ned.afy;
						pos_sp_triplet.current.a_z = set_position_target_local_ned.afz;
						pos_sp_triplet.current.acceleration_is_force =
							is_force_sp;

					} else {
						pos_sp_triplet.current.acceleration_valid = false;
					}

					/* set the yaw sp value */
					if (!offboard_control_mode.ignore_attitude) {
						pos_sp_triplet.current.yaw_valid = true;
						pos_sp_triplet.current.yaw = set_position_target_local_ned.yaw;

					} else {
						pos_sp_triplet.current.yaw_valid = false;
					}

					/* set the yawrate sp value */
					if (!offboard_control_mode.ignore_bodyrate) {
						pos_sp_triplet.current.yawspeed_valid = true;
						pos_sp_triplet.current.yawspeed = set_position_target_local_ned.yaw_rate;

					} else {
						pos_sp_triplet.current.yawspeed_valid = false;
					}

					//XXX handle global pos setpoints (different MAV frames)

					if (_pos_sp_triplet_pub == nullptr) {
						_pos_sp_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet),
										    &pos_sp_triplet);

					} else {
						orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub,
							    &pos_sp_triplet);
					}

				}

			}

		}
	}
}

void
MavlinkReceiver::handle_message_set_actuator_control_target(mavlink_message_t *msg)
{
	mavlink_set_actuator_control_target_t set_actuator_control_target;
	mavlink_msg_set_actuator_control_target_decode(msg, &set_actuator_control_target);

	struct offboard_control_mode_s offboard_control_mode = {};

	struct actuator_controls_s actuator_controls = {};

	bool values_finite =
		PX4_ISFINITE(set_actuator_control_target.controls[0]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[1]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[2]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[3]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[4]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[5]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[6]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[7]);

	if ((mavlink_system.sysid == set_actuator_control_target.target_system ||
	     set_actuator_control_target.target_system == 0) &&
	    (mavlink_system.compid == set_actuator_control_target.target_component ||
	     set_actuator_control_target.target_component == 0) &&
	    values_finite) {

		/* ignore all since we are setting raw actuators here */
		offboard_control_mode.ignore_thrust             = true;
		offboard_control_mode.ignore_attitude           = true;
		offboard_control_mode.ignore_bodyrate           = true;
		offboard_control_mode.ignore_position           = true;
		offboard_control_mode.ignore_velocity           = true;
		offboard_control_mode.ignore_acceleration_force = true;

		offboard_control_mode.timestamp = hrt_absolute_time();

		if (_offboard_control_mode_pub == nullptr) {
			_offboard_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &offboard_control_mode);

		} else {
			orb_publish(ORB_ID(offboard_control_mode), _offboard_control_mode_pub, &offboard_control_mode);
		}


		/* If we are in offboard control mode, publish the actuator controls */
		bool updated;
		orb_check(_control_mode_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
		}

		if (_control_mode.flag_control_offboard_enabled) {

			actuator_controls.timestamp = hrt_absolute_time();

			/* Set duty cycles for the servos in the actuator_controls message */
			for (size_t i = 0; i < 8; i++) {
				actuator_controls.control[i] = set_actuator_control_target.controls[i];
			}

			switch (set_actuator_control_target.group_mlx) {
			case 0:
				if (_actuator_controls_pubs[0] == nullptr) {
					_actuator_controls_pubs[0] = orb_advertise(ORB_ID(actuator_controls_0), &actuator_controls);

				} else {
					orb_publish(ORB_ID(actuator_controls_0), _actuator_controls_pubs[0], &actuator_controls);
				}

				break;

			case 1:
				if (_actuator_controls_pubs[1] == nullptr) {
					_actuator_controls_pubs[1] = orb_advertise(ORB_ID(actuator_controls_1), &actuator_controls);

				} else {
					orb_publish(ORB_ID(actuator_controls_1), _actuator_controls_pubs[1], &actuator_controls);
				}

				break;

			case 2:
				if (_actuator_controls_pubs[2] == nullptr) {
					_actuator_controls_pubs[2] = orb_advertise(ORB_ID(actuator_controls_2), &actuator_controls);

				} else {
					orb_publish(ORB_ID(actuator_controls_2), _actuator_controls_pubs[2], &actuator_controls);
				}

				break;

			case 3:
				if (_actuator_controls_pubs[3] == nullptr) {
					_actuator_controls_pubs[3] = orb_advertise(ORB_ID(actuator_controls_3), &actuator_controls);

				} else {
					orb_publish(ORB_ID(actuator_controls_3), _actuator_controls_pubs[3], &actuator_controls);
				}

				break;

			default:
				break;
			}
		}
	}

}

void
MavlinkReceiver::handle_message_gps_global_origin(mavlink_message_t *msg)
{
	mavlink_gps_global_origin_t origin;
	mavlink_msg_gps_global_origin_decode(msg, &origin);

	if (!globallocalconverter_initialized()) {
		/* Set reference point conversion of local coordiantes <--> global coordinates */
		globallocalconverter_init((double)origin.latitude * 1.0e-7, (double)origin.longitude * 1.0e-7,
					  (float)origin.altitude * 1.0e-3f, hrt_absolute_time());
		_global_ref_timestamp = hrt_absolute_time();

	}
}

void
MavlinkReceiver::handle_message_vision_position_estimate(mavlink_message_t *msg)
{
	mavlink_vision_position_estimate_t ev;
	mavlink_msg_vision_position_estimate_decode(msg, &ev);

	struct vehicle_odometry_s visual_odom = {};

	visual_odom.timestamp = _mavlink_timesync.sync_stamp(ev.usec);
	visual_odom.x = ev.x;
	visual_odom.y = ev.y;
	visual_odom.z = ev.z;
	matrix::Quatf q(matrix::Eulerf(ev.roll, ev.pitch, ev.yaw));
	q.copyTo(visual_odom.q);

	// TODO:
	// - add a MAV_FRAME_*_OTHER to the Mavlink MAV_FRAME enum IOT define
	// a frame of reference which is not aligned with NED or ENU
	// - add usage on the estimator side
	visual_odom.local_frame = visual_odom.LOCAL_FRAME_NED;

	const size_t URT_SIZE = sizeof(visual_odom.pose_covariance) / sizeof(visual_odom.pose_covariance[0]);
	static_assert(URT_SIZE == (sizeof(ev.covariance) / sizeof(ev.covariance[0])),
		      "Odometry Pose Covariance matrix URT array size mismatch");

	for (size_t i = 0; i < URT_SIZE; i++) {
		visual_odom.pose_covariance[i] = ev.covariance[i];
	}

	visual_odom.vx = NAN;
	visual_odom.vy = NAN;
	visual_odom.vz = NAN;
	visual_odom.rollspeed = NAN;
	visual_odom.pitchspeed = NAN;
	visual_odom.yawspeed = NAN;
	visual_odom.velocity_covariance[0] = NAN;

	int instance_id = 0;
	orb_publish_auto(ORB_ID(vehicle_visual_odometry), &_visual_odometry_pub, &visual_odom, &instance_id, ORB_PRIO_HIGH);
}

void
MavlinkReceiver::handle_message_odometry(mavlink_message_t *msg)
{
	mavlink_odometry_t odom;
	mavlink_msg_odometry_decode(msg, &odom);

	struct vehicle_odometry_s odometry = {};

	/* Dcm rotation matrix from body frame to local NED frame */
	matrix::Dcmf Rbl;

	odometry.timestamp = _mavlink_timesync.sync_stamp(odom.time_usec);
	/* The position is in the local NED frame */
	odometry.x = odom.x;
	odometry.y = odom.y;
	odometry.z = odom.z;
	/* The quaternion of the ODOMETRY msg represents a rotation from NED
	 * earth/local frame to XYZ body frame */
	matrix::Quatf q(odom.q);
	q.copyTo(odometry.q);

	// TODO:
	// - add a MAV_FRAME_*_OTHER to the Mavlink MAV_FRAME enum IOT define
	// a frame of reference which is not aligned with NED or ENU
	// - add usage on the estimator side
	odometry.local_frame = odometry.LOCAL_FRAME_NED;

	const size_t POS_URT_SIZE = sizeof(odometry.pose_covariance) / sizeof(odometry.pose_covariance[0]);
	const size_t VEL_URT_SIZE = sizeof(odometry.velocity_covariance) / sizeof(odometry.velocity_covariance[0]);
	static_assert(POS_URT_SIZE == (sizeof(odom.pose_covariance) / sizeof(odom.pose_covariance[0])),
		      "Odometry Pose Covariance matrix URT array size mismatch");
	static_assert(VEL_URT_SIZE == (sizeof(odom.twist_covariance) / sizeof(odom.twist_covariance[0])),
		      "Odometry Velocity Covariance matrix URT array size mismatch");

	// create a method to simplify covariance copy
	for (size_t i = 0; i < POS_URT_SIZE; i++) {
		odometry.pose_covariance[i] = odom.pose_covariance[i];
	}

	bool updated;
	orb_check(_vehicle_attitude_sub, &updated);

	if (odom.child_frame_id == MAV_FRAME_BODY_FRD) { /* WRT to estimated vehicle body-fixed frame */
		/* get quaternion from the msg quaternion itself and build DCM matrix from it */
		Rbl = matrix::Dcmf(matrix::Quatf(odometry.q)).I();

		/* the linear velocities needs to be transformed to the local NED frame */\
		matrix::Vector3<float> linvel_local(Rbl * matrix::Vector3<float>(odom.vx, odom.vy, odom.vz));
		odometry.vx = linvel_local(0);
		odometry.vy = linvel_local(1);
		odometry.vz = linvel_local(2);

		odometry.rollspeed = odom.rollspeed;
		odometry.pitchspeed = odom.pitchspeed;
		odometry.yawspeed = odom.yawspeed;

		//TODO: Apply rotation matrix to transform from body-fixed NED to earth-fixed NED frame
		for (size_t i = 0; i < VEL_URT_SIZE; i++) {
			odometry.velocity_covariance[i] = odom.twist_covariance[i];
		}

	} else if (odom.child_frame_id == MAV_FRAME_BODY_NED) { /* WRT to vehicle body-NED frame */
		if (updated) {
			orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_att);

			/* get quaternion from vehicle_attitude quaternion and build DCM matrix from it */
			Rbl = matrix::Dcmf(matrix::Quatf(_att.q)).I();

			/* the linear velocities needs to be transformed to the local NED frame */
			matrix::Vector3<float> linvel_local(Rbl * matrix::Vector3<float>(odom.vx, odom.vy, odom.vz));
			odometry.vx = linvel_local(0);
			odometry.vy = linvel_local(1);
			odometry.vz = linvel_local(2);

			odometry.rollspeed = odom.rollspeed;
			odometry.pitchspeed = odom.pitchspeed;
			odometry.yawspeed = odom.yawspeed;

			//TODO: Apply rotation matrix to transform from body-fixed to earth-fixed NED frame
			for (size_t i = 0; i < VEL_URT_SIZE; i++) {
				odometry.velocity_covariance[i] = odom.twist_covariance[i];
			}

		}

	} else if (odom.child_frame_id == MAV_FRAME_VISION_NED || /* WRT to vehicle local NED frame */
		   odom.child_frame_id == MAV_FRAME_MOCAP_NED) {
		if (updated) {
			orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_att);

			/* get quaternion from vehicle_attitude quaternion and build DCM matrix from it */
			matrix::Dcmf Rlb = matrix::Quatf(_att.q);

			odometry.vx = odom.vx;
			odometry.vy = odom.vy;
			odometry.vz = odom.vz;

			/* the angular rates need to be transformed to the body frame */
			matrix::Vector3<float> angvel_local(Rlb * matrix::Vector3<float>(odom.rollspeed, odom.pitchspeed, odom.yawspeed));
			odometry.rollspeed = angvel_local(0);
			odometry.pitchspeed = angvel_local(1);
			odometry.yawspeed = angvel_local(2);

			//TODO: Apply rotation matrix to transform from earth-fixed to body-fixed NED frame
			for (size_t i = 0; i < VEL_URT_SIZE; i++) {
				odometry.velocity_covariance[i] = odom.twist_covariance[i];
			}

		}

	} else {
		PX4_ERR("Body frame %u not supported. Unable to publish velocity", odom.child_frame_id);
	}

	int instance_id = 0;

	if (odom.frame_id == MAV_FRAME_VISION_NED) {
		orb_publish_auto(ORB_ID(vehicle_visual_odometry), &_visual_odometry_pub, &odometry, &instance_id, ORB_PRIO_HIGH);

	} else if (odom.frame_id == MAV_FRAME_MOCAP_NED) {
		orb_publish_auto(ORB_ID(vehicle_mocap_odometry), &_mocap_odometry_pub, &odometry, &instance_id, ORB_PRIO_HIGH);

	} else {
		PX4_ERR("Local frame %u not supported. Unable to publish pose and velocity", odom.frame_id);
	}
}

void
MavlinkReceiver::handle_message_set_attitude_target(mavlink_message_t *msg)
{
	mavlink_set_attitude_target_t set_attitude_target;
	mavlink_msg_set_attitude_target_decode(msg, &set_attitude_target);

	bool values_finite =
		PX4_ISFINITE(set_attitude_target.q[0]) &&
		PX4_ISFINITE(set_attitude_target.q[1]) &&
		PX4_ISFINITE(set_attitude_target.q[2]) &&
		PX4_ISFINITE(set_attitude_target.q[3]) &&
		PX4_ISFINITE(set_attitude_target.thrust) &&
		PX4_ISFINITE(set_attitude_target.body_roll_rate) &&
		PX4_ISFINITE(set_attitude_target.body_pitch_rate) &&
		PX4_ISFINITE(set_attitude_target.body_yaw_rate);

	/* Only accept messages which are intended for this system */
	if ((mavlink_system.sysid == set_attitude_target.target_system ||
	     set_attitude_target.target_system == 0) &&
	    (mavlink_system.compid == set_attitude_target.target_component ||
	     set_attitude_target.target_component == 0) &&
	    values_finite) {

		/* set correct ignore flags for thrust field: copy from mavlink message */
		_offboard_control_mode.ignore_thrust = (bool)(set_attitude_target.type_mask & (1 << 6));

		/*
		 * The tricky part in parsing this message is that the offboard sender *can* set attitude and thrust
		 * using different messages. Eg.: First send set_attitude_target containing the attitude and ignore
		 * bits set for everything else and then send set_attitude_target containing the thrust and ignore bits
		 * set for everything else.
		 */

		/*
		 * if attitude or body rate have been used (not ignored) previously and this message only sends
		 * throttle and has the ignore bits set for attitude and rates don't change the flags for attitude and
		 * body rates to keep the controllers running
		 */
		bool ignore_bodyrate_msg = (bool)(set_attitude_target.type_mask & 0x7);
		bool ignore_attitude_msg = (bool)(set_attitude_target.type_mask & (1 << 7));

		if (ignore_bodyrate_msg && ignore_attitude_msg && !_offboard_control_mode.ignore_thrust) {
			/* Message want's us to ignore everything except thrust: only ignore if previously ignored */
			_offboard_control_mode.ignore_bodyrate = ignore_bodyrate_msg && _offboard_control_mode.ignore_bodyrate;
			_offboard_control_mode.ignore_attitude = ignore_attitude_msg && _offboard_control_mode.ignore_attitude;

		} else {
			_offboard_control_mode.ignore_bodyrate = ignore_bodyrate_msg;
			_offboard_control_mode.ignore_attitude = ignore_attitude_msg;
		}

		_offboard_control_mode.ignore_position = true;
		_offboard_control_mode.ignore_velocity = true;
		_offboard_control_mode.ignore_acceleration_force = true;

		_offboard_control_mode.timestamp = hrt_absolute_time();

		if (_offboard_control_mode_pub == nullptr) {
			_offboard_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &_offboard_control_mode);

		} else {
			orb_publish(ORB_ID(offboard_control_mode), _offboard_control_mode_pub, &_offboard_control_mode);
		}

		/* If we are in offboard control mode and offboard control loop through is enabled
		 * also publish the setpoint topic which is read by the controller */
		if (_mavlink->get_forward_externalsp()) {
			bool updated;
			orb_check(_control_mode_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
			}

			if (_control_mode.flag_control_offboard_enabled) {

				/* Publish attitude setpoint if attitude and thrust ignore bits are not set */
				if (!(_offboard_control_mode.ignore_attitude)) {
					vehicle_attitude_setpoint_s att_sp = {};
					att_sp.timestamp = hrt_absolute_time();

					if (!ignore_attitude_msg) { // only copy att sp if message contained new data
						matrix::Quatf q(set_attitude_target.q);
						q.copyTo(att_sp.q_d);
						att_sp.q_d_valid = true;

						matrix::Eulerf euler{q};
						att_sp.roll_body = euler.phi();
						att_sp.pitch_body = euler.theta();
						att_sp.yaw_body = euler.psi();
						att_sp.yaw_sp_move_rate = 0.0f;
					}

					// TODO: We assume offboard is only used for multicopters which produce thrust along the
					// body z axis. If we want to support fixed wing as well we need to handle it differently here, e.g.
					// in that case we should assign att_sp.thrust_body[0]
					if (!_offboard_control_mode.ignore_thrust) { // dont't overwrite thrust if it's invalid
						att_sp.thrust_body[2] = -set_attitude_target.thrust;
					}

					if (_att_sp_pub == nullptr) {
						_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

					} else {
						orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &att_sp);
					}
				}

				/* Publish attitude rate setpoint if bodyrate and thrust ignore bits are not set */
				///XXX add support for ignoring individual axes
				if (!(_offboard_control_mode.ignore_bodyrate)) {
					vehicle_rates_setpoint_s rates_sp = {};
					rates_sp.timestamp = hrt_absolute_time();

					if (!ignore_bodyrate_msg) { // only copy att rates sp if message contained new data
						rates_sp.roll = set_attitude_target.body_roll_rate;
						rates_sp.pitch = set_attitude_target.body_pitch_rate;
						rates_sp.yaw = set_attitude_target.body_yaw_rate;
					}

					if (!_offboard_control_mode.ignore_thrust) { // dont't overwrite thrust if it's invalid
						rates_sp.thrust_body[2] = -set_attitude_target.thrust;
					}

					if (_rates_sp_pub == nullptr) {
						_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);

					} else {
						orb_publish(ORB_ID(vehicle_rates_setpoint), _rates_sp_pub, &rates_sp);
					}
				}
			}

		}
	}
}

void
MavlinkReceiver::handle_message_radio_status(mavlink_message_t *msg)
{
	/* telemetry status supported only on first ORB_MULTI_MAX_INSTANCES mavlink channels */
	if (_mavlink->get_channel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {
		mavlink_radio_status_t rstatus;
		mavlink_msg_radio_status_decode(msg, &rstatus);

		radio_status_s status = {};
		status.timestamp = hrt_absolute_time();
		status.rssi = rstatus.rssi;
		status.remote_rssi = rstatus.remrssi;
		status.txbuf = rstatus.txbuf;
		status.noise = rstatus.noise;
		status.remote_noise = rstatus.remnoise;
		status.rxerrors = rstatus.rxerrors;
		status.fix = rstatus.fixed;

		_mavlink->update_radio_status(status);

		int multi_instance;
		orb_publish_auto(ORB_ID(radio_status), &_radio_status_pub, &status, &multi_instance, ORB_PRIO_HIGH);
	}
}

void
MavlinkReceiver::handle_message_ping(mavlink_message_t *msg)
{
	mavlink_ping_t ping;
	mavlink_msg_ping_decode(msg, &ping);

	if ((ping.target_system == 0) &&
	    (ping.target_component == 0)) {	   // This is a ping request. Return it to the system which requested the ping.

		ping.target_system = msg->sysid;
		ping.target_component = msg->compid;
		mavlink_msg_ping_send_struct(_mavlink->get_channel(), &ping);

	} else if ((ping.target_system == mavlink_system.sysid) &&
		   (ping.target_component ==
		    mavlink_system.compid)) {	// This is a returned ping message from this system. Calculate latency from it.

		const hrt_abstime now = hrt_absolute_time();

		// Calculate round trip time
		float rtt_ms = (now - ping.time_usec) / 1000.0f;

		// Update ping statistics
		struct Mavlink::ping_statistics_s &pstats = _mavlink->get_ping_statistics();

		pstats.last_ping_time = now;

		if (pstats.last_ping_seq == 0 && ping.seq > 0) {
			// This is the first reply we are receiving from an offboard system.
			// We may have been broadcasting pings for some time before it came online,
			// and these do not count as dropped packets.

			// Reset last_ping_seq counter for correct packet drop detection
			pstats.last_ping_seq = ping.seq - 1;
		}

		// We can only count dropped packets after the first message
		if (ping.seq > pstats.last_ping_seq) {
			pstats.dropped_packets += ping.seq - pstats.last_ping_seq - 1;
		}

		pstats.last_ping_seq = ping.seq;
		pstats.last_rtt = rtt_ms;
		pstats.mean_rtt = (rtt_ms + pstats.mean_rtt) / 2.0f;
		pstats.max_rtt = fmaxf(rtt_ms, pstats.max_rtt);
		pstats.min_rtt = pstats.min_rtt > 0.0f ? fminf(rtt_ms, pstats.min_rtt) : rtt_ms;

		/* Ping status is supported only on first ORB_MULTI_MAX_INSTANCES mavlink channels */
		if (_mavlink->get_channel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {

			struct ping_s uorb_ping_msg = {};

			uorb_ping_msg.timestamp = now;
			uorb_ping_msg.ping_time = ping.time_usec;
			uorb_ping_msg.ping_sequence = ping.seq;
			uorb_ping_msg.dropped_packets = pstats.dropped_packets;
			uorb_ping_msg.rtt_ms = rtt_ms;
			uorb_ping_msg.system_id = msg->sysid;
			uorb_ping_msg.component_id = msg->compid;

			if (_ping_pub == nullptr) {
				int multi_instance;
				_ping_pub = orb_advertise_multi(ORB_ID(ping), &uorb_ping_msg, &multi_instance, ORB_PRIO_DEFAULT);

			} else {
				orb_publish(ORB_ID(ping), _ping_pub, &uorb_ping_msg);
			}
		}
	}
}

void
MavlinkReceiver::handle_message_battery_status(mavlink_message_t *msg)
{
	// external battery measurements
	mavlink_battery_status_t battery_mavlink;
	mavlink_msg_battery_status_decode(msg, &battery_mavlink);

	battery_status_s battery_status = {};
	battery_status.timestamp = hrt_absolute_time();

	float voltage_sum = 0.0f;
	uint8_t cell_count = 0;

	while (battery_mavlink.voltages[cell_count] < UINT16_MAX && cell_count < 10) {
		voltage_sum += (float)(battery_mavlink.voltages[cell_count]) / 1000.0f;
		cell_count++;
	}

	battery_status.voltage_v = voltage_sum;
	battery_status.voltage_filtered_v  = voltage_sum;
	battery_status.current_a = battery_status.current_filtered_a = (float)(battery_mavlink.current_battery) / 100.0f;
	battery_status.current_filtered_a = battery_status.current_a;
	battery_status.remaining = (float)battery_mavlink.battery_remaining / 100.0f;
	battery_status.discharged_mah = (float)battery_mavlink.current_consumed;
	battery_status.cell_count = cell_count;
	battery_status.connected = true;

	// Get the battery level thresholds.
	float bat_emergen_thr;
	float bat_crit_thr;
	float bat_low_thr;
	param_get(_p_bat_emergen_thr, &bat_emergen_thr);
	param_get(_p_bat_crit_thr, &bat_crit_thr);
	param_get(_p_bat_low_thr, &bat_low_thr);

	// Set the battery warning based on remaining charge.
	//  Note: Smallest values must come first in evaluation.
	if (battery_status.remaining < bat_emergen_thr) {
		battery_status.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;

	} else if (battery_status.remaining < bat_crit_thr) {
		battery_status.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

	} else if (battery_status.remaining < bat_low_thr) {
		battery_status.warning = battery_status_s::BATTERY_WARNING_LOW;
	}

	if (_battery_pub == nullptr) {
		_battery_pub = orb_advertise(ORB_ID(battery_status), &battery_status);

	} else {
		orb_publish(ORB_ID(battery_status), _battery_pub, &battery_status);
	}
}

void
MavlinkReceiver::handle_message_serial_control(mavlink_message_t *msg)
{
	mavlink_serial_control_t serial_control_mavlink;
	mavlink_msg_serial_control_decode(msg, &serial_control_mavlink);

	// we only support shell commands
	if (serial_control_mavlink.device != SERIAL_CONTROL_DEV_SHELL
	    || (serial_control_mavlink.flags & SERIAL_CONTROL_FLAG_REPLY)) {
		return;
	}

	MavlinkShell *shell = _mavlink->get_shell();

	if (shell) {
		// we ignore the timeout, EXCLUSIVE & BLOCKING flags of the SERIAL_CONTROL message
		if (serial_control_mavlink.count > 0) {
			shell->write(serial_control_mavlink.data, serial_control_mavlink.count);
		}

		// if no response requested, assume the shell is no longer used
		if ((serial_control_mavlink.flags & SERIAL_CONTROL_FLAG_RESPOND) == 0) {
			_mavlink->close_shell();
		}
	}
}

void
MavlinkReceiver::handle_message_logging_ack(mavlink_message_t *msg)
{
	mavlink_logging_ack_t logging_ack;
	mavlink_msg_logging_ack_decode(msg, &logging_ack);

	MavlinkULog *ulog_streaming = _mavlink->get_ulog_streaming();

	if (ulog_streaming) {
		ulog_streaming->handle_ack(logging_ack);
	}
}

void
MavlinkReceiver::handle_message_play_tune(mavlink_message_t *msg)
{
	mavlink_play_tune_t play_tune;
	mavlink_msg_play_tune_decode(msg, &play_tune);

	char *tune = play_tune.tune;

	if ((mavlink_system.sysid == play_tune.target_system ||
	     play_tune.target_system == 0) &&
	    (mavlink_system.compid == play_tune.target_component ||
	     play_tune.target_component == 0)) {

		if (*tune == 'M') {
			int fd = px4_open(TONEALARM0_DEVICE_PATH, PX4_F_WRONLY);

			if (fd >= 0) {
				px4_write(fd, tune, strlen(tune) + 1);
				px4_close(fd);
			}
		}
	}
}

void
MavlinkReceiver::handle_message_obstacle_distance(mavlink_message_t *msg)
{
	mavlink_obstacle_distance_t mavlink_obstacle_distance;
	mavlink_msg_obstacle_distance_decode(msg, &mavlink_obstacle_distance);

	obstacle_distance_s obstacle_distance = {};
	obstacle_distance.timestamp = hrt_absolute_time();
	obstacle_distance.sensor_type = mavlink_obstacle_distance.sensor_type;
	memcpy(obstacle_distance.distances, mavlink_obstacle_distance.distances, sizeof(obstacle_distance.distances));
	obstacle_distance.increment = mavlink_obstacle_distance.increment;
	obstacle_distance.min_distance = mavlink_obstacle_distance.min_distance;
	obstacle_distance.max_distance = mavlink_obstacle_distance.max_distance;

	if (_obstacle_distance_pub == nullptr) {
		_obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &obstacle_distance);

	} else {
		orb_publish(ORB_ID(obstacle_distance), _obstacle_distance_pub, &obstacle_distance);
	}
}

void
MavlinkReceiver::handle_message_trajectory_representation_waypoints(mavlink_message_t *msg)
{
	mavlink_trajectory_representation_waypoints_t trajectory;
	mavlink_msg_trajectory_representation_waypoints_decode(msg, &trajectory);


	struct vehicle_trajectory_waypoint_s trajectory_waypoint = {};

	trajectory_waypoint.timestamp = hrt_absolute_time();
	const int number_valid_points = trajectory.valid_points;

	for (int i = 0; i < vehicle_trajectory_waypoint_s::NUMBER_POINTS; ++i) {
		trajectory_waypoint.waypoints[i].position[0] = trajectory.pos_x[i];
		trajectory_waypoint.waypoints[i].position[1] = trajectory.pos_y[i];
		trajectory_waypoint.waypoints[i].position[2] = trajectory.pos_z[i];

		trajectory_waypoint.waypoints[i].velocity[0] = trajectory.vel_x[i];
		trajectory_waypoint.waypoints[i].velocity[1] = trajectory.vel_y[i];
		trajectory_waypoint.waypoints[i].velocity[2] = trajectory.vel_z[i];

		trajectory_waypoint.waypoints[i].acceleration[0] = trajectory.acc_x[i];
		trajectory_waypoint.waypoints[i].acceleration[1] = trajectory.acc_y[i];
		trajectory_waypoint.waypoints[i].acceleration[2] = trajectory.acc_z[i];

		trajectory_waypoint.waypoints[i].yaw = trajectory.pos_yaw[i];
		trajectory_waypoint.waypoints[i].yaw_speed = trajectory.vel_yaw[i];

	}

	for (int i = 0; i < number_valid_points; ++i) {
		trajectory_waypoint.waypoints[i].point_valid = true;
	}

	for (int i = number_valid_points; i < vehicle_trajectory_waypoint_s::NUMBER_POINTS; ++i) {
		trajectory_waypoint.waypoints[i].point_valid = false;
	}

	if (_trajectory_waypoint_pub == nullptr) {
		_trajectory_waypoint_pub = orb_advertise(ORB_ID(vehicle_trajectory_waypoint), &trajectory_waypoint);

	} else {
		orb_publish(ORB_ID(vehicle_trajectory_waypoint), _trajectory_waypoint_pub, &trajectory_waypoint);
	}

}

switch_pos_t
MavlinkReceiver::decode_switch_pos(uint16_t buttons, unsigned sw)
{
	// This 2-bit method should be used in the future: (buttons >> (sw * 2)) & 3;
	return (buttons & (1 << sw)) ? manual_control_setpoint_s::SWITCH_POS_ON : manual_control_setpoint_s::SWITCH_POS_OFF;
}

int
MavlinkReceiver::decode_switch_pos_n(uint16_t buttons, unsigned sw)
{

	bool on = (buttons & (1 << sw));

	if (sw < MOM_SWITCH_COUNT) {

		bool last_on = (_mom_switch_state & (1 << sw));

		/* first switch is 2-pos, rest is 2 pos */
		unsigned state_count = (sw == 0) ? 3 : 2;

		/* only transition on low state */
		if (!on && (on != last_on)) {

			_mom_switch_pos[sw]++;

			if (_mom_switch_pos[sw] == state_count) {
				_mom_switch_pos[sw] = 0;
			}
		}

		/* state_count - 1 is the number of intervals and 1000 is the range,
		 * with 2 states 0 becomes 0, 1 becomes 1000. With
		 * 3 states 0 becomes 0, 1 becomes 500, 2 becomes 1000,
		 * and so on for more states.
		 */
		return (_mom_switch_pos[sw] * 1000) / (state_count - 1) + 1000;

	} else {
		/* return the current state */
		return on * 1000 + 1000;
	}
}

void
MavlinkReceiver::handle_message_rc_channels_override(mavlink_message_t *msg)
{
	mavlink_rc_channels_override_t man;
	mavlink_msg_rc_channels_override_decode(msg, &man);

	// Check target
	if (man.target_system != 0 && man.target_system != _mavlink->get_system_id()) {
		return;
	}

	// fill uORB message
	struct input_rc_s rc = {};
	// metadata
	rc.timestamp = hrt_absolute_time();
	rc.timestamp_last_signal = rc.timestamp;
	rc.channel_count = 18;
	rc.rssi = RC_INPUT_RSSI_MAX;
	rc.rc_failsafe = false;
	rc.rc_lost = false;
	rc.rc_lost_frame_count = 0;
	rc.rc_total_frame_count = 1;
	rc.rc_ppm_frame_length = 0;
	rc.input_source = input_rc_s::RC_INPUT_SOURCE_MAVLINK;
	// channels
	rc.values[0] = man.chan1_raw;
	rc.values[1] = man.chan2_raw;
	rc.values[2] = man.chan3_raw;
	rc.values[3] = man.chan4_raw;
	rc.values[4] = man.chan5_raw;
	rc.values[5] = man.chan6_raw;
	rc.values[6] = man.chan7_raw;
	rc.values[7] = man.chan8_raw;
	rc.values[8] = man.chan9_raw;
	rc.values[9] = man.chan10_raw;
	rc.values[10] = man.chan11_raw;
	rc.values[11] = man.chan12_raw;
	rc.values[12] = man.chan13_raw;
	rc.values[13] = man.chan14_raw;
	rc.values[14] = man.chan15_raw;
	rc.values[15] = man.chan16_raw;
	rc.values[16] = man.chan17_raw;
	rc.values[17] = man.chan18_raw;

	// publish uORB message
	int instance; // provides the instance ID or the publication
	ORB_PRIO priority = ORB_PRIO_HIGH; // since it is an override, set priority high
	orb_publish_auto(ORB_ID(input_rc), &_rc_pub, &rc, &instance, priority);
}

void
MavlinkReceiver::handle_message_manual_control(mavlink_message_t *msg)
{
	mavlink_manual_control_t man;
	mavlink_msg_manual_control_decode(msg, &man);

	// Check target
	if (man.target != 0 && man.target != _mavlink->get_system_id()) {
		return;
	}

	if (_mavlink->get_manual_input_mode_generation()) {

		struct input_rc_s rc = {};
		rc.timestamp = hrt_absolute_time();
		rc.timestamp_last_signal = rc.timestamp;

		rc.channel_count = 8;
		rc.rc_failsafe = false;
		rc.rc_lost = false;
		rc.rc_lost_frame_count = 0;
		rc.rc_total_frame_count = 1;
		rc.rc_ppm_frame_length = 0;
		rc.input_source = input_rc_s::RC_INPUT_SOURCE_MAVLINK;
		rc.rssi = RC_INPUT_RSSI_MAX;

		/* roll */
		rc.values[0] = man.x / 2 + 1500;
		/* pitch */
		rc.values[1] = man.y / 2 + 1500;
		/* yaw */
		rc.values[2] = man.r / 2 + 1500;
		/* throttle */
		rc.values[3] = fminf(fmaxf(man.z / 0.9f + 800, 1000.0f), 2000.0f);

		/* decode all switches which fit into the channel mask */
		unsigned max_switch = (sizeof(man.buttons) * 8);
		unsigned max_channels = (sizeof(rc.values) / sizeof(rc.values[0]));

		if (max_switch > (max_channels - 4)) {
			max_switch = (max_channels - 4);
		}

		/* fill all channels */
		for (unsigned i = 0; i < max_switch; i++) {
			rc.values[i + 4] = decode_switch_pos_n(man.buttons, i);
		}

		_mom_switch_state = man.buttons;

		if (_rc_pub == nullptr) {
			_rc_pub = orb_advertise(ORB_ID(input_rc), &rc);

		} else {
			orb_publish(ORB_ID(input_rc), _rc_pub, &rc);
		}

	} else {
		struct manual_control_setpoint_s manual = {};

		manual.timestamp = hrt_absolute_time();
		manual.x = man.x / 1000.0f;
		manual.y = man.y / 1000.0f;
		manual.r = man.r / 1000.0f;
		manual.z = man.z / 1000.0f;
		manual.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0 + _mavlink->get_instance_id();

		int m_inst;
		orb_publish_auto(ORB_ID(manual_control_setpoint), &_manual_pub, &manual, &m_inst, ORB_PRIO_LOW);
	}
}

void
MavlinkReceiver::handle_message_heartbeat(mavlink_message_t *msg)
{
	/* telemetry status supported only on first TELEMETRY_STATUS_ORB_ID_NUM mavlink channels */
	if (_mavlink->get_channel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {
		mavlink_heartbeat_t hb;
		mavlink_msg_heartbeat_decode(msg, &hb);

		/* ignore own heartbeats, accept only heartbeats from GCS */
		if (msg->sysid != mavlink_system.sysid && hb.type == MAV_TYPE_GCS) {

			telemetry_status_s &tstatus = _mavlink->get_telemetry_status();

			/* set heartbeat time and topic time and publish -
			 * the telem status also gets updated on telemetry events
			 */
			tstatus.heartbeat_time = tstatus.timestamp;
			tstatus.system_id = msg->sysid;
			tstatus.component_id = msg->compid;
		}
	}
}

int
MavlinkReceiver::set_message_interval(int msgId, float interval, int data_rate)
{
	if (msgId == MAVLINK_MSG_ID_HEARTBEAT) {
		return PX4_ERROR;
	}

	if (data_rate > 0) {
		_mavlink->set_data_rate(data_rate);
	}

	// configure_stream wants a rate (msgs/second), so convert here.
	float rate = 0.f;

	if (interval < -0.00001f) {
		rate = 0.f; // stop the stream

	} else if (interval > 0.00001f) {
		rate = 1000000.0f / interval;

	} else {
		rate = -2.f; // set default rate
	}

	bool found_id = false;


	// The interval between two messages is in microseconds.
	// Set to -1 to disable and 0 to request default rate
	if (msgId != 0) {
		const char *stream_name = get_stream_name(msgId);

		if (stream_name != nullptr) {
			_mavlink->configure_stream_threadsafe(stream_name, rate);
			found_id = true;
		}
	}

	return (found_id ? PX4_OK : PX4_ERROR);
}

void
MavlinkReceiver::get_message_interval(int msgId)
{
	unsigned interval = 0;

	MavlinkStream *stream = nullptr;
	LL_FOREACH(_mavlink->get_streams(), stream) {
		if (stream->get_id() == msgId) {
			interval = stream->get_interval();
			break;
		}
	}

	// send back this value...
	mavlink_msg_message_interval_send(_mavlink->get_channel(), msgId, interval);
}

void
MavlinkReceiver::handle_message_hil_sensor(mavlink_message_t *msg)
{
	mavlink_hil_sensor_t imu;
	mavlink_msg_hil_sensor_decode(msg, &imu);

	uint64_t timestamp = hrt_absolute_time();

	/* airspeed */
	{
		struct airspeed_s airspeed = {};

		float ias = calc_indicated_airspeed(imu.diff_pressure * 1e2f);
		float tas = calc_true_airspeed_from_indicated(ias, imu.abs_pressure * 100, imu.temperature);

		airspeed.timestamp = timestamp;
		airspeed.indicated_airspeed_m_s = ias;
		airspeed.true_airspeed_m_s = tas;

		if (_airspeed_pub == nullptr) {
			_airspeed_pub = orb_advertise(ORB_ID(airspeed), &airspeed);

		} else {
			orb_publish(ORB_ID(airspeed), _airspeed_pub, &airspeed);
		}
	}

	/* gyro */
	{
		sensor_gyro_s gyro = {};

		gyro.timestamp = timestamp;
		gyro.x_raw = imu.xgyro * 1000.0f;
		gyro.y_raw = imu.ygyro * 1000.0f;
		gyro.z_raw = imu.zgyro * 1000.0f;
		gyro.x = imu.xgyro;
		gyro.y = imu.ygyro;
		gyro.z = imu.zgyro;
		gyro.temperature = imu.temperature;

		if (_gyro_pub == nullptr) {
			_gyro_pub = orb_advertise(ORB_ID(sensor_gyro), &gyro);

		} else {
			orb_publish(ORB_ID(sensor_gyro), _gyro_pub, &gyro);
		}
	}

	/* accelerometer */
	{
		sensor_accel_s accel = {};

		accel.timestamp = timestamp;
		accel.x_raw = imu.xacc / (CONSTANTS_ONE_G / 1000.0f);
		accel.y_raw = imu.yacc / (CONSTANTS_ONE_G / 1000.0f);
		accel.z_raw = imu.zacc / (CONSTANTS_ONE_G / 1000.0f);
		accel.x = imu.xacc;
		accel.y = imu.yacc;
		accel.z = imu.zacc;
		accel.temperature = imu.temperature;

		if (_accel_pub == nullptr) {
			_accel_pub = orb_advertise(ORB_ID(sensor_accel), &accel);

		} else {
			orb_publish(ORB_ID(sensor_accel), _accel_pub, &accel);
		}
	}

	/* magnetometer */
	{
		struct mag_report mag = {};

		mag.timestamp = timestamp;
		mag.x_raw = imu.xmag * 1000.0f;
		mag.y_raw = imu.ymag * 1000.0f;
		mag.z_raw = imu.zmag * 1000.0f;
		mag.x = imu.xmag;
		mag.y = imu.ymag;
		mag.z = imu.zmag;

		if (_mag_pub == nullptr) {
			/* publish to the first mag topic */
			_mag_pub = orb_advertise(ORB_ID(sensor_mag), &mag);

		} else {
			orb_publish(ORB_ID(sensor_mag), _mag_pub, &mag);
		}
	}

	/* baro */
	{
		sensor_baro_s baro = {};

		baro.timestamp = timestamp;
		baro.pressure = imu.abs_pressure;
		baro.temperature = imu.temperature;

		/* fake device ID */
		baro.device_id = 1234567;

		if (_baro_pub == nullptr) {
			_baro_pub = orb_advertise(ORB_ID(sensor_baro), &baro);

		} else {
			orb_publish(ORB_ID(sensor_baro), _baro_pub, &baro);
		}
	}

	/* battery status */
	{
		struct battery_status_s hil_battery_status = {};

		hil_battery_status.timestamp = timestamp;
		hil_battery_status.voltage_v = 11.5f;
		hil_battery_status.voltage_filtered_v = 11.5f;
		hil_battery_status.current_a = 10.0f;
		hil_battery_status.discharged_mah = -1.0f;

		if (_battery_pub == nullptr) {
			_battery_pub = orb_advertise(ORB_ID(battery_status), &hil_battery_status);

		} else {
			orb_publish(ORB_ID(battery_status), _battery_pub, &hil_battery_status);
		}
	}

	/* increment counters */
	_hil_frames++;

	/* print HIL sensors rate */
	if ((timestamp - _old_timestamp) > 10000000) {
		// printf("receiving HIL sensors at %d hz\n", _hil_frames / 10);
		_old_timestamp = timestamp;
		_hil_frames = 0;
	}
}

void
MavlinkReceiver::handle_message_hil_gps(mavlink_message_t *msg)
{
	mavlink_hil_gps_t gps;
	mavlink_msg_hil_gps_decode(msg, &gps);

	uint64_t timestamp = hrt_absolute_time();

	struct vehicle_gps_position_s hil_gps = {};

	hil_gps.timestamp_time_relative = 0;
	hil_gps.time_utc_usec = gps.time_usec;

	hil_gps.timestamp = timestamp;
	hil_gps.lat = gps.lat;
	hil_gps.lon = gps.lon;
	hil_gps.alt = gps.alt;
	hil_gps.eph = (float)gps.eph * 1e-2f; // from cm to m
	hil_gps.epv = (float)gps.epv * 1e-2f; // from cm to m

	hil_gps.s_variance_m_s = 0.1f;

	hil_gps.vel_m_s = (float)gps.vel * 1e-2f; // from cm/s to m/s
	hil_gps.vel_n_m_s = gps.vn * 1e-2f; // from cm to m
	hil_gps.vel_e_m_s = gps.ve * 1e-2f; // from cm to m
	hil_gps.vel_d_m_s = gps.vd * 1e-2f; // from cm to m
	hil_gps.vel_ned_valid = true;
	hil_gps.cog_rad = ((gps.cog == 65535) ? NAN : wrap_2pi(math::radians(gps.cog * 1e-2f)));

	hil_gps.fix_type = gps.fix_type;
	hil_gps.satellites_used = gps.satellites_visible;  //TODO: rename mavlink_hil_gps_t sats visible to used?

	hil_gps.heading = NAN;
	hil_gps.heading_offset = NAN;

	if (_gps_pub == nullptr) {
		_gps_pub = orb_advertise(ORB_ID(vehicle_gps_position), &hil_gps);

	} else {
		orb_publish(ORB_ID(vehicle_gps_position), _gps_pub, &hil_gps);
	}
}

void MavlinkReceiver::handle_message_follow_target(mavlink_message_t *msg)
{
	mavlink_follow_target_t follow_target_msg;
	follow_target_s follow_target_topic = {};

	mavlink_msg_follow_target_decode(msg, &follow_target_msg);

	follow_target_topic.timestamp = hrt_absolute_time();

	follow_target_topic.lat = follow_target_msg.lat * 1e-7;
	follow_target_topic.lon = follow_target_msg.lon * 1e-7;
	follow_target_topic.alt = follow_target_msg.alt;

	if (_follow_target_pub == nullptr) {
		_follow_target_pub = orb_advertise(ORB_ID(follow_target), &follow_target_topic);

	} else {
		orb_publish(ORB_ID(follow_target), _follow_target_pub, &follow_target_topic);
	}
}

void MavlinkReceiver::handle_message_landing_target(mavlink_message_t *msg)
{
	mavlink_landing_target_t landing_target;

	mavlink_msg_landing_target_decode(msg, &landing_target);

	if (landing_target.position_valid && landing_target.frame == MAV_FRAME_LOCAL_NED) {
		landing_target_pose_s landing_target_pose = {};
		landing_target_pose.timestamp = _mavlink_timesync.sync_stamp(landing_target.time_usec);
		landing_target_pose.abs_pos_valid = true;
		landing_target_pose.x_abs = landing_target.x;
		landing_target_pose.y_abs = landing_target.y;
		landing_target_pose.z_abs = landing_target.z;

		if (_landing_target_pose_pub == nullptr) {
			_landing_target_pose_pub = orb_advertise(ORB_ID(landing_target_pose), &landing_target_pose);

		} else {
			orb_publish(ORB_ID(landing_target_pose), _landing_target_pose_pub, &landing_target_pose);
		}
	}
}

void MavlinkReceiver::handle_message_adsb_vehicle(mavlink_message_t *msg)
{
	mavlink_adsb_vehicle_t adsb;
	transponder_report_s t = {};

	mavlink_msg_adsb_vehicle_decode(msg, &adsb);

	t.timestamp = hrt_absolute_time();

	t.icao_address = adsb.ICAO_address;
	t.lat = adsb.lat * 1e-7;
	t.lon = adsb.lon * 1e-7;
	t.altitude_type = adsb.altitude_type;
	t.altitude = adsb.altitude / 1000.0f;
	t.heading = adsb.heading / 100.0f / 180.0f * M_PI_F - M_PI_F;
	t.hor_velocity = adsb.hor_velocity / 100.0f;
	t.ver_velocity = adsb.ver_velocity / 100.0f;
	memcpy(&t.callsign[0], &adsb.callsign[0], sizeof(t.callsign));
	t.emitter_type = adsb.emitter_type;
	t.tslc = adsb.tslc;
	t.squawk = adsb.squawk;

	t.flags = transponder_report_s::PX4_ADSB_FLAGS_RETRANSLATE;  //Unset in receiver already broadcast its messages

	if (adsb.flags & ADSB_FLAGS_VALID_COORDS) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS; }

	if (adsb.flags & ADSB_FLAGS_VALID_ALTITUDE) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE; }

	if (adsb.flags & ADSB_FLAGS_VALID_HEADING) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING; }

	if (adsb.flags & ADSB_FLAGS_VALID_VELOCITY) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY; }

	if (adsb.flags & ADSB_FLAGS_VALID_CALLSIGN) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN; }

	if (adsb.flags & ADSB_FLAGS_VALID_SQUAWK) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_SQUAWK; }

	//PX4_INFO("code: %d callsign: %s, vel: %8.4f, tslc: %d", (int)t.ICAO_address, t.callsign, (double)t.hor_velocity, (int)t.tslc);

	if (_transponder_report_pub == nullptr) {
		_transponder_report_pub = orb_advertise_queue(ORB_ID(transponder_report), &t, transponder_report_s::ORB_QUEUE_LENGTH);

	} else {
		orb_publish(ORB_ID(transponder_report), _transponder_report_pub, &t);
	}
}

void MavlinkReceiver::handle_message_collision(mavlink_message_t *msg)
{
	mavlink_collision_t collision;
	collision_report_s t = {};

	mavlink_msg_collision_decode(msg, &collision);

	t.timestamp = hrt_absolute_time();
	t.src = collision.src;
	t.id = collision.id;
	t.action = collision.action;
	t.threat_level = collision.threat_level;
	t.time_to_minimum_delta = collision.time_to_minimum_delta;
	t.altitude_minimum_delta = collision.altitude_minimum_delta;
	t.horizontal_minimum_delta = collision.horizontal_minimum_delta;

	if (_collision_report_pub == nullptr) {
		_collision_report_pub = orb_advertise(ORB_ID(collision_report), &t);

	} else {
		orb_publish(ORB_ID(collision_report), _collision_report_pub, &t);
	}
}

void MavlinkReceiver::handle_message_gps_rtcm_data(mavlink_message_t *msg)
{
	mavlink_gps_rtcm_data_t gps_rtcm_data_msg = {};
	mavlink_msg_gps_rtcm_data_decode(msg, &gps_rtcm_data_msg);

	gps_inject_data_s gps_inject_data_topic = {};
	gps_inject_data_topic.len = math::min((int)sizeof(gps_rtcm_data_msg.data),
					      (int)sizeof(uint8_t) * gps_rtcm_data_msg.len);
	gps_inject_data_topic.flags = gps_rtcm_data_msg.flags;
	memcpy(gps_inject_data_topic.data, gps_rtcm_data_msg.data,
	       math::min((int)sizeof(gps_inject_data_topic.data), (int)sizeof(uint8_t) * gps_inject_data_topic.len));

	orb_advert_t &pub = _gps_inject_data_pub;

	if (pub == nullptr) {
		pub = orb_advertise_queue(ORB_ID(gps_inject_data), &gps_inject_data_topic,
					  _gps_inject_data_queue_size);

	} else {
		orb_publish(ORB_ID(gps_inject_data), pub, &gps_inject_data_topic);
	}

}

void
MavlinkReceiver::handle_message_hil_state_quaternion(mavlink_message_t *msg)
{
	mavlink_hil_state_quaternion_t hil_state;
	mavlink_msg_hil_state_quaternion_decode(msg, &hil_state);

	uint64_t timestamp = hrt_absolute_time();

	/* airspeed */
	{
		struct airspeed_s airspeed = {};

		airspeed.timestamp = timestamp;
		airspeed.indicated_airspeed_m_s = hil_state.ind_airspeed * 1e-2f;
		airspeed.true_airspeed_m_s = hil_state.true_airspeed * 1e-2f;

		if (_airspeed_pub == nullptr) {
			_airspeed_pub = orb_advertise(ORB_ID(airspeed), &airspeed);

		} else {
			orb_publish(ORB_ID(airspeed), _airspeed_pub, &airspeed);
		}
	}

	/* attitude */
	struct vehicle_attitude_s hil_attitude;
	{
		hil_attitude = {};
		hil_attitude.timestamp = timestamp;

		matrix::Quatf q(hil_state.attitude_quaternion);
		q.copyTo(hil_attitude.q);

		hil_attitude.rollspeed = hil_state.rollspeed;
		hil_attitude.pitchspeed = hil_state.pitchspeed;
		hil_attitude.yawspeed = hil_state.yawspeed;

		if (_attitude_pub == nullptr) {
			_attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &hil_attitude);

		} else {
			orb_publish(ORB_ID(vehicle_attitude), _attitude_pub, &hil_attitude);
		}
	}

	/* global position */
	{
		struct vehicle_global_position_s hil_global_pos = {};

		hil_global_pos.timestamp = timestamp;
		hil_global_pos.lat = hil_state.lat / ((double)1e7);
		hil_global_pos.lon = hil_state.lon / ((double)1e7);
		hil_global_pos.alt = hil_state.alt / 1000.0f;
		hil_global_pos.vel_n = hil_state.vx / 100.0f;
		hil_global_pos.vel_e = hil_state.vy / 100.0f;
		hil_global_pos.vel_d = hil_state.vz / 100.0f;
		hil_global_pos.eph = 2.0f;
		hil_global_pos.epv = 4.0f;

		if (_global_pos_pub == nullptr) {
			_global_pos_pub = orb_advertise(ORB_ID(vehicle_global_position), &hil_global_pos);

		} else {
			orb_publish(ORB_ID(vehicle_global_position), _global_pos_pub, &hil_global_pos);
		}
	}

	/* local position */
	{
		double lat = hil_state.lat * 1e-7;
		double lon = hil_state.lon * 1e-7;

		if (!_hil_local_proj_inited) {
			_hil_local_proj_inited = true;
			_hil_local_alt0 = hil_state.alt / 1000.0f;
			map_projection_init(&_hil_local_proj_ref, lat, lon);
			_hil_local_pos.ref_timestamp = timestamp;
			_hil_local_pos.ref_lat = lat;
			_hil_local_pos.ref_lon = lon;
			_hil_local_pos.ref_alt = _hil_local_alt0;
		}

		float x = 0.0f;
		float y = 0.0f;
		map_projection_project(&_hil_local_proj_ref, lat, lon, &x, &y);
		_hil_local_pos.timestamp = timestamp;
		_hil_local_pos.xy_valid = true;
		_hil_local_pos.z_valid = true;
		_hil_local_pos.v_xy_valid = true;
		_hil_local_pos.v_z_valid = true;
		_hil_local_pos.x = x;
		_hil_local_pos.y = y;
		_hil_local_pos.z = _hil_local_alt0 - hil_state.alt / 1000.0f;
		_hil_local_pos.vx = hil_state.vx / 100.0f;
		_hil_local_pos.vy = hil_state.vy / 100.0f;
		_hil_local_pos.vz = hil_state.vz / 100.0f;
		matrix::Eulerf euler = matrix::Quatf(hil_attitude.q);
		_hil_local_pos.yaw = euler.psi();
		_hil_local_pos.xy_global = true;
		_hil_local_pos.z_global = true;
		_hil_local_pos.vxy_max = INFINITY;
		_hil_local_pos.vz_max = INFINITY;
		_hil_local_pos.hagl_min = INFINITY;
		_hil_local_pos.hagl_max = INFINITY;

		if (_local_pos_pub == nullptr) {
			_local_pos_pub = orb_advertise(ORB_ID(vehicle_local_position), &_hil_local_pos);

		} else {
			orb_publish(ORB_ID(vehicle_local_position), _local_pos_pub, &_hil_local_pos);
		}
	}

	/* land detector */
	{
		bool landed = (float)(hil_state.alt) / 1000.0f < (_hil_local_alt0 + 0.1f); // XXX improve?

		if (_hil_land_detector.landed != landed) {
			_hil_land_detector.landed = landed;
			_hil_land_detector.timestamp = hrt_absolute_time();

			if (_land_detector_pub == nullptr) {
				_land_detector_pub = orb_advertise(ORB_ID(vehicle_land_detected), &_hil_land_detector);

			} else {
				orb_publish(ORB_ID(vehicle_land_detected), _land_detector_pub, &_hil_land_detector);
			}
		}
	}

	/* accelerometer */
	{
		sensor_accel_s accel = {};

		accel.timestamp = timestamp;
		accel.x_raw = hil_state.xacc / CONSTANTS_ONE_G * 1e3f;
		accel.y_raw = hil_state.yacc / CONSTANTS_ONE_G * 1e3f;
		accel.z_raw = hil_state.zacc / CONSTANTS_ONE_G * 1e3f;
		accel.x = hil_state.xacc;
		accel.y = hil_state.yacc;
		accel.z = hil_state.zacc;
		accel.temperature = 25.0f;

		if (_accel_pub == nullptr) {
			_accel_pub = orb_advertise(ORB_ID(sensor_accel), &accel);

		} else {
			orb_publish(ORB_ID(sensor_accel), _accel_pub, &accel);
		}
	}

	/* battery status */
	{
		struct battery_status_s	hil_battery_status = {};

		hil_battery_status.timestamp = timestamp;
		hil_battery_status.voltage_v = 11.1f;
		hil_battery_status.voltage_filtered_v = 11.1f;
		hil_battery_status.current_a = 10.0f;
		hil_battery_status.discharged_mah = -1.0f;

		if (_battery_pub == nullptr) {
			_battery_pub = orb_advertise(ORB_ID(battery_status), &hil_battery_status);

		} else {
			orb_publish(ORB_ID(battery_status), _battery_pub, &hil_battery_status);
		}
	}
}

void MavlinkReceiver::handle_message_named_value_float(mavlink_message_t *msg)
{
	mavlink_named_value_float_t debug_msg;
	debug_key_value_s debug_topic;

	mavlink_msg_named_value_float_decode(msg, &debug_msg);

	debug_topic.timestamp = hrt_absolute_time();
	memcpy(debug_topic.key, debug_msg.name, sizeof(debug_topic.key));
	debug_topic.key[sizeof(debug_topic.key) - 1] = '\0'; // enforce null termination
	debug_topic.value = debug_msg.value;

	if (_debug_key_value_pub == nullptr) {
		_debug_key_value_pub = orb_advertise(ORB_ID(debug_key_value), &debug_topic);

	} else {
		orb_publish(ORB_ID(debug_key_value), _debug_key_value_pub, &debug_topic);
	}
}

void MavlinkReceiver::handle_message_debug(mavlink_message_t *msg)
{
	mavlink_debug_t debug_msg;
	debug_value_s debug_topic;

	mavlink_msg_debug_decode(msg, &debug_msg);

	debug_topic.timestamp = hrt_absolute_time();
	debug_topic.ind = debug_msg.ind;
	debug_topic.value = debug_msg.value;

	if (_debug_value_pub == nullptr) {
		_debug_value_pub = orb_advertise(ORB_ID(debug_value), &debug_topic);

	} else {
		orb_publish(ORB_ID(debug_value), _debug_value_pub, &debug_topic);
	}
}

void MavlinkReceiver::handle_message_debug_vect(mavlink_message_t *msg)
{
	mavlink_debug_vect_t debug_msg;
	debug_vect_s debug_topic;

	mavlink_msg_debug_vect_decode(msg, &debug_msg);

	debug_topic.timestamp = hrt_absolute_time();
	memcpy(debug_topic.name, debug_msg.name, sizeof(debug_topic.name));
	debug_topic.name[sizeof(debug_topic.name) - 1] = '\0'; // enforce null termination
	debug_topic.x = debug_msg.x;
	debug_topic.y = debug_msg.y;
	debug_topic.z = debug_msg.z;

	if (_debug_vect_pub == nullptr) {
		_debug_vect_pub = orb_advertise(ORB_ID(debug_vect), &debug_topic);

	} else {
		orb_publish(ORB_ID(debug_vect), _debug_vect_pub, &debug_topic);
	}
}

void MavlinkReceiver::handle_message_debug_float_array(mavlink_message_t *msg)
{
	mavlink_debug_float_array_t debug_msg;
	debug_array_s debug_topic = {};

	mavlink_msg_debug_float_array_decode(msg, &debug_msg);

	debug_topic.timestamp = hrt_absolute_time();
	debug_topic.id = debug_msg.array_id;
	memcpy(debug_topic.name, debug_msg.name, sizeof(debug_topic.name));
	debug_topic.name[sizeof(debug_topic.name) - 1] = '\0'; // enforce null termination

	for (size_t i = 0; i < debug_array_s::ARRAY_SIZE; i++) {
		debug_topic.data[i] = debug_msg.data[i];
	}

	if (_debug_array_pub == nullptr) {
		_debug_array_pub = orb_advertise(ORB_ID(debug_array), &debug_topic);

	} else {
		orb_publish(ORB_ID(debug_array), _debug_array_pub, &debug_topic);
	}
}

/**
 * Receive data from UART/UDP
 */
void *
MavlinkReceiver::receive_thread(void *arg)
{

	/* set thread name */
	{
		char thread_name[24];
		sprintf(thread_name, "mavlink_rcv_if%d", _mavlink->get_instance_id());
		px4_prctl(PR_SET_NAME, thread_name, px4_getpid());
	}

	// poll timeout in ms. Also defines the max update frequency of the mission & param manager, etc.
	const int timeout = 10;

#if defined(__PX4_POSIX)
	/* 1500 is the Wifi MTU, so we make sure to fit a full packet */
	uint8_t buf[1600 * 5];
#elif defined(CONFIG_NET)
	/* 1500 is the Wifi MTU, so we make sure to fit a full packet */
	uint8_t buf[1000];
#else
	/* the serial port buffers internally as well, we just need to fit a small chunk */
	uint8_t buf[64];
#endif
	mavlink_message_t msg;

	struct pollfd fds[1] = {};

	if (_mavlink->get_protocol() == SERIAL) {
		fds[0].fd = _mavlink->get_uart_fd();
		fds[0].events = POLLIN;
	}

#if defined(CONFIG_NET) || defined(__PX4_POSIX)
	struct sockaddr_in srcaddr = {};
	socklen_t addrlen = sizeof(srcaddr);

	if (_mavlink->get_protocol() == UDP || _mavlink->get_protocol() == TCP) {
		// make sure mavlink app has booted before we start using the socket
		while (!_mavlink->boot_complete()) {
			px4_usleep(100000);
		}

		fds[0].fd = _mavlink->get_socket_fd();
		fds[0].events = POLLIN;
	}

#endif
	ssize_t nread = 0;
	hrt_abstime last_send_update = 0;

	while (!_mavlink->_task_should_exit) {
		if (poll(&fds[0], 1, timeout) > 0) {
			if (_mavlink->get_protocol() == SERIAL) {

				/*
				 * to avoid reading very small chunks wait for data before reading
				 * this is designed to target one message, so >20 bytes at a time
				 */
				const unsigned character_count = 20;

				/* non-blocking read. read may return negative values */
				if ((nread = ::read(fds[0].fd, buf, sizeof(buf))) < (ssize_t)character_count) {
					const unsigned sleeptime = character_count * 1000000 / (_mavlink->get_baudrate() / 10);
					px4_usleep(sleeptime);
				}
			}

#if defined(CONFIG_NET) || defined(__PX4_POSIX)

			if (_mavlink->get_protocol() == UDP) {
				if (fds[0].revents & POLLIN) {
					nread = recvfrom(_mavlink->get_socket_fd(), buf, sizeof(buf), 0, (struct sockaddr *)&srcaddr, &addrlen);
				}

			} else {
				// could be TCP or other protocol
			}

			struct sockaddr_in &srcaddr_last = _mavlink->get_client_source_address();

			int localhost = (127 << 24) + 1;

			if (!_mavlink->get_client_source_initialized()) {

				// set the address either if localhost or if 3 seconds have passed
				// this ensures that a GCS running on localhost can get a hold of
				// the system within the first N seconds
				hrt_abstime stime = _mavlink->get_start_time();

				if ((stime != 0 && (hrt_elapsed_time(&stime) > 3 * 1000 * 1000))
				    || (srcaddr_last.sin_addr.s_addr == htonl(localhost))) {
					srcaddr_last.sin_addr.s_addr = srcaddr.sin_addr.s_addr;
					srcaddr_last.sin_port = srcaddr.sin_port;
					_mavlink->set_client_source_initialized();
					PX4_INFO("partner IP: %s", inet_ntoa(srcaddr.sin_addr));
				}
			}

#endif
			// only start accepting messages once we're sure who we talk to

			if (_mavlink->get_client_source_initialized()) {
				/* if read failed, this loop won't execute */
				for (ssize_t i = 0; i < nread; i++) {
					if (mavlink_parse_char(_mavlink->get_channel(), buf[i], &msg, &_status)) {

						/* check if we received version 2 and request a switch. */
						if (!(_mavlink->get_status()->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1)) {
							/* this will only switch to proto version 2 if allowed in settings */
							_mavlink->set_proto_version(2);
						}

						/* handle generic messages and commands */
						handle_message(&msg);

						/* handle packet with mission manager */
						_mission_manager.handle_message(&msg);


						/* handle packet with parameter component */
						_parameters_manager.handle_message(&msg);

						if (_mavlink->ftp_enabled()) {
							/* handle packet with ftp component */
							_mavlink_ftp.handle_message(&msg);
						}

						/* handle packet with log component */
						_mavlink_log_handler.handle_message(&msg);

						/* handle packet with timesync component */
						_mavlink_timesync.handle_message(&msg);

						/* handle packet with parent object */
						_mavlink->handle_message(&msg);
					}
				}

				/* count received bytes (nread will be -1 on read error) */
				if (nread > 0) {
					_mavlink->count_rxbytes(nread);
				}
			}
		}

		hrt_abstime t = hrt_absolute_time();

		if (t - last_send_update > timeout * 1000) {
			_mission_manager.check_active_mission();
			_mission_manager.send(t);

			_parameters_manager.send(t);

			if (_mavlink->ftp_enabled()) {
				_mavlink_ftp.send(t);
			}

			_mavlink_log_handler.send(t);
			last_send_update = t;
		}

	}

	return nullptr;
}

void MavlinkReceiver::print_status()
{

}

void *MavlinkReceiver::start_helper(void *context)
{

	MavlinkReceiver *rcv = new MavlinkReceiver((Mavlink *)context);

	if (!rcv) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	void *ret = rcv->receive_thread(nullptr);

	delete rcv;

	return ret;
}

void
MavlinkReceiver::receive_start(pthread_t *thread, Mavlink *parent)
{
	pthread_attr_t receiveloop_attr;
	pthread_attr_init(&receiveloop_attr);

	struct sched_param param;
	(void)pthread_attr_getschedparam(&receiveloop_attr, &param);
	param.sched_priority = SCHED_PRIORITY_MAX - 80;
	(void)pthread_attr_setschedparam(&receiveloop_attr, &param);

	pthread_attr_setstacksize(&receiveloop_attr, PX4_STACK_ADJUSTED(2840 + MAVLINK_RECEIVER_NET_ADDED_STACK));
	pthread_create(thread, &receiveloop_attr, MavlinkReceiver::start_helper, (void *)parent);

	pthread_attr_destroy(&receiveloop_attr);
}
/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file mavlink_shell.cpp
 * A shell to be used via MAVLink
 *
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include "mavlink_shell.h"
#include <px4_defines.h>

#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>


#ifdef __PX4_NUTTX
#include <nshlib/nshlib.h>
#endif /* __PX4_NUTTX */

#ifdef __PX4_CYGWIN
#include <asm/socket.h>
#endif

MavlinkShell::~MavlinkShell()
{
	//closing the pipes will stop the thread as well
	if (_to_shell_fd >= 0) {
		PX4_INFO("Stopping mavlink shell");
		close(_to_shell_fd);
	}

	if (_from_shell_fd >= 0) {
		close(_from_shell_fd);
	}
}

int MavlinkShell::start()
{
	//this currently only works for NuttX
#ifndef __PX4_NUTTX
	return -1;
#endif /* __PX4_NUTTX */


	PX4_INFO("Starting mavlink shell");

	int p1[2], p2[2];

	/* Create the shell task and redirect its stdin & stdout. If we used pthread, we would redirect
	 * stdin/out of the calling process as well, so we need px4_task_spawn_cmd. However NuttX only
	 * keeps (duplicates) the first 3 fd's when creating a new task, all others are not inherited.
	 * This means we need to temporarily change the first 3 fd's of the current task (or at least
	 * the first 2 if stdout=stderr).
	 * And we hope :-) that during the temporary phase, no other thread from the same task writes to
	 * stdout (as it would end up in the pipe).
	 */

	if (pipe(p1) != 0) {
		return -errno;
	}

	if (pipe(p2) != 0) {
		close(p1[0]);
		close(p1[1]);
		return -errno;
	}

	int ret = 0;

	_from_shell_fd  = p1[0];
	_to_shell_fd = p2[1];
	_shell_fds[0]  = p2[0];
	_shell_fds[1] = p1[1];

	int fd_backups[2]; //we don't touch stderr, we will redirect it to stdout in the startup of the shell task

	for (int i = 0; i < 2; ++i) {
		fd_backups[i] = dup(i);

		if (fd_backups[i] == -1) {
			ret = -errno;
		}
	}

	dup2(_shell_fds[0], 0);
	dup2(_shell_fds[1], 1);

	if (ret == 0) {
		_task = px4_task_spawn_cmd("mavlink_shell",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_DEFAULT,
					   2048,
					   &MavlinkShell::shell_start_thread,
					   nullptr);

		if (_task < 0) {
			ret = -1;
		}
	}

	//restore fd's
	for (int i = 0; i < 2; ++i) {
		if (dup2(fd_backups[i], i) == -1) {
			ret = -errno;
		}

		close(fd_backups[i]);
	}

	//close unused pipe fd's
	close(_shell_fds[0]);
	close(_shell_fds[1]);

	return ret;
}

int MavlinkShell::shell_start_thread(int argc, char *argv[])
{
	dup2(1, 2); //redirect stderror to stdout

#ifdef __PX4_NUTTX
	nsh_consolemain(0, NULL);
#endif /* __PX4_NUTTX */

	return 0;
}

size_t MavlinkShell::write(uint8_t *buffer, size_t len)
{
	return ::write(_to_shell_fd, buffer, len);
}

size_t MavlinkShell::read(uint8_t *buffer, size_t len)
{
	return ::read(_from_shell_fd, buffer, len);
}

size_t MavlinkShell::available()
{
	int ret = 0;

	if (ioctl(_from_shell_fd, FIONREAD, (unsigned long)&ret) == OK) {
		return ret;
	}

	return 0;
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_simple_analyzer.cpp
 *
 * @author Achermann Florian <acfloria@ethz.ch>
 */

#include "mavlink_simple_analyzer.h"

#include <float.h>

#include <px4_log.h>
#include <px4_defines.h>

SimpleAnalyzer::SimpleAnalyzer(Mode mode, float window) :
	_window(window),
	_mode(mode)
{
	reset();
}

void SimpleAnalyzer::reset()
{
	_n = 0;

	switch (_mode) {
	case AVERAGE:
		_result = 0.0f;

		break;

	case MIN:
		_result = FLT_MAX;

		break;

	case MAX:
		_result = FLT_MIN;

		break;

	default:
		PX4_ERR("SimpleAnalyzer: Unknown mode.");
	}
}

void SimpleAnalyzer::add_value(float val, float update_rate)
{
	switch (_mode) {
	case AVERAGE:
		_result = (_result * _n + val) / (_n + 1u);

		break;

	case MIN:
		if (val < _result) {
			_result = val;
		}

		break;

	case MAX:
		if (val > _result) {
			_result = val;
		}

		break;
	}

	// if we get more measurements than n_max so the exponential moving average
	// is computed
	if ((_n < update_rate * _window) && (update_rate > 1.0f)) {
		_n++;
	}

	// value sanity checks
	if (!PX4_ISFINITE(_result)) {
		PX4_DEBUG("SimpleAnalyzer: Result is not finite, reset the analyzer.");
		reset();
	}
}

bool SimpleAnalyzer::valid() const
{
	return _n > 0u;
}

float SimpleAnalyzer::get() const
{
	return _result;
}

float SimpleAnalyzer::get_scaled(float scalingfactor) const
{
	return get() * scalingfactor;
}

void SimpleAnalyzer::check_limits(float &x, float min, float max) const
{
	if (x > max) {
		x = max;

	} else if (x < min) {
		x = min;
	}
}

void SimpleAnalyzer::int_round(float &x) const
{
	if (x < 0) {
		x -= 0.5f;

	} else {
		x += 0.5f;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2014-2017 PX4 Development Team. All rights reserved.
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
 * @file mavlink_stream.cpp
 * Mavlink messages stream implementation.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <stdlib.h>

#include "mavlink_stream.h"
#include "mavlink_main.h"

MavlinkStream::MavlinkStream(Mavlink *mavlink) :
	ModuleParams(nullptr),
	_mavlink(mavlink)
{
	_last_sent = hrt_absolute_time();
}

/**
 * Update subscriptions and send message if necessary
 */
int
MavlinkStream::update(const hrt_abstime &t)
{
	update_data();
	ModuleParams::updateParams();

	// If the message has never been sent before we want
	// to send it immediately and can return right away
	if (_last_sent == 0) {
		// this will give different messages on the same run a different
		// initial timestamp which will help spacing them out
		// on the link scheduling
		if (send(t)) {
			_last_sent = hrt_absolute_time();

			if (!_first_message_sent) {
				_first_message_sent = true;
			}
		}

		return 0;
	}

	// One of the previous iterations sent the update
	// already before the deadline
	if (_last_sent > t) {
		return -1;
	}

	int64_t dt = t - _last_sent;
	int interval = (_interval > 0) ? _interval : 0;

	if (!const_rate()) {
		interval /= _mavlink->get_rate_mult();
	}

	// Send the message if it is due or
	// if it will overrun the next scheduled send interval
	// by 30% of the interval time. This helps to avoid
	// sending a scheduled message on average slower than
	// scheduled. Doing this at 50% would risk sending
	// the message too often as the loop runtime of the app
	// needs to be accounted for as well.
	// This method is not theoretically optimal but a suitable
	// stopgap as it hits its deadlines well (0.5 Hz, 50 Hz and 250 Hz)

	if (interval == 0 || (dt > (interval - (_mavlink->get_main_loop_delay() / 10) * 3))) {
		// interval expired, send message

		// If the interval is non-zero and dt is smaller than 1.5 times the interval
		// do not use the actual time but increment at a fixed rate, so that processing delays do not
		// distort the average rate. The check of the maximum interval is done to ensure that after a
		// long time not sending anything, sending multiple messages in a short time is avoided.
		if (send(t)) {
			_last_sent = ((interval > 0) && ((int64_t)(1.5f * interval) > dt)) ? _last_sent + interval : t;

			if (!_first_message_sent) {
				_first_message_sent = true;
			}

			return 0;

		} else {
			return -1;
		}
	}

	return -1;
}
/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file mavlink_ulog.cpp
 * ULog data streaming via MAVLink
 *
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include "mavlink_ulog.h"
#include <px4_log.h>
#include <errno.h>
#include <mathlib/mathlib.h>

bool MavlinkULog::_init = false;
MavlinkULog *MavlinkULog::_instance = nullptr;
px4_sem_t MavlinkULog::_lock;
const float MavlinkULog::_rate_calculation_delta_t = 0.1f;


MavlinkULog::MavlinkULog(int datarate, float max_rate_factor, uint8_t target_system, uint8_t target_component)
	: _target_system(target_system), _target_component(target_component),
	  _max_rate_factor(max_rate_factor),
	  _max_num_messages(math::max(1, (int)ceilf(_rate_calculation_delta_t *_max_rate_factor * datarate /
				      (MAVLINK_MSG_ID_LOGGING_DATA_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES)))),
	  _current_rate_factor(max_rate_factor)
{
	_ulog_stream_sub = orb_subscribe(ORB_ID(ulog_stream));

	if (_ulog_stream_sub < 0) {
		PX4_ERR("orb_subscribe failed (%i)", errno);

	} else {
		// make sure we won't read any old messages
		struct ulog_stream_s stream_msg;
		bool update;

		while (orb_check(_ulog_stream_sub, &update) == 0 && update) {
			orb_copy(ORB_ID(ulog_stream), _ulog_stream_sub, &stream_msg);
		}
	}

	_waiting_for_initial_ack = true;
	_last_sent_time = hrt_absolute_time(); //(ab)use this timestamp during initialization
	_next_rate_check = _last_sent_time + _rate_calculation_delta_t * 1.e6f;
}

MavlinkULog::~MavlinkULog()
{
	if (_ulog_stream_ack_pub) {
		orb_unadvertise(_ulog_stream_ack_pub);
	}

	if (_ulog_stream_sub >= 0) {
		orb_unsubscribe(_ulog_stream_sub);
	}
}

void MavlinkULog::start_ack_received()
{
	if (_waiting_for_initial_ack) {
		_last_sent_time = 0;
		_waiting_for_initial_ack = false;
		PX4_DEBUG("got logger ack");
	}
}

int MavlinkULog::handle_update(mavlink_channel_t channel)
{
	static_assert(sizeof(ulog_stream_s::data) == MAVLINK_MSG_LOGGING_DATA_FIELD_DATA_LEN,
		      "Invalid uorb ulog_stream.data length");
	static_assert(sizeof(ulog_stream_s::data) == MAVLINK_MSG_LOGGING_DATA_ACKED_FIELD_DATA_LEN,
		      "Invalid uorb ulog_stream.data length");

	if (_waiting_for_initial_ack) {
		if (hrt_elapsed_time(&_last_sent_time) > 3e5) {
			PX4_WARN("no ack from logger (is it running?)");
			return -1;
		}

		return 0;
	}

	// check if we're waiting for an ACK
	if (_last_sent_time) {
		bool check_for_updates = false;

		if (_ack_received) {
			_last_sent_time = 0;
			check_for_updates = true;

		} else {

			if (hrt_elapsed_time(&_last_sent_time) > ulog_stream_ack_s::ACK_TIMEOUT * 1000) {
				if (++_sent_tries > ulog_stream_ack_s::ACK_MAX_TRIES) {
					return -ETIMEDOUT;

				} else {
					PX4_DEBUG("re-sending ulog mavlink message (try=%i)", _sent_tries);
					_last_sent_time = hrt_absolute_time();
					mavlink_logging_data_acked_t msg;
					msg.sequence = _ulog_data.sequence;
					msg.length = _ulog_data.length;
					msg.first_message_offset = _ulog_data.first_message_offset;
					msg.target_system = _target_system;
					msg.target_component = _target_component;
					memcpy(msg.data, _ulog_data.data, sizeof(msg.data));
					mavlink_msg_logging_data_acked_send_struct(channel, &msg);
				}
			}
		}

		if (!check_for_updates) {
			return 0;
		}
	}

	bool updated = false;
	int ret = orb_check(_ulog_stream_sub, &updated);

	while (updated && !ret && _current_num_msgs < _max_num_messages) {
		orb_copy(ORB_ID(ulog_stream), _ulog_stream_sub, &_ulog_data);

		if (_ulog_data.timestamp > 0) {
			if (_ulog_data.flags & ulog_stream_s::FLAGS_NEED_ACK) {
				_sent_tries = 1;
				_last_sent_time = hrt_absolute_time();
				lock();
				_wait_for_ack_sequence = _ulog_data.sequence;
				_ack_received = false;
				unlock();

				mavlink_logging_data_acked_t msg;
				msg.sequence = _ulog_data.sequence;
				msg.length = _ulog_data.length;
				msg.first_message_offset = _ulog_data.first_message_offset;
				msg.target_system = _target_system;
				msg.target_component = _target_component;
				memcpy(msg.data, _ulog_data.data, sizeof(msg.data));
				mavlink_msg_logging_data_acked_send_struct(channel, &msg);

			} else {
				mavlink_logging_data_t msg;
				msg.sequence = _ulog_data.sequence;
				msg.length = _ulog_data.length;
				msg.first_message_offset = _ulog_data.first_message_offset;
				msg.target_system = _target_system;
				msg.target_component = _target_component;
				memcpy(msg.data, _ulog_data.data, sizeof(msg.data));
				mavlink_msg_logging_data_send_struct(channel, &msg);
			}
		}

		++_current_num_msgs;
		ret = orb_check(_ulog_stream_sub, &updated);
	}

	//need to update the rate?
	hrt_abstime t = hrt_absolute_time();

	if (t > _next_rate_check) {
		if (_current_num_msgs < _max_num_messages) {
			_current_rate_factor = _max_rate_factor * (float)_current_num_msgs / _max_num_messages;

		} else {
			_current_rate_factor = _max_rate_factor;
		}

		_current_num_msgs = 0;
		_next_rate_check = t + _rate_calculation_delta_t * 1.e6f;
		PX4_DEBUG("current rate=%.3f (max=%i msgs in %.3fs)", (double)_current_rate_factor, _max_num_messages,
			  (double)_rate_calculation_delta_t);
	}

	return 0;
}

void MavlinkULog::initialize()
{
	if (_init) {
		return;
	}

	px4_sem_init(&_lock, 1, 1);
	_init = true;
}

MavlinkULog *MavlinkULog::try_start(int datarate, float max_rate_factor, uint8_t target_system,
				    uint8_t target_component)
{
	MavlinkULog *ret = nullptr;
	bool failed = false;
	lock();

	if (!_instance) {
		ret = _instance = new MavlinkULog(datarate, max_rate_factor, target_system, target_component);

		if (!_instance) {
			failed = true;
		}
	}

	unlock();

	if (failed) {
		PX4_ERR("alloc failed");
	}

	return ret;
}

void MavlinkULog::stop()
{
	lock();

	if (_instance) {
		delete _instance;
		_instance = nullptr;
	}

	unlock();
}

void MavlinkULog::handle_ack(mavlink_logging_ack_t ack)
{
	lock();

	if (_instance) { // make sure stop() was not called right before
		if (_wait_for_ack_sequence == ack.sequence) {
			_ack_received = true;
			publish_ack(ack.sequence);
		}
	}

	unlock();
}

void MavlinkULog::publish_ack(uint16_t sequence)
{
	ulog_stream_ack_s ack;
	ack.timestamp = hrt_absolute_time();
	ack.sequence = sequence;

	if (_ulog_stream_ack_pub == nullptr) {
		_ulog_stream_ack_pub = orb_advertise(ORB_ID(ulog_stream_ack), &ack);

	} else {
		orb_publish(ORB_ID(ulog_stream_ack), _ulog_stream_ack_pub, &ack);
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_timesync.cpp
 * Mavlink timesync implementation.
 *
 * @author Mohammed Kabir <mhkabir98@gmail.com>
 */

#include "mavlink_timesync.h"
#include "mavlink_main.h"

MavlinkTimesync::MavlinkTimesync(Mavlink *mavlink) :
	_mavlink(mavlink)
{
}

MavlinkTimesync::~MavlinkTimesync()
{
	if (_timesync_status_pub) {
		orb_unadvertise(_timesync_status_pub);
	}
}

void
MavlinkTimesync::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_TIMESYNC: {

			mavlink_timesync_t tsync = {};
			mavlink_msg_timesync_decode(msg, &tsync);

			const uint64_t now = hrt_absolute_time();

			if (tsync.tc1 == 0) {			// Message originating from remote system, timestamp and return it

				mavlink_timesync_t rsync;

				rsync.tc1 = now * 1000ULL;
				rsync.ts1 = tsync.ts1;

				mavlink_msg_timesync_send_struct(_mavlink->get_channel(), &rsync);

				return;

			} else if (tsync.tc1 > 0) {		// Message originating from this system, compute time offset from it

				// Calculate time offset between this system and the remote system, assuming RTT for
				// the timesync packet is roughly equal both ways.
				int64_t offset_us = (int64_t)((tsync.ts1 / 1000ULL) + now - (tsync.tc1 / 1000ULL) * 2) / 2 ;

				// Calculate the round trip time (RTT) it took the timesync packet to bounce back to us from remote system
				uint64_t rtt_us = now - (tsync.ts1 / 1000ULL);

				// Calculate the difference of this sample from the current estimate
				uint64_t deviation = llabs((int64_t)_time_offset - offset_us);

				if (rtt_us < MAX_RTT_SAMPLE) {	// Only use samples with low RTT

					if (sync_converged() && (deviation > MAX_DEVIATION_SAMPLE)) {

						// Increment the counter if we have a good estimate and are getting samples far from the estimate
						_high_deviation_count++;

						// We reset the filter if we received 5 consecutive samples which violate our present estimate.
						// This is most likely due to a time jump on the offboard system.
						if (_high_deviation_count > MAX_CONSECUTIVE_HIGH_DEVIATION) {
							PX4_ERR("[timesync] Time jump detected. Resetting time synchroniser.");
							// Reset the filter
							reset_filter();
						}

					} else {

						// Filter gain scheduling
						if (!sync_converged()) {
							// Interpolate with a sigmoid function
							double progress = (double)_sequence / (double)CONVERGENCE_WINDOW;
							double p = 1.0 - exp(0.5 * (1.0 - 1.0 / (1.0 - progress)));
							_filter_alpha = p * ALPHA_GAIN_FINAL + (1.0 - p) * ALPHA_GAIN_INITIAL;
							_filter_beta = p * BETA_GAIN_FINAL + (1.0 - p) * BETA_GAIN_INITIAL;

						} else {
							_filter_alpha = ALPHA_GAIN_FINAL;
							_filter_beta = BETA_GAIN_FINAL;
						}

						// Perform filter update
						add_sample(offset_us);

						// Increment sequence counter after filter update
						_sequence++;

						// Reset high deviation count after filter update
						_high_deviation_count = 0;

						// Reset high RTT count after filter update
						_high_rtt_count = 0;
					}

				} else {
					// Increment counter if round trip time is too high for accurate timesync
					_high_rtt_count++;

					if (_high_rtt_count > MAX_CONSECUTIVE_HIGH_RTT) {
						PX4_WARN("[timesync] RTT too high for timesync: %llu ms", rtt_us / 1000ULL);
						// Reset counter to rate-limit warnings
						_high_rtt_count = 0;
					}

				}

				// Publish status message
				struct timesync_status_s tsync_status = {};

				tsync_status.timestamp = hrt_absolute_time();
				tsync_status.remote_timestamp = tsync.tc1 / 1000ULL;
				tsync_status.observed_offset = offset_us;
				tsync_status.estimated_offset = (int64_t)_time_offset;
				tsync_status.round_trip_time = rtt_us;

				if (_timesync_status_pub == nullptr) {
					int instance;
					_timesync_status_pub = orb_advertise_multi(ORB_ID(timesync_status), &tsync_status, &instance, ORB_PRIO_DEFAULT);

				} else {
					orb_publish(ORB_ID(timesync_status), _timesync_status_pub, &tsync_status);
				}

			}

			break;
		}

	case MAVLINK_MSG_ID_SYSTEM_TIME: {

			mavlink_system_time_t time;
			mavlink_msg_system_time_decode(msg, &time);

			timespec tv = {};
			px4_clock_gettime(CLOCK_REALTIME, &tv);

			// date -d @1234567890: Sat Feb 14 02:31:30 MSK 2009
			bool onb_unix_valid = (unsigned long long)tv.tv_sec > PX4_EPOCH_SECS;
			bool ofb_unix_valid = time.time_unix_usec > PX4_EPOCH_SECS * 1000ULL;

			if (!onb_unix_valid && ofb_unix_valid) {
				tv.tv_sec = time.time_unix_usec / 1000000ULL;
				tv.tv_nsec = (time.time_unix_usec % 1000000ULL) * 1000ULL;

				if (px4_clock_settime(CLOCK_REALTIME, &tv)) {
					PX4_ERR("[timesync] Failed setting realtime clock");
				}
			}

			break;
		}

	default:
		break;
	}
}

uint64_t
MavlinkTimesync::sync_stamp(uint64_t usec)
{
	// Only return synchronised stamp if we have converged to a good value
	if (sync_converged()) {
		return usec + (int64_t)_time_offset;

	} else {
		return hrt_absolute_time();
	}
}

bool
MavlinkTimesync::sync_converged()
{
	return _sequence >= CONVERGENCE_WINDOW;
}

void
MavlinkTimesync::add_sample(int64_t offset_us)
{
	/* Online exponential smoothing filter. The derivative of the estimate is also
	 * estimated in order to produce an estimate without steady state lag:
	 * https://en.wikipedia.org/wiki/Exponential_smoothing#Double_exponential_smoothing
	 */

	double time_offset_prev = _time_offset;

	if (_sequence == 0) {			// First offset sample
		_time_offset = offset_us;

	} else {
		// Update the clock offset estimate
		_time_offset = _filter_alpha * offset_us + (1.0 - _filter_alpha) * (_time_offset + _time_skew);

		// Update the clock skew estimate
		_time_skew = _filter_beta * (_time_offset - time_offset_prev) + (1.0 - _filter_beta) * _time_skew;
	}

}

void
MavlinkTimesync::reset_filter()
{
	// Do a full reset of all statistics and parameters
	_sequence = 0;
	_time_offset = 0.0;
	_time_skew = 0.0;
	_filter_alpha = ALPHA_GAIN_INITIAL;
	_filter_beta = BETA_GAIN_INITIAL;
	_high_deviation_count = 0;
	_high_rtt_count = 0;

}
