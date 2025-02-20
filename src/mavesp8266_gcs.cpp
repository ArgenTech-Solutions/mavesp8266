/****************************************************************************
 *
 * Copyright (c) 2015, 2016 Gus Grubba. All rights reserved.
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
 * @file mavesp8266_gcs.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "mavesp8266.h"
#include "mavesp8266_gcs.h"
#include "mavesp8266_parameters.h"
#include "mavesp8266_component.h"

// in txmod it makes no sense to send RADIO_STATUS as the RFD900x is
// already sending them. Sending them twice just leads to confusing
// and incorrect status
#define RADIO_STATUS_ENABLE 0

//---------------------------------------------------------------------------------
MavESP8266GCS::MavESP8266GCS()
    : _udp_port(DEFAULT_UDP_HPORT)
{
    _recv_chan = MAVLINK_COMM_1;
    _send_chan = MAVLINK_COMM_0;
    memset(&_message, 0, sizeof(_message));
}

//---------------------------------------------------------------------------------
//-- Initialize
void
MavESP8266GCS::begin(MavESP8266Bridge* forwardTo, IPAddress gcsIP)
{
    MavESP8266Bridge::begin(forwardTo);
    _ip = gcsIP;
    //-- Init variables that shouldn't change unless we reboot
    _udp_port = getWorld()->getParameters()->getWifiUdpHport();
    //-- Start UDP
    _udp.begin(getWorld()->getParameters()->getWifiUdpCport());
}

//---------------------------------------------------------------------------------
//-- Read MavLink message from GCS
void
MavESP8266GCS::readMessage()
{
    //-- Read UDP
    if(_readMessage()) {
        //-- If we have a message, forward it
        _forwardTo->sendMessage(&_message);
        memset(&_message, 0, sizeof(_message));
    }
    uint32_t now = millis();
    //-- Update radio status (1Hz)
    if(_heard_from && (now - _last_status_time > 1000)) {
        delay(0);
        _sendRadioStatus();
        _last_status_time = millis();
    }
    if (_sendbuf_ofs > 0 && (now - _send_start_ms > UDP_QUEUE_TIMEOUT || _packets_queued >= 20)) {
        _send_pending();
    }
}


//---------------------------------------------------------------------------------
//-- Read MavLink message from GCS
bool
MavESP8266GCS::_readMessage()
{
    bool msgReceived = false;
    int udp_count = _udp.parsePacket();
    if (udp_count <= 0 && _non_mavlink_len != 0 && _rxstatus.parse_state <= MAVLINK_PARSE_STATE_IDLE) {
        // flush out the non-mavlink buffer when there is nothing pending. This
        // allows us to gather non-mavlink msgs into a single write
        _forwardTo->sendMessageRaw(_non_mavlink_buffer, _non_mavlink_len);
        _non_mavlink_len = 0;
    }
    if(udp_count > 0)
    {
        while(udp_count--)
        {
            int result = _udp.read();
            if (result >= 0)
            {
                // Parsing
                uint8_t last_parse_error = _rxstatus.parse_error;
                msgReceived = mavlink_frame_char_buffer(&_rxmsg,
                                                        &_rxstatus,
                                                        result,
                                                        &_message,
                                                        &_mav_status);
                handle_non_mavlink(result, msgReceived);
                if (last_parse_error != _rxstatus.parse_error) {
                    _status.parse_errors++;                
                }
                if(msgReceived) {
                    //-- We no longer need to broadcast
                    _status.packets_received++;
                    if(_ip[3] == 255) {
                        _ip = _udp.remoteIP();
                        getWorld()->getLogger()->log("Response from GCS. Setting GCS IP to: %s\n", _ip.toString().c_str());
                    }
                    //-- First packets
                    if(!_heard_from) {
                        if(_message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            //-- We no longer need DHCP
                            if(getWorld()->getParameters()->getWifiMode() == WIFI_MODE_AP) {
                                //wifi_softap_dhcps_stop();
                            }
                            _heard_from      = true;
                            _system_id       = _message.sysid;
                            _component_id    = _message.compid;
                            _seq_expected    = _message.seq + 1;
                            _last_heartbeat  = millis();
                        }
                    } else {
                        if(_message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
                            _last_heartbeat = millis();
                        _checkLinkErrors(&_message);
                    }

                    if (msgReceived == MAVLINK_FRAMING_BAD_CRC) {
                        // we don't process messages locally with bad CRC,
                        // but we do forward them, so when new messages
                        // are added we can bridge them
                        break;
                    }

#ifdef MAVLINK_FRAMING_BAD_SIGNATURE
                    if (msgReceived == MAVLINK_FRAMING_BAD_SIGNATURE) {
                        break;
                    }
#endif
                    
                    //-- Check for message we might be interested
                    if(getWorld()->getComponent()->handleMessage(this, &_message)){
                        //-- Eat message (don't send it to FC)
                        memset(&_message, 0, sizeof(_message));
                        msgReceived = false;
                        continue;
                    }


                    //-- Got message, leave
                    break;
                }
            }
        }
    }
    if(!msgReceived) {
        if(_heard_from && (millis() - _last_heartbeat) > HEARTBEAT_TIMEOUT) {
            //-- Restart DHCP and start broadcasting again
            if(getWorld()->getParameters()->getWifiMode() == WIFI_MODE_AP) {
                wifi_softap_dhcps_start();
            }
            _heard_from = false;
            _ip[3] = 255;
            getWorld()->getLogger()->log("Heartbeat timeout from GCS\n");
        }
    }
    return msgReceived;
}

void
MavESP8266GCS::readMessageRaw() {
    int udp_count = _udp.parsePacket();
    char buf[1024];
    int buf_index = 0;

    if(udp_count > 0)
    {
        while(buf_index < udp_count)
        {
            int result = _udp.read();
            if (result >= 0)
            {
                buf[buf_index] = (char)result;
                buf_index++;
            }
        }

        if (buf[0] == 0x30 && buf[1] == 0x20) {
            // reboot command, switch out of raw mode soon
            getWorld()->getComponent()->resetRawMode();
        }

        _forwardTo->sendMessageRaw((uint8_t*)buf, buf_index);
    }
}

//---------------------------------------------------------------------------------
//-- Forward message to the GCS
int
MavESP8266GCS::sendMessage(mavlink_message_t* message) {
    _sendSingleUdpMessage(message);
    return 1;
}

int
MavESP8266GCS::sendMessageRaw(uint8_t *buffer, int len)
{
    _udp.beginPacket(_ip, _udp_port);
    size_t sent = _udp.write(buffer, len);
    _udp.endPacket();
    return sent;
}

//---------------------------------------------------------------------------------
//-- Send Radio Status
void
MavESP8266GCS::_sendRadioStatus()
{
#if RADIO_STATUS_ENABLE
    linkStatus* st = _forwardTo->getStatus();
    uint8_t rssi = 0;
    uint8_t lostVehicleMessages = 100;
    uint8_t lostGcsMessages = 100;

    if(wifi_get_opmode() == STATION_MODE) {
        rssi = (uint8_t)wifi_station_get_rssi();
    }

    if (st->packets_received > 0) {
        lostVehicleMessages = (st->packets_lost * 100) / st->packets_received;
    }

    if (_status.packets_received > 0) {
        lostGcsMessages = (_status.packets_lost * 100) / _status.packets_received;
    }

    //-- Build message
    mavlink_message_t msg;
    mavlink_msg_radio_status_pack_chan(
        _forwardTo->systemID(),
        MAV_COMP_ID_UDP_BRIDGE,
        _forwardTo->_recv_chan,
        &msg,
        rssi,                   // RSSI Only valid in STA mode
        0,                      // We don't have access to Remote RSSI
        100,
        0,                      // We don't have access to noise data
        lostVehicleMessages,    // Percent of lost messages from Vehicle (UART)
        lostGcsMessages,        // Percent of lost messages from GCS (UDP)
        0                       // We don't fix anything
    );

    _sendSingleUdpMessage(&msg);
    _status.radio_status_sent++;
#endif // RADIO_STATUS_SENT
}

//---------------------------------------------------------------------------------
//-- Send UDP Single Message
void
MavESP8266GCS::_sendSingleUdpMessage(mavlink_message_t* msg)
{
    // Translate message to buffer
    char buf[300];
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, msg);
    if (len + _sendbuf_ofs > sizeof(_sendbuf)) {
        _send_pending();
    }
    memcpy(&_sendbuf[_sendbuf_ofs], buf, len);
    if (_sendbuf_ofs == 0) {
        _send_start_ms = millis();
    }
    _packets_queued++;
    _sendbuf_ofs += len;
}

/*
  send pending data in _sendbuf
 */
void MavESP8266GCS::_send_pending(void)
{
    if (_sendbuf_ofs > 0) {
        if (sendMessageRaw(_sendbuf, _sendbuf_ofs) == _sendbuf_ofs) {
            _sendbuf_ofs = 0;
            _status.packets_sent += _packets_queued;
            _packets_queued = 0;
        }
    }
}
