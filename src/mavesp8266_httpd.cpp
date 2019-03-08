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
 * @file mavesp8266_httpd.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "mavesp8266.h"
#include "mavesp8266_httpd.h"
#include "mavesp8266_parameters.h"
#include "mavesp8266_gcs.h"
#include "mavesp8266_vehicle.h"

#include <ESP8266WebServer.h>
#include <FS.h> // spiffs support

#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>

#include <SoftwareSerial.h>

#include "updater.h"  // const char PROGMEM UPDATER[] , allows us to embed a copy of update.htm for the /updatepage

extern SoftwareSerial swSer;

const char PROGMEM kTEXTPLAIN[]  = "text/plain";
const char PROGMEM kTEXTHTML[]   = "text/html";
const char PROGMEM kACCESSCTL[]  = "Access-Control-Allow-Origin";

const char PROGMEM kUPLOADFORM[] = "";
const char PROGMEM kHEADER[]     = "<!doctype html><html><head><title>RFDesign TXMOD</title><link href='/s.css' rel='stylesheet'></head><body><div class='doc'><h1><a href='/'>RFDesign TXMOD</a></h1>";

const char PROGMEM kBADARG[]     = "BAD ARGS";
const char PROGMEM kAPPJSON[]    = "application/json";


const char* kBAUD       = "baud";
const char* kPLAIN       = "plain";
const char* kPWD        = "pwd";
const char* kSSID       = "ssid";
const char* kPWDSTA     = "pwdsta";
const char* kSSIDSTA    = "ssidsta";
const char* kIPSTA      = "ipsta";
const char* kGATESTA    = "gatewaysta";
const char* kSUBSTA     = "subnetsta";
const char* kCPORT      = "cport";
const char* kHPORT      = "hport";
const char* kCHANNEL    = "channel";
const char* kDEBUG      = "debug";
const char* kREBOOT     = "reboot";
const char* kPOSITION   = "position";
const char* kMODE       = "mode";

const char* kFlashMaps[7] = {
    "512KB (256/256)",
    "256KB",
    "1MB (512/512)",
    "2MB (512/512)",
    "4MB (512/512)",
    "2MB (1024/1024)",
    "4MB (1024/1024)"
};

// convert git version and build date to string
#define str(s) #s
#define xstr(s) str(s)
#define GIT_VERSION_STRING xstr(PIO_SRC_REV)
#define BUILD_DATE_STRING xstr(PIO_BUILD_DATE)
#define BUILD_TIME_STRING xstr(PIO_BUILD_TIME)

static uint32_t flash = 0;
static char paramCRC[12] = {""};

ESP8266WebServer    webServer(80);
MavESP8266Update*   updateCB    = NULL;
bool                started     = false;


ESP8266HTTPUpdateServer httpUpdater;


//holds the current upload, if there is one, except OTA binary uploads, done elsewhere.
File fsUploadFile;

//---------------------------------------------------------------------------------
void setNoCacheHeaders() {
    webServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    webServer.sendHeader("Pragma", "no-cache");
    webServer.sendHeader("Expires", "0");
}

//---------------------------------------------------------------------------------
void returnFail(String msg) {
    webServer.send(500, FPSTR(kTEXTPLAIN), msg + "\r\n");
}

//---------------------------------------------------------------------------------
void respondOK() {
    webServer.send(200, FPSTR(kTEXTPLAIN), "OK");
}

//---------------------------------------------------------------------------------
void handle_update() {
    webServer.sendHeader("Connection", "close");
    webServer.sendHeader(FPSTR(kACCESSCTL), "*");
    webServer.send(200, FPSTR(kTEXTHTML), FPSTR(kUPLOADFORM));
}

//---------------------------------------------------------------------------------
void handle_upload() {
    webServer.sendHeader("Connection", "close");
    webServer.sendHeader(FPSTR(kACCESSCTL), "*");
    webServer.send(200, FPSTR(kTEXTPLAIN), (Update.hasError()) ? "FAIL" : "OK");
    if(updateCB) {
        updateCB->updateCompleted();
    }
    //ESP.restart(); lets not do this just here, lets tell the user before we drop out on them... ( we'll let an ajax call to /reboot do it )
}

//---------------------------------------------------------------------------------

// push some notification/s during the firmware update through the soft-serial port...
#define DEBUG_SERIAL swSer

void handle_upload_status() {
    bool success  = true;
    if(!started) {
        started = true;
        if(updateCB) {
            updateCB->updateStarted();
        }
    }
    HTTPUpload& upload = webServer.upload();
    if(upload.status == UPLOAD_FILE_START) {
        #ifdef DEBUG_SERIAL
            //DEBUG_SERIAL.setDebugOutput(true);
        #endif
        WiFiUDP::stopAll();
        Serial.end();
        #ifdef DEBUG_SERIAL
            DEBUG_SERIAL.printf("Update: %s\n", upload.filename.c_str());
        #endif
        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        if(!Update.begin(maxSketchSpace)) {
            #ifdef DEBUG_SERIAL
                Update.printError(DEBUG_SERIAL);
            #endif
            success = false;
        }
    } else if(upload.status == UPLOAD_FILE_WRITE) {
        if(Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            #ifdef DEBUG_SERIAL
                Update.printError(DEBUG_SERIAL);
            #endif
            success = false;
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            #ifdef DEBUG_SERIAL
                DEBUG_SERIAL.printf("Update Success: %u\nReboot Needed.\n", upload.totalSize);
            #endif
              webServer.sendHeader("Location","/success.htm");      // Redirect the client to the success page
              webServer.send(303);
              //delay(2000); // so client can request /success just before we reboot.
        } else {
            #ifdef DEBUG_SERIAL
                Update.printError(DEBUG_SERIAL);
            #endif
            success = false;
            webServer.send(500, "text/plain", "500: couldn't create file");
        }
        #ifdef DEBUG_SERIAL
            //DEBUG_SERIAL.setDebugOutput(false);
        #endif


    }
    yield();
    if(!success) {
        if(updateCB) {
            updateCB->updateError();
        }
    }
}

//---------------------------------------------------------------------------------
void handle_getParameters()
{
    String message = FPSTR(kHEADER);
    message += "<p>TXMOD Parameters</p><table><tr><td width=\"240\">Name</td><td>Value</td></tr>";
    for(int i = 0; i < MavESP8266Parameters::ID_COUNT; i++) {
        message += "<tr><td>";
        message += getWorld()->getParameters()->getAt(i)->id;
        message += "</td>";
        unsigned long val = 0;
        if(getWorld()->getParameters()->getAt(i)->type == MAV_PARAM_TYPE_UINT32)
            val = (unsigned long)*((uint32_t*)getWorld()->getParameters()->getAt(i)->value);
        else if(getWorld()->getParameters()->getAt(i)->type == MAV_PARAM_TYPE_UINT16)
            val = (unsigned long)*((uint16_t*)getWorld()->getParameters()->getAt(i)->value);
        else
            val = (unsigned long)*((int8_t*)getWorld()->getParameters()->getAt(i)->value);
        message += "<td>";
        message += val;
        message += "</td></tr>";
    }
    message += "</table>";
    message += "</body>";
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

String getContentType(String filename) {
  if (webServer.hasArg("download")) {
    return "application/octet-stream";
  } else if (filename.endsWith(".htm")) {
    return "text/html";
  } else if (filename.endsWith(".html")) {
    return "text/html";
  } else if (filename.endsWith(".css")) {
    return "text/css";
  } else if (filename.endsWith(".js")) {
    return "application/javascript";
  } else if (filename.endsWith(".png")) {
    return "image/png";
  } else if (filename.endsWith(".gif")) {
    return "image/gif";
  } else if (filename.endsWith(".jpg")) {
    return "image/jpeg";
  } else if (filename.endsWith(".ico")) {
    return "image/x-icon";
  } else if (filename.endsWith(".xml")) {
    return "text/xml";
  } else if (filename.endsWith(".pdf")) {
    return "application/x-pdf";
  } else if (filename.endsWith(".zip")) {
    return "application/x-zip";
  } else if (filename.endsWith(".gz")) {
    return "application/x-gzip";
  } else if (filename.endsWith(".txt")) {
    return "text/plain";
  } else if (filename.endsWith(".json")) {
    return "application/json";
  }
  return "text/plain";
}

#define DBG_OUTPUT_PORT swSer
	
// by storing all the big files ( especially javascript ) in SPIFFS as .gz, we can save a bunch of space easily.
// but this can present either form to the user
bool handleFileRead(String path) {
  DBG_OUTPUT_PORT.println("handleFileRead: " + path);
  if (path.endsWith("/")) {
    path += "index.htm";
  }
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)) {
    if (SPIFFS.exists(pathWithGz)) {
      path += ".gz";
    }
    File file = SPIFFS.open(path, "r");
    webServer.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}



//---------------------------------------------------------------------------------
static void handle_root()
{

    File cache = SPIFFS.open("/index.cache.htm", "w");

    String message = FPSTR(kHEADER);
    message += "TXMOD Software Version: RFD-";
    char vstr[30];
    snprintf(vstr, sizeof(vstr), "%u.%u.%u", MAVESP8266_VERSION_MAJOR, MAVESP8266_VERSION_MINOR, MAVESP8266_VERSION_BUILD);
    message += vstr;
    message += "<br>\n";
    message += "Git Version: ";
    message += GIT_VERSION_STRING;
    message += "<br>\n";
    message += "Build Date: ";
    message += BUILD_DATE_STRING;
    message += " ";
    message += BUILD_TIME_STRING; 
    message += "<br>\n";

extern String mac_s;
extern String mac_ap_s;

    message += "STA MAC Address:";
    message += mac_s; 
    message += "<br>\n";
    message += "AP MAC Address:";
    message += mac_ap_s; 
    message += "<br>\n";

    cache.print(message); 
    message = "";

extern IPAddress  localIP;

    message += "TXMOD IP Address:";
    message += localIP.toString(); 
    message += "<br>\n";

    cache.print(message); 
    message = "";

    // try to open a version file for the 900x inside the txmod, continue without it anyway.
    File v = SPIFFS.open("/r900x_version.txt", "r");
    if ( v ) { 
        message += "Internal 900x Modem Version: ";
        message += v.readString();
        v.close();
    }
    message += "<br>\n";

    // try to open a version file for the 900x outside the txmod, continue without it anyway.
    File v2 = SPIFFS.open("/r900x_version_remote.txt", "r");
    if ( v2 ) { 
        message += "External 900x Modem Version: ";
        message += v2.readString();
        v2.close();
    }
    message += "<br>\n";

extern String realSize;

    if ( realSize == "4194304" ) { 
        message += "Your Flash-Size is 4M.<p>\n";
    }

    if ( realSize == "2097152" ) { 
        message += "Your Flash-Size is 2M.<p>\n";
    }
    if (( realSize != "2097152" ) && ( realSize != "4194304" )) { 
        message += "Your Flash Size is weird.  It's not 2Meg or 4Meg. Reported Size: "+realSize+"<p>\n";
    }

    message += "<p>\n";

    cache.print(message); 
    message = "";



    // if we have an index.html in spiffs, build a cahce based on that, otherwise use a basic version that's included below.
    // CAUTION: this cache to spiffs is becasue of a breakdown if the resulting index.htm we send is more than abut 6k in size
    // but could be made to work on bigger files if we write the above to spiffs as (say) index.cache.htm, then 
    //sent the result with handleFileRead("/index.cache.htm"). 
    File f = SPIFFS.open("/index.htm", "r");
    if ( f ) { 
        message += f.readString();
        f.close();
    } 

    cache.print(message); 
    message = "";

    cache.close(); // close index.cache.htm

 
    // try to render /index.cache as-is, otherwise fallback to this more minimal static version... 
    if (!handleFileRead("/index.cache.htm")) {
    
        String message = FPSTR(kHEADER);
        message += "<ul>\n";
        message += "<li><a href='/getstatus'>Get Status ( UDP/Mavlink mode )</a>\n";
        message += "<li><a href='/getstatus_tcp'>Get Status (TCP/passthrough mode )</a>\n";
        message += "<li><a href='/setup'>View/Edit WiFi/Network Setup</a>\n";
        message += "<li><a href='/plist'>View/Edit 900x Radio Parameters</a>\n";
        message += "<li><a href='/updatepage'>Update Firmware</a>\n";
        message += "<li><a href='/reboot'>Reboot</a>\n";
        message += "<li><a href='/edit'>Advanced Mode -  Review and Edit (some) files in the SPIFFS filesystem.</a>";
        message += "</ul>\n";
        message += "<hr>\n";
        message += "<h2>Documentation</h2>\n";
        message += "(requires internet access)<br>\n";
        message += "<ul>\n";
        message += "<li><a href='http://ardupilot.org'>ArduPilot Website</a>\n";
        message += "<li><a href='http://ardupilot.org/copter/docs/common-esp8266-telemetry.html'>ESP8266 WiFi Documentation</a>\n";
        message += "<li><a href='https://github.com/RFDesign/mavesp8266'>RFDesign ESP8266 Source Code</a>\n";
        message += "<li><a href='http://files.rfdesign.com.au/firmware/'>RFDesign TXMOD Firmware Updates</a>\n";
        message += "</ul>\n";
        message += "</div></body></html>";

        setNoCacheHeaders();
        webServer.send(200, FPSTR(kTEXTHTML), message);
    }

}


static void handle_setup_adv(String more)
{
    String message = FPSTR(kHEADER);
    message += "<h1>Wifi Setup</h1>\n";
    message += more;
    message += "<form action='/setparameters' method='post' onsubmit=\"p=document.getElementById('pwd'); if ( p.value.length < 8 ){p.value='toshort';return false;} return true;\" >\n";
    message += "WiFi Mode:&nbsp;";
    message += "<input type='radio' name='mode' value='0'";
    if (getWorld()->getParameters()->getWifiMode() == WIFI_MODE_AP) {
        message += " checked";
    }
    message += ">AccessPoint\n";
    message += "<input type='radio' name='mode' value='1'";
    if (getWorld()->getParameters()->getWifiMode() == WIFI_MODE_STA) {
        message += " checked";
    }
    message += ">Station<br>\n";
    
    message += "AP SSID:&nbsp;";
    message += "<input type='text' name='ssid' value='";
    message += getWorld()->getParameters()->getWifiSsid();
    message += "'><br>";

    message += "AP Password (min len 8):&nbsp;";
    message += "<input type='text' name='pwd' id='pwd' value='";
    message += getWorld()->getParameters()->getWifiPassword();
    message += "'><br>";

    message += "WiFi Channel:&nbsp;";
    message += "<input type='text' name='channel' value='";
    message += getWorld()->getParameters()->getWifiChannel();
    message += "'><br>";

    message += "Station SSID:&nbsp;";
    message += "<input type='text' name='ssidsta' value='";
    message += getWorld()->getParameters()->getWifiStaSsid();
    message += "'><br>";

    message += "Station Password:&nbsp;";
    message += "<input type='text' name='pwdsta' value='";
    message += getWorld()->getParameters()->getWifiStaPassword();
    message += "'><br>";

    IPAddress IP;    
    message += "Station IP:&nbsp;";
    message += "<input type='text' name='ipsta' value='";
    IP = getWorld()->getParameters()->getWifiStaIP();
    message += IP.toString();
    message += "'><br>";

    message += "Station Gateway:&nbsp;";
    message += "<input type='text' name='gatewaysta' value='";
    IP = getWorld()->getParameters()->getWifiStaGateway();
    message += IP.toString();
    message += "'><br>";

    message += "Station Subnet:&nbsp;";
    message += "<input type='text' name='subnetsta' value='";
    IP = getWorld()->getParameters()->getWifiStaSubnet();
    message += IP.toString();
    message += "'><br>";

    message += "Host Port:&nbsp;";
    message += "<input type='text' name='hport' value='";
    message += getWorld()->getParameters()->getWifiUdpHport();
    message += "'><br>";

    message += "Client Port:&nbsp;";
    message += "<input type='text' name='cport' value='";
    message += getWorld()->getParameters()->getWifiUdpCport();
    message += "'><br>";
    
    message += "Baudrate:&nbsp;";
    message += "<input type='text' name='baud' value='";
    message += getWorld()->getParameters()->getUartBaudRate();
    message += "'><br>";
    
    message += "<input type='submit' value='Save' onclick="">";
    message += "</form>";
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

//---------------------------------------------------------------------------------
static void handle_setup()
{
    return handle_setup_adv("");
}
//---------------------------------------------------------------------------------
static void handle_getStatus()
{
    if(!flash)
        flash = ESP.getFreeSketchSpace();
    if(!paramCRC[0]) {
        snprintf(paramCRC, sizeof(paramCRC), "%08X", getWorld()->getParameters()->paramHashCheck());
    }
    linkStatus* gcsStatus = getWorld()->getGCS()->getStatus();
    linkStatus* vehicleStatus = getWorld()->getVehicle()->getStatus();
    String message = FPSTR(kHEADER);

extern bool tcp_passthrumode;

    if ( tcp_passthrumode == false ) { 
    message += "<font color=green>Device is currently In UDP(mavlink) mode right now.</font><br>\n";
    } else { 
    message += "<font color=red>NOT In UDP(mavlink) mode right now</font><br>\n";
    }

    message += "<font color=green>To connect your GCS in UDP(mavlink) mode, please open a UDP port on port number:";
    message += getWorld()->getParameters()->getWifiUdpHport();
    message += "</font><br></p>\n";

    message += "<p>Comm Status</p><table><tr><td width=\"240\">Packets Received from GCS</td><td>";


    message += gcsStatus->packets_received;
    message += "</td></tr><tr><td>Packets Sent to GCS</td><td>";
    message += gcsStatus->packets_sent;
    message += "</td></tr><tr><td>GCS Packets Lost</td><td>";
    message += gcsStatus->packets_lost;
    message += "</td></tr><tr><td>GCS Parse Errors</td><td>";
    message += gcsStatus->parse_errors;
    message += "</td></tr><tr><td>Packets Received from Vehicle</td><td>";
    message += vehicleStatus->packets_received;
    message += "</td></tr><tr><td>Packets Sent to Vehicle</td><td>";
    message += vehicleStatus->packets_sent;
    message += "</td></tr><tr><td>Vehicle Packets Lost</td><td>";
    message += vehicleStatus->packets_lost;
    message += "</td></tr><tr><td>Vehicle Parse Errors</td><td>";
    message += vehicleStatus->parse_errors;
    message += "</td></tr><tr><td>Radio Messages</td><td>";
    message += gcsStatus->radio_status_sent;
    message += "</td></tr></table>";
    message += "<p>System Status</p><table>\n";
    message += "<tr><td width=\"240\">Flash Size</td><td>";
    message += ESP.getFlashChipRealSize();
    message += "</td></tr>\n";
    message += "<tr><td width=\"240\">Flash Available</td><td>";
    message += flash;
    message += "</td></tr>\n";
    message += "<tr><td>RAM Left</td><td>";
    message += String(ESP.getFreeHeap());
    message += "</td></tr>\n";
    message += "<tr><td>Parameters CRC</td><td>";
    message += paramCRC;
    message += "</td></tr>\n";
    message += "</table>";
    message += "</body>";
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

//---------------------------------------------------------------------------------
static void handle_getStatusTcp()
{
    if(!flash)
        flash = ESP.getFreeSketchSpace();

    String message = FPSTR(kHEADER);

// kinda hacky to use extern global vars here, but it'll do for now.
extern long int stats_serial_in;
extern long int stats_tcp_in;
extern long int stats_serial_pkts;
extern long int stats_tcp_pkts;
//extern long int largest_serial_packet;
//extern long int largest_tcp_packet;
extern bool tcp_passthrumode;

    message += "<p>TCP Comms Status - last 1 second of TCP throughput.</p><br>";

    if ( tcp_passthrumode == true ) { 
    message += "<font color=green>Currently In TCP pass-through mode right now.</font><br>\n";
    } else { 
    message += "<font color=red>NOT In TCP pass-through mode right now</font><br>\n";
    }

    message += "<font color=green>To connect your GCS in TCP mode, please connect as a TCP client to IP: 192.168.4.1, with port number 23.</font><br></p>\n";

    message += "<table><tr><td width=\"240\">TCP Bytes Received from GCS</td><td>";
    message += stats_tcp_in;
    message += "</td></tr>";

    message += "<tr><td>TCP Packets Sent to GCS</td><td>";
    message += stats_tcp_pkts;
    message += "</td></tr>";

    message += "<tr><td>Serial Bytes Received from Vehicle</td><td>";
    message += stats_serial_in;
    message += "</td></tr>";

    message += "<tr><td>Serial Packets Sent to Vehicle</td><td>";
    message += stats_serial_pkts;
    message += "</td></tr></table>";

    message += "<p>System Status</p><table>\n";
    message += "<tr><td width=\"240\">Flash Size</td><td>";
    message += ESP.getFlashChipRealSize();
    message += "</td></tr>\n";
    message += "<tr><td width=\"240\">Flash Available</td><td>";
    message += flash;
    message += "</td></tr>\n";
    message += "<tr><td>RAM Left</td><td>";
    message += String(ESP.getFreeHeap());
    message += "</td></tr>\n";

    message += "</table>";
    message += "</body>";
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

//---------------------------------------------------------------------------------
void handle_getJLog()
{
    uint32_t position = 0, len;
    if(webServer.hasArg(kPOSITION)) {
        position = webServer.arg(kPOSITION).toInt();
    }
    String logText = getWorld()->getLogger()->getLog(&position, &len);
    char jStart[128];
    snprintf(jStart, 128, "{\"len\":%d, \"start\":%d, \"text\": \"", len, position);
    String payLoad = jStart;
    payLoad += logText;
    payLoad += "\"}";
    webServer.send(200, FPSTR(kAPPJSON), payLoad);
}

//---------------------------------------------------------------------------------
void handle_getJSysInfo()
{
    if(!flash)
        flash = ESP.getFreeSketchSpace();
    if(!paramCRC[0]) {
        snprintf(paramCRC, sizeof(paramCRC), "%08X", getWorld()->getParameters()->paramHashCheck());
    }
    uint32_t fid = spi_flash_get_id();
    char message[512];
    snprintf(message, 512,
        "{ "
        "\"size\": \"%s\", "
        "\"id\": \"0x%02lX 0x%04lX\", "
        "\"flashfree\": \"%u\", "
        "\"heapfree\": \"%u\", "
        "\"logsize\": \"%u\", "
        "\"paramcrc\": \"%s\""
        " }",
        kFlashMaps[system_get_flash_size_map()],
        (long unsigned int)(fid & 0xff), (long unsigned int)((fid & 0xff00) | ((fid >> 16) & 0xff)),
        flash,
        ESP.getFreeHeap(),
        getWorld()->getLogger()->getPosition(),
        paramCRC
    );
    webServer.send(200, "application/json", message);
}

//---------------------------------------------------------------------------------
void handle_getJSysStatus()
{
    bool reset = false;
    if(webServer.hasArg("r")) {
        reset = webServer.arg("r").toInt() != 0;
    }
    linkStatus* gcsStatus = getWorld()->getGCS()->getStatus();
    linkStatus* vehicleStatus = getWorld()->getVehicle()->getStatus();
    if(reset) {
        memset(gcsStatus,     0, sizeof(linkStatus));
        memset(vehicleStatus, 0, sizeof(linkStatus));
    }
    char message[512];
    snprintf(message, 512,
        "{ "
        "\"gpackets\": \"%u\", "
        "\"gsent\": \"%u\", "
        "\"glost\": \"%u\", "
        "\"vpackets\": \"%u\", "
        "\"vsent\": \"%u\", "
        "\"vlost\": \"%u\", "
        "\"radio\": \"%u\", "
        "\"buffer\": \"%u\""
        " }",
        gcsStatus->packets_received,
        gcsStatus->packets_sent,
        gcsStatus->packets_lost,
        vehicleStatus->packets_received,
        vehicleStatus->packets_sent,
        vehicleStatus->packets_lost,
        gcsStatus->radio_status_sent,
        vehicleStatus->queue_status
    );
    webServer.send(200, "application/json", message);
}



// see main.cpp
extern int r900x_savesingle_param_and_verify(String prefix, String ParamID, String ParamVAL); 
extern int r900x_savesingle_param_and_verify_more(String prefix, String ParamID, String ParamVAL, bool save_and_reboot); 
extern int r900x_readsingle_param(String prefix, String ParamID);

//---------------------------------------------------------------------------------

// returns 200, 201, or 202
int wiz_param_helper( String ParamID, String ParamVAL, bool save_and_reboot  ) { 
        
        // activate remote 900x Network Channel (ID)  and verify  S3:NETID=
        int retval =    r900x_savesingle_param_and_verify_more("RT", ParamID, ParamVAL, save_and_reboot);
        int retval2 = 0;
        if ( retval > 0 ) {  //if remote verified, then activate local 900x Network Channel (ID)  and verify
           retval2 = r900x_savesingle_param_and_verify_more("AT", ParamID, ParamVAL, save_and_reboot);
           // if it failed, retry, because the remote already succeeded and the local should too.
           if ( retval2 < 0 ) { 
            retval2 = r900x_savesingle_param_and_verify_more("AT", ParamID, ParamVAL, save_and_reboot);
           }
           if ( retval2 > 0 ) { 
              return 200;
           }
        }
        // saving to remote failed, advise of that as a priority       
        if ( retval < 0 ) {
            return 201;
        }
        // saving to local failed, advise of that otherwise.         
        if ( retval2 < 0 ) {
            return 202;
        }
} 
//---------------------------------------------------------------------------------
// tries to save to remote first, and if that works, tries twice to save to local.
// in any case, it returns HTTP codes 200 for success, and 201 for remote fail, and 202 for local fail.
void wiz_param_saver( String ParamID, String ParamVAL ) { 

    // returns 200, 201, or 202
    int retval = wiz_param_helper( ParamID, ParamVAL, true);  // true means save-and-reboot after setting param

    String message = "";

    // saving to remote failed, advise of that        
    if ( retval == 201 ) {
        swSer.println("B2");
        message = "FAIL param saving to REMOTE 900x radio. RT ";
        message += ParamID+"->"+ParamVAL;
        setNoCacheHeaders();
        webServer.send(201, FPSTR(kTEXTHTML), message);
        return;
    }
    // saving to local failed, advise of that          
    if ( retval == 202 ) {
        swSer.println("C2");
        message = "FAIL param saving to LOCAL 900x radio. AT ";
        message += ParamID+"->"+ParamVAL;
        setNoCacheHeaders();
        webServer.send(202, FPSTR(kTEXTHTML), message);
        return;
    } 
    //if ( retval == 200 ) {
        message = "SUCCESS param saving to REMOTE & LOCAL 900x radio. AT ";
        message += ParamID+"->"+ParamVAL;
        swSer.println("E2");
        setNoCacheHeaders();
        swSer.println(message);
        webServer.send(200, FPSTR(kTEXTHTML), message);
        return;
    //}

}


//---------------------------------------------------------------------------------
// for /wiz url
void handle_wiz_save() // accept updated param/s via POST, save them, then display standard 'edit' form with new data.
{
    if(webServer.args() == 0) {
        // no args, just return 200/ok
        webServer.send(200, FPSTR(kTEXTHTML), "ok");
        return;
    }
    bool ok = false;

    //localC: 148
    //localE: d81eaaa5c263080
    //localPPMin: 1
    //remotePPMOUT: 1
    //remoteDefaultPPM: 1
    //confirmtest: Yes

    String message = "";  // ultimately will be output to user
    //String message = FPSTR(kHEADER);

    int page = -1;
    if(webServer.hasArg("page")) {
        //ok = true;
        page = webServer.arg("page").toInt();
    }
    if(page == 0) { // page=0 called on the first-load of the first page in the wizard, before any user input, so we do nothing here but succeed
        ok = true;
    }

    // link check
    if(page == 1) { // page=1 called on the 'Next' from the first page, where we actually input nothing, so we do nothing here but succeed
        //ok = true;

        // test if the remote radio is present by trying to read a parameter from it...
        int val = r900x_readsingle_param("RT", "S0");
        int retval = 200; // 200=success, 201 = fail.
        if (val >= 35) { // success
            message = "SUCCESS param reading from REMOTE 900x radio. RT S0->"+String(val);
        } else { 
            message = "FAIL param reading from REMOTE 900x radio. RT S0->"+String(val);
            retval = 201;
        }
        setNoCacheHeaders();
        webServer.send(retval, FPSTR(kTEXTHTML), message);
        return;

    }    
    //NETID
    if((page == 2) && webServer.hasArg("localC")) {
        //ok = true;
        int w1 = webServer.arg("localC").toInt(); // convert to int and back removes any whitespace and \r\n etc.
        String Sw1 = String(w1);

        // disable encryption first and reboot 
        int retval = wiz_param_helper( "S15", "0" , true); 
		delay(500);
        if (retval != 200 ) {
            message += "FAILED to set S15=0 on one or more of the radio/s.";
		    setNoCacheHeaders();
		    webServer.send(retval, FPSTR(kTEXTHTML), message);
		    return;
        }

		// factory default the radio/s to (a) put them in a good state, and (b) clear any encryption key for later...
        // disable encryption via factory default and reboot
        retval = wiz_param_helper( "&F", "" , true); 
		delay(500);
        if (retval != 200 ) {
            message += "FAILED to factory-default radio/s with RT&F/AT&F on one or more of the radio/s.";
		    setNoCacheHeaders();
		    webServer.send(retval, FPSTR(kTEXTHTML), message);
		    return;
        }

		// after a factory-default, THEN set the netid in both radio/s as the first step in the process.
        wiz_param_saver( "S3", Sw1 ); // non-returning, emits http response, saves and reboots radios after
        
    }

    // ENCRYPTION
    if((page == 3) && webServer.hasArg("localE")) {
        //ok = true;

        String w2 = webServer.arg("localE").c_str();
        int EE = webServer.arg("localEE").toInt(); // convert to int and back removes any whitespace and \r\n etc.

        int retval = -1;
        int retval2 = -1;
        int retval3 = -1;

        // first test if the remote radio is present by trying to read a parameter from it...
        int val = r900x_readsingle_param("RT", "S0");
        bool linktest = false;
        if (val >= 35) { // success
            //message = "remote link tested OK";
            linktest = true;
        } else { 
            message = "FAILED link test to REMOTE 900x radio, check settings. RT S0->"+String(val);
        }

        // don't try to enable encryption if link didn't test OK.
        if ( linktest ) {

            // disable encryption
            if ( EE == 0 ){ 

                   wiz_param_saver( "S15", "0" ); // non-returning, emits http response, saves and reboots radios after
            }

            // enable encryption - warning, a prerequisite to this succeeding is that u MUST have disabled 
            // encryption with S15=0 and a reboot before doing this.
            if ( EE == 1 ){ 

                 // cache last-used encryption key to spiffs to display later if needed
                 
                if (w2.length() > 30 ) { // basic check, file should exist and have at least 30 bytes in it to be plausible

                     File encfile = SPIFFS.open("/key.txt", "w");
                     encfile.println(w2);
                     encfile.close();
                     swSer.println("Wrote Enc Key to /key.txt");

                    // enable encryption first but don't reboot
                    retval = wiz_param_helper( "S15", "1" , false); 
                    if (retval != 200 ) {
                        message += "FAILED to set S15=1 on one or more of the radio/s.";
                    }

                    // set key with save and  reboot
                    if (retval == 200 ) {  // check retval and continue to enable encryption if asked for.
                        wiz_param_saver( "&E", w2 );
                    } 


                } else {
                    message = "FAILED requested encryption key too short. ";
                }
            }

        }

        message += "FAILED problem executing request for encryption enable/disable.";
        setNoCacheHeaders();
        webServer.send(203, FPSTR(kTEXTHTML), message);
        return;

    }
    // PPMIN and PPMOUT two on same page (4):
    if((page == 4) && webServer.hasArg("localPPMin")) {
        //ok = true;
        int w3 = webServer.arg("localPPMin").toInt();
        int w4 = webServer.arg("remotePPMOUT").toInt();

        int retval = -1;
        int retval2 = -1;
        //int retval3 = -1;

        //ATS16=1
        //RTS17=1
        // and while we are at it, enable local STAT LED:
        //ATS19=1 

        // ... also , side effect, set ATS19=1 but ignore all its output and don't reboot.
        r900x_savesingle_param_and_verify_more("AT", "S19", "1", false);

        // activate remote 900x Network Channel (ID)  and verify  S3:NETID=
        retval =    r900x_savesingle_param_and_verify_more("RT", "S17", String(w4), true);
        if ( retval < 0 ) {  //if remote verified, then activate local 900x Network Channel (ID)  and verify
                message += "FAILED param saving to REMOTE 900x radio. RT S17->"+String(w4);
                setNoCacheHeaders();
                swSer.println(message);
                webServer.send(201, FPSTR(kTEXTHTML), message);
                return;
        }

        retval2 = r900x_savesingle_param_and_verify_more("AT", "S16", String(w3), true);
        // if it failed, retry, because the remote already succeeded and the local should too.
        if ( retval2 < 0 ) { 
        retval2 = r900x_savesingle_param_and_verify_more("AT", "S16", String(w3), true);
        }
        if ( retval2 < 0 ) {  //if remote verified, then activate local 900x Network Channel (ID)  and verify
                message += "FAILED param saving to LOCAL 900x radio. AT S16->"+String(w3);
                setNoCacheHeaders();
                swSer.println(message);
                webServer.send(202, FPSTR(kTEXTHTML), message);
                return;
        }
       
        // success, as both retval/s are now > 0
        if ((retval > 0 ) && (retval2 > 0 )) { 
            message = "SUCCESS param saving to LOCAL/REMOTE 900x radio. AT/RT S16/17->";
            setNoCacheHeaders();
            swSer.println(message);
            webServer.send(200, FPSTR(kTEXTHTML), message);
            return;
        }

      
    }


    if((page == 5) && webServer.hasArg("remoteDefaultPPM")) {
        ok = true;
        int w5 = webServer.arg("remoteDefaultPPM").toInt();

        if ( w5 == 1 ) { 

             // send command to remote radio:
             //RT&R
            // Record default PPM stream for PPM output (vehicle side)
            // activate PPM failsave channel settings on remote radio:
            int retval =    r900x_savesingle_param_and_verify_more("RT", "&R", "", false);
            if ( retval < 0 ) {  //if remote verified, then activate local 900x Network Channel (ID)  and verify
                    message += "FAILED param saving to REMOTE 900x radio. RT &R";
                    setNoCacheHeaders();
                    swSer.println(message);
                    webServer.send(201, FPSTR(kTEXTHTML), message);
                    return;
            }
        }

    }
    if((page == 6) && webServer.hasArg("confirmtest")) {
        ok = true;
        String w6 = webServer.arg("confirmtest").c_str();
    }

    // the actual final form submit doesn't know its page number, but should succeed as it has all other data.
    if(page == -1) {  

        message = "Thanks, your Wizard is Completed, and all Setting Saved.";

    }  

    swSer.println("D");


    if(ok) {
    message += "<font color=green>WIZARD page SETTINGS are Saved to EEPROM!</font>";
    }
    swSer.println("E");
    setNoCacheHeaders();
    swSer.println(message);
    webServer.send(200, FPSTR(kTEXTHTML), message);
    //} else {
    //    returnFail(kBADARG);
    //}
}

//---------------------------------------------------------------------------------
void handle_setParameters() // accept updated param/s via POST, save them, then display standard 'edit' form with new data.
{
    if(webServer.args() == 0) {
        returnFail(kBADARG);
        return;
    }
    bool ok = false;
    bool reboot = false;
    if(webServer.hasArg(kBAUD)) {
        ok = true;
        getWorld()->getParameters()->setUartBaudRate(webServer.arg(kBAUD).toInt());
    }
    if(webServer.hasArg(kPWD)) {
        ok = true;
        getWorld()->getParameters()->setWifiPassword(webServer.arg(kPWD).c_str());
    }
    if(webServer.hasArg(kSSID)) {
        ok = true;
        getWorld()->getParameters()->setWifiSsid(webServer.arg(kSSID).c_str());
    }
    if(webServer.hasArg(kPWDSTA)) {
        ok = true;
        getWorld()->getParameters()->setWifiStaPassword(webServer.arg(kPWDSTA).c_str());
    }
    if(webServer.hasArg(kSSIDSTA)) {
        ok = true;
        getWorld()->getParameters()->setWifiStaSsid(webServer.arg(kSSIDSTA).c_str());
    }
    if(webServer.hasArg(kIPSTA)) {
        IPAddress ip;
        ip.fromString(webServer.arg(kIPSTA).c_str());
        getWorld()->getParameters()->setWifiStaIP(ip);
    }
    if(webServer.hasArg(kGATESTA)) {
        IPAddress ip;
        ip.fromString(webServer.arg(kGATESTA).c_str());
        getWorld()->getParameters()->setWifiStaGateway(ip);
    }
    if(webServer.hasArg(kSUBSTA)) {
        IPAddress ip;
        ip.fromString(webServer.arg(kSUBSTA).c_str());
        getWorld()->getParameters()->setWifiStaSubnet(ip);
    }
    if(webServer.hasArg(kCPORT)) {
        ok = true;
        getWorld()->getParameters()->setWifiUdpCport(webServer.arg(kCPORT).toInt());
    }
    if(webServer.hasArg(kHPORT)) {
        ok = true;
        getWorld()->getParameters()->setWifiUdpHport(webServer.arg(kHPORT).toInt());
    }
    if(webServer.hasArg(kCHANNEL)) {
        ok = true;
        getWorld()->getParameters()->setWifiChannel(webServer.arg(kCHANNEL).toInt());
    }
    if(webServer.hasArg(kDEBUG)) {
        ok = true;
        getWorld()->getParameters()->setDebugEnabled(webServer.arg(kDEBUG).toInt());
    }
    if(webServer.hasArg(kMODE)) {
        ok = true;
        getWorld()->getParameters()->setWifiMode(webServer.arg(kMODE).toInt());
    }
    if(webServer.hasArg(kREBOOT)) {
        ok = true;
        reboot = webServer.arg(kREBOOT) == "1";
    }
    if(ok) {
        getWorld()->getParameters()->saveAllToEeprom();
        //-- Send new parameters back
        //handle_getParameters();
        handle_setup_adv("<font color=green>Params are Saved to EEPROM!</font>");
        if(reboot) {
            delay(100);
            ESP.restart();
        }
    } else
        returnFail(kBADARG);
}

//---------------------------------------------------------------------------------
static void handle_reboot()
{
    String message = FPSTR(kHEADER);
    message += "rebooting ...</body>\n";
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
    delay(500);
    ESP.restart();    
}

//---------------------------------------------------------------------------------
//-- 404
/* static void handle_notFound(){
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += webServer.uri();
    message += "\nMethod: ";
    message += (webServer.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += webServer.args();
    message += "\n";
    for (uint8_t i = 0; i < webServer.args(); i++){
        message += " " + webServer.argName(i) + ": " + webServer.arg(i) + "\n";
    }
    webServer.send(404, FPSTR(kTEXTPLAIN), message);
} */




extern bool r900x_saveparams(String file); // its in main.cpp

/*
void save900xparams() { 

    String message = FPSTR(kHEADER);
    if (r900x_saveparams("/r900x_params.txt") ) { 
        message += "900x radio params saved to modem OK. ";
    } else { 
        message += "900x radio params save FAILED. Does file /r900x_params.txt exist? ";
    }
    message =+ "<a href=/>Click here to continue.</a></body></html>";
    webServer.send(200, FPSTR(kTEXTHTML), message);
} 
*/



// also getting below 7k or 8k or RAM hurts some form of file download, but not this, aparently:
// https://github.com/esp8266/Arduino/issues/3205

//TODO we should consider "use PROGMEM and server.send_P to send data from PROGMEM — in this case it doesn't need to be copied to RAM twice, you avoid allocating Strings and all the associated issues"

void send_favicon() { 
    // stored in PROGMEM due to the 'const':
    const char favicon[] = {
      0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10, 0x10, 0x00, 0x00, 0x01, 0x00,
      0x18, 0x00, 0x68, 0x03, 0x00, 0x00
    }; 

    webServer.send_P(200, "image/x-icon", favicon, sizeof(favicon));
}



//---------------------------------------------------------------------------------
static void handle_update_html()
{

    // try to render external /update.htm as-is, otherwise fallback to the embedded copy. 
    if (!handleFileRead("/update.htm")) {

        String message =  FPSTR(UPDATER);
        webServer.send(200, FPSTR(kTEXTHTML), message);
    }

}

void handleFileUpload() {
    //DBG_OUTPUT_PORT.println("handleFileUpload 0: "); 
  if (webServer.uri() != "/edit") {
    return;
  }

  HTTPUpload& upload = webServer.upload();
  DBG_OUTPUT_PORT.print("status:"); DBG_OUTPUT_PORT.println(upload.status);
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (!filename.startsWith("/")) {
      filename = "/" + filename;
    }
    DBG_OUTPUT_PORT.print("handleFileUpload Name: "); DBG_OUTPUT_PORT.println(filename);

    // if its a firmware .bin file we are about to upload, remove the .in.ok first
    // .. as in a small 2M SPIFFS we really don't have room for more than 1 of these
    if (filename == BOOTLOADERNAME ) { 
        SPIFFS.remove(BOOTLOADERNAME); // cleanup incase an old one is still there. 
        SPIFFS.remove(BOOTLOADERCOMPLETE); // cleanup incase an old one is still there. 
    }

    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    DBG_OUTPUT_PORT.print("handleFileUpload Data: "); DBG_OUTPUT_PORT.println(upload.currentSize);
    if (fsUploadFile) {
      fsUploadFile.write(upload.buf, upload.currentSize);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (fsUploadFile) {
      fsUploadFile.close();
      DBG_OUTPUT_PORT.print("handleFileUpload Size: "); DBG_OUTPUT_PORT.println(upload.totalSize);
      webServer.sendHeader("Location","/success.htm");      // Redirect the client to the success page
      webServer.send(303);
    } else { 
      webServer.send(500, "text/plain", "500: couldn't create file");
    }
    //DBG_OUTPUT_PORT.print("handleFileUpload Size: "); DBG_OUTPUT_PORT.println(upload.totalSize);
  }
  //webServer.send(200, "text/plain", "");
}

// /plist
void handle900xParamList() { 

    File html = SPIFFS.open("/r900x_params.htm", "r");
 
    html.setTimeout(50); // don't wait long as it's a file object, not a serial port.
 
    if ( ! html ) {  // did we open this file ok, ie does it exist? 
        swSer.println("no /r900x_params.htm exists, can't display modem settings properly.\n");
        webServer.send(200, "text/html", "ERR,couldn't do it - /r900x_params.htm is missing from spiffs."); return;
    }


    String contentType = getContentType("/r900x_params.htm");
    webServer.streamFile(html, contentType);
    html.close();

    return;
}

// do AT and RT commands to get 
extern int r900x_getparams(String filename,bool factory_reset_first); // see main.cpp
extern void r900x_setup(bool refresh);

// /prefresh
void handle900xParamRefresh() { 

    String type = webServer.arg("type");

    String factory = webServer.arg("factory");
    bool f = false;
    // a factory-default of the 900 radios is actually a r900x_getparams() request with the 2nd "factory_reset_first" parameter true
    if ( factory == "yes" ) {  f = true; }

    int num_read = 0;
    if ( type == "local" ) { 
    num_read = r900x_getparams("/r900x_params.txt",f);
    }
    if ( type == "remote" ) { 
    num_read = r900x_getparams("/r900x_params_remote.txt",f);
    }
    if ( type == "setup" ) { 
    r900x_setup(false);
    }
    if ( type == "reflash" ) { 
    r900x_setup(true);
    }

    //todo would this be better sent as json ? 
    //var mydata = JSON.parse(data);
    //type = "handle900xParamRefresh() executed. type:"+type+" num_read:"+num_read;
    type = "{ \"request\":\"paramrefresh\", \"type\":\""+type+"\", \"num_read\":"+num_read+"}"; // valid json to browser

    //webServer.send(200, "text/plain", type );
     webServer.send(200, "application/json", type ); 



} 


// spiffs files  goto url:  http://192.168.4.1/list?dir=/
void handleFileList() {
  if (!webServer.hasArg("dir")) {
    webServer.send(500, "text/plain", "BAD ARGS");
    return;
  }

  String path = webServer.arg("dir");
  DBG_OUTPUT_PORT.println("handleFileList: " + path);
  Dir dir = SPIFFS.openDir(path);
  path = String();

  String output = "[";
  while (dir.next()) {
    File entry = dir.openFile("r");
    if (output != "[") {
      output += ',';
    }
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir) ? "dir" : "file";
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(1);
    output += "\"}";
    entry.close();
  }

  output += "]";
  webServer.send(200, "text/json", output);
}

void handleFileDelete() {
  if (webServer.args() == 0) {
    return webServer.send(500, "text/plain", "BAD ARGS");
  }
  String path = webServer.arg(0);
  DBG_OUTPUT_PORT.println("handleFileDelete: " + path);
  if (path == "/") {
    return webServer.send(500, "text/plain", "BAD PATH");
  }
  if (!SPIFFS.exists(path)) {
    return webServer.send(404, "text/plain", "FileNotFound");
  }
  SPIFFS.remove(path);
  webServer.send(200, "text/plain", "");
  path = String();
}

void handleFileCreate() {
  if (webServer.args() == 0) {
    return webServer.send(500, "text/plain", "BAD ARGS");
  }
  String path = webServer.arg(0);
  DBG_OUTPUT_PORT.println("handleFileCreate: " + path);
  if (path == "/") {
    return webServer.send(500, "text/plain", "BAD PATH");
  }
  if (SPIFFS.exists(path)) {
    return webServer.send(500, "text/plain", "FILE EXISTS");
  }
  File file = SPIFFS.open(path, "w");
  if (file) {
    file.close();
  } else {
    return webServer.send(500, "text/plain", "CREATE FAILED");
  }
  webServer.send(200, "text/plain", "");
  path = String();
}

void handle900xParamSave() { 
  if (webServer.args() == 0) {
    return webServer.send(500, "text/plain", "BAD ARGS");
  }
  //String p = webServer.arg('params');
   // DBG_OUTPUT_PORT.println("handle900xParamSave: " + p);


  String message = "";
  for (uint8_t i = 0; i < webServer.args(); i++) {
    message += " " + webServer.argName(i) + " -> " + webServer.arg(i) + "\n";
  }

  int retval = 404; // if its good, then some sort of 200 message, depending on what happens.

  File f;
  String filename = "/r900x_params.txt"; // default name
  if (webServer.hasArg("f")) { 
       filename = webServer.arg("f");
       f = SPIFFS.open(filename, "w");
  } else { 
      f = SPIFFS.open(filename, "w");
  }

  if(webServer.hasArg(kPLAIN)) { // ie PUT/POST content was given....
        // write params to spiffs,  TODO need better checks here, as it comes from the client, but the worse they can do is pretty minor
       //File f = SPIFFS.open("/r900x_params.txt", "w");
       f.print(webServer.arg(kPLAIN));
       f.close();
       message += "\nFile written:";
       message += filename;
       message += "\n";

        if (r900x_saveparams(filename) ) { 
            message += "and 900x radio params activated to modem OK. ";
            retval = 201;
        } else { 
            message += "and 900x radio params activate FAILED. Does file "+filename+" exist? ";
            retval = 202;
        }
    
  }

  webServer.send(retval, "text/plain", message);

  DBG_OUTPUT_PORT.println("handle900xParamSave: " + message);




// optionally run  save900xparams() or r900x_saveparams() 

}

//---------------------------------------------------------------------------------
MavESP8266Httpd::MavESP8266Httpd()
{

}

//---------------------------------------------------------------------------------
//-- Initialize


void
MavESP8266Httpd::begin(MavESP8266Update* updateCB_)
{
    updateCB = updateCB_;
    webServer.on("/",               handle_root);
    webServer.on("/getparameters",  handle_getParameters);
    webServer.on("/setparameters",  handle_setParameters);
    webServer.on("/getstatus",      handle_getStatus);
    webServer.on("/getstatus_tcp",  handle_getStatusTcp);
    webServer.on("/reboot",         handle_reboot);
    webServer.on("/setup",          handle_setup);
    webServer.on("/info.json",      handle_getJSysInfo);
    webServer.on("/status.json",    handle_getJSysStatus);
    webServer.on("/log.json",       handle_getJLog);

    webServer.on("/updatepage",       handle_update_html); // presents a webpage that then might upload a binary to the /upload endpoint via POST

    webServer.on("/wiz", handle_wiz_save); 

//    webServer.on("/save900xparams",         save900xparams); // see /psave

    
    webServer.on("/upload",         HTTP_POST, handle_upload, handle_upload_status); // this save/s an uploaded file


    //list directory, return details as json.
    webServer.on("/list", HTTP_GET, handleFileList);

    //load editor
    webServer.on("/edit", HTTP_GET, []() {
    if (!handleFileRead("/edit.htm")) {
      webServer.send(404, "text/plain", "FileNotFound");
    }
    });

    //create file
    webServer.on("/edit", HTTP_PUT, handleFileCreate);

    //delete file
    webServer.on("/edit", HTTP_DELETE, handleFileDelete);


   //TIP:  /edit endpoint uses ace.js and friends from https://github.com/ajaxorg/ace-builds/tree/master/src

    //first callback is called after the request has ended with all parsed arguments
    //second callback handles file uploads at that location
    //webServer.on("/edit", HTTP_POST, handleFileUpload);

    //webServer.send(200, "text/plain", "");
    //}, handleFileUpload);

//    webServer.on("/edit", HTTP_POST, [](){ webServer.send(200, "text/plain", "yup."); }, handleFileUpload);

    webServer.on("/edit", HTTP_POST, [](){ webServer.sendHeader("Location","/success.htm"); webServer.send(303); }, handleFileUpload);

    //      // Redirect the client to the success page
    //webServer.send(303);

    webServer.on("/plist", HTTP_GET, handle900xParamList);

    webServer.on("/prefresh", HTTP_GET, handle900xParamRefresh);

    webServer.on("/psave", HTTP_PUT, handle900xParamSave);

    //called when the url is not defined here
    //use it to load content from SPIFFS
    webServer.onNotFound([]() {
    if (!handleFileRead(webServer.uri())) {
      webServer.send(404, "text/plain", "FileNotFound");
    }
    });

    //const char* webupdatehost = "esp8266-webupdate";

    //MDNS.begin(webupdatehost);


    httpUpdater.setup(&webServer);

    webServer.begin();

    //MDNS.addService("http", "tcp", 80);
    //DBG_OUTPUT_PORT.printf("HTTPUpdateServer ready! Open http://%s.local/update in your browser\n", webupdatehost);
}

//---------------------------------------------------------------------------------
//-- Initialize
void
MavESP8266Httpd::checkUpdates()
{
    webServer.handleClient();
}
