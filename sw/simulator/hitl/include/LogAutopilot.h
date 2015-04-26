/*
 * log_autopilot.h
 *
 *  Created on: Apr 25, 2015
 *      Author: Michal Podhradsky
 *
 *  This class takes USB_SERIAL from autopilot, and reads
 *  commands (and possibly other data) from AP.
 *
 */

#ifndef LOGAUTOPILOT_H_
#define LOGAUTOPILOT_H_

#include <stdio.h>     // Standard input/output definitions
#include <string.h>    // String function definitions

#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>

#include "MsgConfig.h"
#include "MsgPacket.h"
#include "AutopilotData.h"

using namespace boost;
using namespace std;

class LogAutopilot {
private:
  MsgPacket data_packet;
  AutopilotData data;
  asio::serial_port& port;
  char buffer[BUF_SIZE] = {};
  uint8_t work_; // flag
  bool new_data;
  boost::mutex datalock; // mutex

  /**
   * Sleep in ms
   */
  void msleep(int ms)
  {
    struct timespec ts;

    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000;

    nanosleep(&ts, NULL);
  }

  /**
   * Verify checksum
   */
  bool verify_chk(volatile uint8_t *buf_add, uint16_t size) {
    static uint8_t c0, c1;
    static uint16_t i;//, size;
    c0 = 0;
    c1 = 0;
    //the header is not part of the checksum
    for ( i = 0; i < size; i++ ) {
      c0 += (uint8_t)buf_add[i];
      c1 += c0;
    }
    return ( (uint16_t)(c0 << 8 | c1) == (uint16_t)(buf_add[size] << 8 | buf_add[size+1]));
  }

  /**
   * Attempt to read data from the serial port
   */
  void read_data(void) {
    //int length = boost::asio::read(port,boost::asio::buffer((char*)buffer, sizeof(buffer)));
    // FIXME: just a hack for now - add some buffers later
    int length = boost::asio::read(port,boost::asio::buffer((char*)buffer, 1));

    if (length == -1)
    {
      printf("Error reading from serial port\n");
      return;
    }
    else if (length == 0)
    {
      //printf("No more data\n");
      return;
    }
    else
    {
      // parse individual bytes
      for (int i=0; i<length;i++) {
        parse_message(buffer[i]);
      }
    }
  }

  /**
   * Parse message
   */
  void parse_message(uint8_t c){
    switch (data_packet.status) {
      case MsgSync:
        data_packet.msg_idx = 0;
        if (c == HITL_SYNC) {
          data_packet.status = MsgData;
        } else {
          data_packet.hdr_error++;
        }
        break;
      case MsgData:
        data_packet.msg_buf[data_packet.msg_idx] =  c;
        data_packet.msg_idx++;
        if (data_packet.msg_idx == (data_packet.datalength+CHK_LEN)) {
          if (verify_chk(data_packet.msg_buf,data_packet.datalength)) {
            read_message();
          } else {
            data_packet.chksm_error++;
          }
          data_packet.status = MsgSync;
        }
        break;
      default:
        data_packet.status = MsgSync;
        data_packet.msg_idx = 0;
        break;
    }
  }

  /**
   * Read message
   * we have a packet with datalength and a buffer
   */
  void read_message(void){
    datalock.lock();
    uint16_t idx = 0;

    data.msg_cnt = (uint32_t) (data_packet.msg_buf[idx]   << 24 |
        data_packet.msg_buf[idx+1] << 16 |
        data_packet.msg_buf[idx+2] << 8  |
        data_packet.msg_buf[idx+3]);
    idx += sizeof(uint32_t);


    // Actuators value, int16
    memcpy(&data.actuators, &data_packet.msg_buf[idx], data.actuators_nb*sizeof(int16_t));
    idx += data.actuators_nb*sizeof(uint16_t);
    new_data = true; // mew data arrived
    datalock.unlock();

    //debug

    cout << "CNT: " << data.msg_cnt << ", hdr errors: " << data_packet.hdr_error << ", chck err: "
        << data_packet.chksm_error << ", " <<
        data.actuators[0] << ", " << data.actuators[1] << ", " << data.actuators[2] << ", " << data.actuators[3] << endl;

    // here some signal that we can proceed further...maybe another buffer with commands?
  }

public:
  LogAutopilot(asio::serial_port& port_n)
    : port(port_n) {
    std::cout << "Binding autopilot \n";
    work_ = 1;
    data_packet = MsgPacket();
    data = AutopilotData();
    new_data = false;
    work_ = 1;
  }

  void workerFunc()
  {
    //std::cout << "AP Thread: Starting to work \n";
    //while (work_) {
      read_data();
      // Sleep not needed - we just wait
      //msleep(AUTOPILOT_UPDATE_RATE);
    //}
  }

  /**
   * Ask first if new data are available
   * @return true if yes
   */
  bool newDataAvailable(){
    datalock.lock();
    bool ans = new_data;
    datalock.unlock();
    return ans;
  }


  /**
   * get data from autopilot. Also marks new_data false
   * as they were just read.
   * @return AutopilotData - contains all data received
   * from the message.
   */
  AutopilotData getAutopilotData(){
    datalock.lock();
    AutopilotData n_data = data;
    new_data = false;
    datalock.unlock();
    return n_data;
  }

};


#endif /* LOGAUTOPILOT_H_ */
