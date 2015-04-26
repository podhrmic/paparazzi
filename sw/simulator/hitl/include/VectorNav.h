/*
 * VectorNav.h
 *
 *  Created on: Apr 25, 2015
 *      Author: fwmav
 */

#ifndef INCLUDE_VECTORNAV_H_
#define INCLUDE_VECTORNAV_H_

#include <stdio.h>     // Standard input/output definitions
#include <string.h>    // String function definitions

#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>

#include "VectorNavData.h"

const uint8_t VN_DATA_START = 10;
const short VN_BUFFER_SIZE = 512;
const int VN_UPDATE_RATE = 2;

const uint8_t VN_SYNC = 0xFA;
const uint8_t VN_OUTPUT_GROUP = 0x39;
const uint16_t VN_GROUP_FIELD_1 = 0x01E9;
const uint16_t VN_GROUP_FIELD_2 = 0x061A;
const uint16_t VN_GROUP_FIELD_3 = 0x0140;
const uint16_t VN_GROUP_FIELD_4 = 0x0009;

using namespace boost;
using namespace std;

class VectorNav {
private:
  asio::io_service& io_service_;
  asio::serial_port& port;
  uint8_t buffer[VN_BUFFER_SIZE] = {0};
  uint8_t work_; // flag
  bool new_data;
  boost::mutex datalock; // mutex
  VectorNavData data; // data to be sent.


  /**
   * Calculates the 16-bit CRC for the given ASCII or binary message.
   * The CRC is calculated over the packet starting just after the sync byte (not including the sync byte)
   * and ending at the end of payload.
   */
  unsigned short calculateCRC(unsigned char data[], unsigned int length)
  {
    unsigned int i;
    unsigned short crc = 0;
    for (i=0; i<length; i++){
      crc = (unsigned char)(crc >> 8) | (crc << 8);
      crc ^= data[i];
      crc ^= (unsigned char)(crc & 0xff) >> 4;
      crc ^= crc << 12;
      crc ^= (crc & 0x00ff) << 5;
    }
    return crc;
  }

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
   * Send data to port
   */
  void send_data(){
    static uint8_t cksum0, cksum1;
    static uint16_t idx;

    buffer[0] = VN_SYNC;
    buffer[1] = VN_OUTPUT_GROUP;
    buffer[2] = (uint8_t) (VN_GROUP_FIELD_1 >> 8);
    buffer[3] = (uint8_t) (VN_GROUP_FIELD_1);
    buffer[4] = (uint8_t) (VN_GROUP_FIELD_2 >> 8);
    buffer[5] = (uint8_t) (VN_GROUP_FIELD_2);
    buffer[6] = (uint8_t) (VN_GROUP_FIELD_3 >> 8);
    buffer[7] = (uint8_t) (VN_GROUP_FIELD_3);
    buffer[8] = (uint8_t) (VN_GROUP_FIELD_4 >> 8);
    buffer[9] = (uint8_t) (VN_GROUP_FIELD_4);

    idx = VN_DATA_START;

    // Timestamp
    memcpy(&buffer[idx], &data.TimeStartup, sizeof(uint64_t));
    idx += sizeof(uint64_t);

    //Attitude, float, [degrees], yaw, pitch, roll, NED frame
    memcpy(&buffer[idx], &data.YawPitchRoll, 3*sizeof(float));
    idx += 3*sizeof(float);

    // Rates (imu frame), float, [rad/s]
    memcpy(&buffer[idx], &data.AngularRate, 3*sizeof(float));
    idx += 3*sizeof(float);

    //Pos LLA, double,[beg, deg, m]
    //The estimated position given as latitude, longitude, and altitude given in [deg, deg, m] respectfully.
    memcpy(&buffer[idx], &data.Position, 3*sizeof(double));
    idx += 3*sizeof(double);

    //VelNed, float [m/s]
    //The estimated velocity in the North East Down (NED) frame, given in m/s.
    memcpy(&buffer[idx], &data.Velocity, 3*sizeof(float));
    idx += 3*sizeof(float);

    // Accel (imu-frame), float, [m/s^-2]
    memcpy(&buffer[idx], &data.Accel, 3*sizeof(float));
    idx += 3*sizeof(float);

    // tow (in nanoseconds), uint64
    memcpy(&buffer[idx], &data.Tow, sizeof(uint64_t));
    idx += sizeof(uint64_t);

    //num sats, uint8
    buffer[idx] = data.NumSats;
    idx++;

    //gps fix, uint8
    buffer[idx] = data.Fix;
    idx++;

    //posU, float[3]
    memcpy(&buffer[idx], &data.PosU, 3*sizeof(float));
    idx += 3*sizeof(float);

    //velU, float
    memcpy(&buffer[idx], &data.VelU, sizeof(float));
    idx += sizeof(float);

    //linear acceleration imu-body frame, float [m/s^2]
    memcpy(&buffer[idx], &data.LinearAccelBody, 3*sizeof(float));
    idx += 3*sizeof(float);

    //YprU, float[3]
    memcpy(&buffer[idx], &data.YprU, 3*sizeof(float));
    idx += 3*sizeof(float);

    //instatus, uint16
    memcpy(&buffer[idx], &data.InsStatus, sizeof(uint16_t));
    idx += sizeof(uint16_t);

    //Vel body, float [m/s]
    // The estimated velocity in the body (i.e. imu) frame, given in m/s.
    memcpy(&buffer[idx], &data.VelBody, 3*sizeof(float));
    idx += 3*sizeof(float);

    // calculate checksum & send
    uint16_t chk = calculateCRC(&buffer[1], idx-1);
    buffer[idx] = (uint8_t) (chk >> 8);
    idx++;
    buffer[idx] = (uint8_t) (chk & 0xFF);
    idx++;

    int len = port.write_some(asio::buffer(buffer, idx));
    cout << "Written " << len << " bytes\n";
  }

public:
  VectorNav(asio::io_service& io_service, asio::serial_port& port_n)
    : io_service_(io_service), port(port_n) {
    work_ = 1;
    new_data = false;
    data = VectorNavData(); // data to be sent.

    boost::thread autothread(boost::bind(&VectorNav::workerFunc, this));
  }

  void workerFunc()
  {
    std::cout << "VN Thread: Starting to work \n";
    while (work_) {
      //if (new_data) {
        datalock.lock();
        send_data();
        new_data = false;
        datalock.unlock();
        std::cout << "VN Thread: Working \n";
      //}
      // eventually we wont need to sleep - we will just wait for a signal from another thread
      // this is just an example
      msleep(VN_UPDATE_RATE);
    }
  }
};



#endif /* INCLUDE_VECTORNAV_H_ */
