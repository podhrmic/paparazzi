#include <iostream>

#include "include/LogAutopilot.h"
#include "include/VectorNav.h"

static LogAutopilot *ap;
static VectorNav *vn;
static asio::serial_port *apt;
static asio::serial_port *vpt;

void init_ap() {
  /*
   * We need to open the port before calling autopilot
   */
  asio::io_service io;
  asio::serial_port port(io);
  apt = &port;

  try {
    apt->open(AP_DEV);
    apt->set_option(asio::serial_port_base::baud_rate(AP_BAUD));
  }
  catch (const std::exception& e)
  {
    std::cout << "Exception: " << e.what() << "\n";
  }

  boost::asio::io_service ap_service;
  LogAutopilot a(ap_service, *apt);
  ap = &a;
}

void init_vn() {
  asio::io_service io;
  asio::serial_port port(io);
  vpt = &port;

  try {
    vpt->open("/dev/ttyUSB1");
    vpt->set_option(asio::serial_port_base::baud_rate(921600));
  }
  catch (const std::exception& e)
  {
    std::cout << "Exception: " << e.what() << "\n";
  }
  cout << "port opened!\n";

  boost::asio::io_service vn_service;
  VectorNav v(vn_service, *vpt);
  vn = &v;
}


int main() {
  //init_vn();
  //init_ap();







  //apt->close();
  //vpt->close();
  cout << "Done" << endl;
}
