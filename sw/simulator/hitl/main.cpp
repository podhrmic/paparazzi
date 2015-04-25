#include <iostream>

#include "include/LogAutopilot.h"

static LogAutopilot *ap;
static asio::serial_port *pt;

void init() {
  /*
   * We need to open the port before calling autopilot
   */
  asio::io_service io;
  asio::serial_port port(io);
  pt = &port;

  try {
    pt->open(AP_DEV);
    //port.open(AP_DEV);
    //port.set_option(asio::serial_port_base::baud_rate(AP_BAUD));
    pt->set_option(asio::serial_port_base::baud_rate(AP_BAUD));
  }
  catch (const std::exception& e)
  {
    std::cout << "Exception: " << e.what() << "\n";
  }

  LogAutopilot a(*pt);
  ap = &a;
  ap->workerFunc();

}


int main() {
  init();

  pt->close();
  cout << "Done" << endl;
}
