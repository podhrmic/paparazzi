#include <iostream>

#include "include/LogAutopilot.h"
#include "include/VectorNav.h"
#include "include/FDM.h"


const float SIM_DT (1./400.0); // we're running at 400Hz

static LogAutopilot *ap;
static VectorNav *vn;
static asio::serial_port *apt;
static asio::serial_port *vpt;
static FDM *sm;

void init_ap() {
  try {
    apt->open(AP_DEV);
    apt->set_option(asio::serial_port_base::baud_rate(AP_BAUD));
  }
  catch (const std::exception& e)
  {
    std::cout << "AP: Exception: " << e.what() << "\n";
    exit(-1);
  }
  cout << "AP port opened!\n";
}

void init_vn() {
  try {
    vpt->open("/dev/ttyUSB1");
    vpt->set_option(asio::serial_port_base::baud_rate(921600));
  }
  catch (const std::exception& e)
  {
    std::cout << "VN: Exception: " << e.what() << "\n";
    exit(-1);
  }
  cout << "VN port opened!\n";
}


int main() {
  asio::io_service io1;
  asio::serial_port port1(io1);
  apt = &port1;
  init_ap();
  LogAutopilot a(*apt);
  ap = &a;

  asio::io_service io2;
  asio::serial_port port2(io2);
  vpt = &port2;
  init_vn();
  VectorNav v(*vpt);
  vn = &v;

  FDM sim;
  sm = &sim;

  // master loop
while (true) {
  // read command message
  while(!ap->newDataAvailable()) {
    ap->workerFunc();
  }

  //get autopilot data
  ap->getAutopilotData();

  // process it somehow

  // send it to JSBSim

  // get results

  // convert for VN
  vn->setNewData(true);

  // send to AP
  if (vn->hasNewData()) {
    vn->workerFunc();
  }
}
  apt->close();
  vpt->close();
  cout << "Done" << endl;
}
