/*
 * FDM.h
 *
 *  Created on: Apr 25, 2015
 *      Author: fwmav
 */

#ifndef INCLUDE_FDM_H_
#define INCLUDE_FDM_H_

#include <FGFDMExec.h>
#include <FGJSBBase.h>
#include <initialization/FGInitialCondition.h>
#include <models/FGPropulsion.h>
#include <models/FGGroundReactions.h>
#include <models/FGAccelerations.h>
#include <models/atmosphere/FGWinds.h>

using namespace JSBSim;
using namespace std;

/*
 * Flight Dynamic Model for JSBSim
 */
class FDM {
private:
  /// The JSBSim executive object
  static FGFDMExec* FDMExec;

public:
  FDM(){
    FDMExec = new FGFDMExec();
  }

};



#endif /* INCLUDE_FDM_H_ */
