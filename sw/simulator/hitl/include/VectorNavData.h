/*
 * VectorNavData.h
 *
 *  Created on: Apr 25, 2015
 *      Author: fwmav
 */

#ifndef INCLUDE_VECTORNAVDATA_H_
#define INCLUDE_VECTORNAVDATA_H_

class VectorNavData {
public:
  uint64_t TimeStartup;
  float YawPitchRoll[3] = {0};
  float AngularRate[3] = {0};
  double Position[3] = {0};
  float Velocity[3] = {0};
  float Accel[3] = {0};
  uint64_t Tow;
  uint8_t NumSats;
  uint8_t Fix;
  float PosU[3] = {0};
  float VelU;
  float LinearAccelBody[3] = {0};
  float YprU[3] = {0};
  uint16_t InsStatus;
  float VelBody[3] = {0};
};



#endif /* INCLUDE_VECTORNAVDATA_H_ */
