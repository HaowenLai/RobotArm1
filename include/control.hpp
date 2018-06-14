/* ********************************************************
 *   This file contains control algerithms that may be used 
 * in robotic arm control.
 * @Author : Derek Lai
 * @Date   : 2018/6/11
 * @Version: v1.0
 * Copyright(c) All right reserved
 * ********************************************************/

#ifndef __CONTROL_HPP__
#define __CONTROL_HPP__

#include <vector>
#include "UsbCAN.hpp"

//  This function change the motor pwm values from old values
//to new values by a fixed step Δx, where Δx = 1 by default. 
//It will not return until the new values are reached.
//@CAUTION: step should be positive integer!
void fixStepMove(std::vector<int>& oldVals, 
                 std::vector<int>& newVals,
                 UsbCAN& canDev,
                 int ID,
                 int step = 1);


#endif
