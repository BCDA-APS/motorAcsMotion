#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <string>
#include <sstream>
#include <cstdarg>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynDriver.h"
#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>

#include "SPiiPlusDriver.h"

static const char *driverName = "SPiiPlusController";

static void SPiiPlusProfileThreadC(void *pPvt);

SPiiPlusController::SPiiPlusController(const char* ACSPortName, const char* asynPortName, int numAxes,
                                             double movingPollPeriod, double idlePollPeriod)
 : asynMotorController(ACSPortName, numAxes, 0, 0, 0, ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, 0, 0)
{
	asynStatus status = pasynOctetSyncIO->connect(asynPortName, 0, &pasynUserController_, NULL);
	
	if (status)
	{
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
		"SPiiPlusController::SPiiPlusController: cannot connect to SPii+ controller\n");
		
		return;
	}
	
	for (int index = 0; index < numAxes; index += 1)
	{
		new SPiiPlusAxis(this, index);
	}
	
	this->startPoller(movingPollPeriod, idlePollPeriod, 2);
	
	// Create the event that wakes up the thread for profile moves
	profileExecuteEvent_ = epicsEventMustCreate(epicsEventEmpty);
	
	// Create the thread that will execute profile moves
	epicsThreadCreate("SPiiPlusProfile", 
		epicsThreadPriorityLow,
		epicsThreadGetStackSize(epicsThreadStackMedium),
		(EPICSTHREADFUNC)SPiiPlusProfileThreadC, (void *)this);
	
	// TODO: should this be an arg to the controller creation call or a separate call like it is in the XPS driver
	// hard-code the max number of points for now
	initializeProfile(2000);
}

SPiiPlusAxis* SPiiPlusController::getAxis(asynUser *pasynUser)
{
	return static_cast<SPiiPlusAxis*>(asynMotorController::getAxis(pasynUser));
}

SPiiPlusAxis* SPiiPlusController::getAxis(int axisNo)
{
	return static_cast<SPiiPlusAxis*>(asynMotorController::getAxis(axisNo));
}

asynStatus SPiiPlusController::writeread(const char* format, ...)
{
	va_list args;
	va_start(args, format);
	
	std::fill(outString_, outString_ + 256, '\0');
	std::fill(outString_, inString_ + 256, '\0');
	
	vsprintf(outString_, format, args);
	
	size_t response;
	asynStatus out = this->writeReadController(outString_, inString_, 256, &response, -1);
	
	this->instring = std::string(inString_);
	
	va_end(args);
	
	return out;
}

SPiiPlusAxis::SPiiPlusAxis(SPiiPlusController *pC, int axisNo)
: asynMotorAxis(pC, axisNo)
{
	setIntegerParam(pC->motorStatusHasEncoder_, 1);
	// Gain Support is required for setClosedLoop to be called
	setIntegerParam(pC->motorStatusGainSupport_, 1);
}

asynStatus SPiiPlusAxis::poll(bool* moving)
{
	asynStatus status;
	SPiiPlusController* controller = (SPiiPlusController*) pC_;	
	
	status = controller->writeread("?APOS(%d)", axisNo_);
	controller->instring.replace(0,1," ");
	
	double position;
	std::stringstream pos_convert;

	pos_convert << controller->instring;
	pos_convert >> position;
	
	setDoubleParam(controller->motorPosition_, position);
	
	
	status = controller->writeread("?FPOS(%d)", axisNo_);
	controller->instring.replace(0,1," ");
	
	double enc_position;
	std::stringstream enc_pos_convert;	

	enc_pos_convert << controller->instring;
	enc_pos_convert >> enc_position;
	
	setDoubleParam(controller->motorEncoderPosition_, enc_position);
	
	
	int left_limit, right_limit;
	std::stringstream fault_convert;
	
	status = controller->writeread("?FAULT(%d).#LL", axisNo_);
	controller->instring.replace(0,1," ");
	
	fault_convert << controller->instring;
	fault_convert >> left_limit;
	
	status = controller->writeread("?FAULT(%d).#RL", axisNo_);
	controller->instring.replace(0,1," ");
	
	fault_convert.str("");
	fault_convert.clear();
	
	fault_convert << controller->instring;
	fault_convert >> right_limit;
	
	setIntegerParam(controller->motorStatusHighLimit_, right_limit);
	setIntegerParam(controller->motorStatusLowLimit_, left_limit);
	
	// Read the entire motor status and parse the relevant bits
	int axis_status;
	std::stringstream status_convert;
	
	status = controller->writeread("?D/MST(%d)", axisNo_);
	controller->instring.replace(0,1," ");
	
	status_convert << controller->instring;
	status_convert >> axis_status;
	
	int enabled;
	//int open_loop;
	//int in_pos;
	int motion;
	
	enabled = axis_status & (1<<0);
	//open_loop = axis_status & (1<<1);
	//in_pos = axis_status & (1<<4);
	motion = axis_status & (1<<5);
	
	setIntegerParam(controller->motorStatusDone_, !motion);
	setIntegerParam(controller->motorStatusMoving_, motion);
	setIntegerParam(controller->motorStatusPowerOn_, enabled);
	
	callParamCallbacks();
	
	if (motion)    { *moving = false; }
	else           { *moving = true; }
	
	return status;
}

asynStatus SPiiPlusAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
	asynStatus status;
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	
	status = controller->writeread("XACC(%d)=%f", axisNo_, acceleration + 10);
	status = controller->writeread("ACC(%d)=%f", axisNo_, acceleration);
	status = controller->writeread("DEC(%d)=%f", axisNo_, acceleration);
	
	status = controller->writeread("XVEL(%d)=%f", axisNo_, maxVelocity + 10);
	status = controller->writeread("VEL(%d)=%f", axisNo_, maxVelocity);
	
	if (relative)
	{
		status = controller->writeread("PTP/r %d, %f", axisNo_, position);
	}
	else
	{
		status = controller->writeread("PTP %d, %f", axisNo_, position);
	}
	
	return status;
}

asynStatus SPiiPlusAxis::setPosition(double position)
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	return controller->writeread("SET %d_APOS=%f", axisNo_, position);
}

asynStatus SPiiPlusAxis::stop(double acceleration)
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	return controller->writeread("HALT %d", axisNo_);
}

/** Set the motor closed loop status. 
  * \param[in] closedLoop true = close loop, false = open looop. */
asynStatus SPiiPlusAxis::setClosedLoop(bool closedLoop)
{
	SPiiPlusController* controller = (SPiiPlusController*) pC_;
	asynStatus status;
	
	/*
	 Enable/disable the axis instead of changing the closed-loop state.
	*/
	if (closedLoop)
	{
		status = controller->writeread("ENABLE %d", axisNo_);
	}
	else
	{
		status = controller->writeread("DISABLE %d", axisNo_);
	}
	
	return status;
}

std::string SPiiPlusController::axesToString(std::vector <int> axes)
{
  static const char *functionName = "axesToString";
  uint i;
  std::stringstream outputStr;
  
  for (i=0; i<axes.size(); i++)
  {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: %i axis will be used\n", driverName, functionName, axes[i]);
    if (axes[i] == axes.front())
    {
      outputStr << '(' << axes[i];
    }
    else if (axes[i] == axes.back())
    {
      outputStr << ',' << axes[i] << ')';
    }
    else
    {
      outputStr << ',' << axes[i];
    }
  }
  
  return outputStr.str();
}

/** Function to build a coordinated move of multiple axes. */
asynStatus SPiiPlusController::buildProfile()
{
  //int i, j; 
  int j; 
  //int status;
  //bool buildOK=true;
  //bool verifyOK=true;
  //int numPoints;
  //int numElements;
  //double trajVel;
  //double D0, D1, T0, T1;
  char message[MAX_MESSAGE_LEN];
  int buildStatus;
  //double distance;
  //double maxVelocity;
  //double maxAcceleration;
  //double maxVelocityActual=0.0;
  //double maxAccelerationActual=0.0;
  //double minPositionActual=0.0, maxPositionActual=0.0;
  //double minProfile, maxProfile;
  //double lowLimit, highLimit;
  //double minJerkTime, maxJerkTime;
  //double preTimeMax, postTimeMax;
  //double preVelocity[SPIIPLUS_MAX_AXES], postVelocity[SPIIPLUS_MAX_AXES];
  //double time;
  //int axis
  std::string axisList;
  int useAxis;
  
  static const char *functionName = "buildProfile";
  
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: entry\n",
            driverName, functionName);
            
  // Call the base class method which will build the time array if needed
  asynMotorController::buildProfile();
  
  strcpy(message, "");
  setStringParam(profileBuildMessage_, message);
  setIntegerParam(profileBuildState_, PROFILE_BUILD_BUSY);
  setIntegerParam(profileBuildStatus_, PROFILE_STATUS_UNDEFINED);
  callParamCallbacks();
  
  // Calculate pre & post profile distances?

  // check which axes should be used
  profileAxes_.clear();
  for (j=0; j<numAxes_; j++) {
    getIntegerParam(j, profileUseAxis_, &useAxis);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: %i axis will be used: %i\n", driverName, functionName, j, useAxis);
    if (useAxis)
    {
      profileAxes_.push_back(j);
    }
  }
  
  axisList = axesToString(profileAxes_);
  asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: axisList = %s\n", driverName, functionName, axisList.c_str());
  
  // POINT commands have this syntax: POINT (0,1,5), 1000,2000,3000, 500
  
  // Verfiy the profile (check speed, accel, limit violations)
  
  done:
  // Can't fail if nothing is verified
  buildStatus = PROFILE_STATUS_SUCCESS;
  setIntegerParam(profileBuildStatus_, buildStatus);
  setStringParam(profileBuildMessage_, message);
  if (buildStatus != PROFILE_STATUS_SUCCESS) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: %s\n",
              driverName, functionName, message);
  }
  /* Clear build command.  This is a "busy" record, don't want to do this until build is complete. */
  setIntegerParam(profileBuild_, 0);
  setIntegerParam(profileBuildState_, PROFILE_BUILD_DONE);
  callParamCallbacks();
  //return status ? asynError : asynSuccess;
  return asynSuccess;
}

/** Function to execute a coordinated move of multiple axes. */
asynStatus SPiiPlusController::executeProfile()
{
  // static const char *functionName = "executeProfile";
  epicsEventSignal(profileExecuteEvent_);
  return asynSuccess;
}

/* C Function which runs the profile thread */ 
static void SPiiPlusProfileThreadC(void *pPvt)
{
  SPiiPlusController *pC = (SPiiPlusController*)pPvt;
  pC->profileThread();
}

/* Function which runs in its own thread to execute profiles */ 
void SPiiPlusController::profileThread()
{
  while (true) {
    epicsEventWait(profileExecuteEvent_);
    runProfile();
  }
}

/* Function to run trajectory.  It runs in a dedicated thread, so it's OK to block.
 * It needs to lock and unlock when it accesses class data. */ 
asynStatus SPiiPlusController::runProfile()
{
  //IAMHERE

  // move motors to the starting position

  // configure data recording

  // configure pulse output

  // start data recording?

  // wake up poller
  
  // run the trajectory
  
  // cleanup

  return asynSuccess;
}

/** Function to abort a profile. */
asynStatus SPiiPlusController::abortProfile()
{
  // static const char *functionName = "abortProfile";
  // TODO
  return asynSuccess;
}

static void AcsMotionConfig(const char* acs_port, const char* asyn_port, int num_axes, double moving_rate, double idle_rate)
{
	new SPiiPlusController(acs_port, asyn_port, num_axes, moving_rate, idle_rate);
}


extern "C"
{

// ACS Setup arguments
static const iocshArg configArg0 = {"ACS port name", iocshArgString};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};
static const iocshArg configArg2 = {"num axes", iocshArgInt};
static const iocshArg configArg3 = {"Moving polling rate", iocshArgDouble};
static const iocshArg configArg4 = {"Idle polling rate", iocshArgDouble};

static const iocshArg * const AcsMotionConfigArgs[5] = {&configArg0, &configArg1, &configArg2, &configArg3, &configArg4};

static const iocshFuncDef configAcsMotion = {"AcsMotionConfig", 5, AcsMotionConfigArgs};

static void AcsMotionCallFunc(const iocshArgBuf *args)
{
    AcsMotionConfig(args[0].sval, args[1].sval, args[2].ival, args[3].dval, args[4].dval);
}

static void AcsMotionRegister(void)
{
	iocshRegister(&configAcsMotion, AcsMotionCallFunc);
}

epicsExportRegistrar(AcsMotionRegister);

}
