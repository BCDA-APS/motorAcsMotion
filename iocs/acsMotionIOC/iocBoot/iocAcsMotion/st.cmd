#!../../bin/linux-x86_64/acsMotion

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/acsMotionIOC.dbd"
acsMotion_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=acsMotion:")

## 
#< AcsMotion.cmd

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("acsMotion:")

# Boot complete