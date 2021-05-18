#ifndef _KMDFQPU_H_
#define _KMDFQPU_H_

#define DBG 1

#include <ntddk.h>
#include <wdf.h>

#include <math.h> //Note: All Windows Drivers need to be able to perform Floating Point Operations without Hassle.
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#pragma warning(disable : 26451) //Deliberate Arithmetic Overflow is a Feature, not a Bug.

#define debug(...)   \
	KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL, __VA_ARGS__)); \
	KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL, "\n"));

//#define debug(...) DebugLogToFile(__VA_ARGS__);

#define DEBUG_FILEROOT   L"\\SystemRoot\\"
#define DEBUG_FILENAME   L"KMDFQPU.log"

extern "C"
{
	DRIVER_INITIALIZE DriverEntry;

	EVT_WDF_DRIVER_DEVICE_ADD DriverDeviceAdd;
	EVT_WDF_DRIVER_UNLOAD     DriverUnload;

	EVT_WDF_DEVICE_PREPARE_HARDWARE DevicePrepareHardware;
	EVT_WDF_DEVICE_RELEASE_HARDWARE DeviceReleaseHardware;
	EVT_WDF_DEVICE_D0_ENTRY         DevicePowerUp;
	EVT_WDF_DEVICE_D0_EXIT          DevicePowerDown;

	void main(int argc, char* argv[]);

	void DebugLogToFile(LPSTR format, ...);
}

#endif//_KMDFQPU_H_