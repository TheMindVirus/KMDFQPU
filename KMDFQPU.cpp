#include "KMDFQPU.h"

#ifdef ALLOC_PRAGMA
#pragma alloc_text (INIT, DriverEntry)
#endif

extern "C" { int _fltused = 0; }

NTSTATUS DriverEntry
(
    PDRIVER_OBJECT DriverObject,
    PUNICODE_STRING RegistryPath
)
{
    debug("[CALL]: DriverEntry");
    NTSTATUS status = STATUS_SUCCESS;
    WDF_DRIVER_CONFIG DriverConfig = { 0 };

    WDF_DRIVER_CONFIG_INIT(&DriverConfig, DriverDeviceAdd);
    DriverConfig.EvtDriverUnload = DriverUnload;

    status = WdfDriverCreate(DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &DriverConfig, WDF_NO_HANDLE);
    if (NT_ERROR(status)) { debug("[WARN]: WdfDriverCreate Failed (0x%08lX)", status); return status; }
    return STATUS_SUCCESS;
}

NTSTATUS DriverDeviceAdd
(
    WDFDRIVER Driver,
    PWDFDEVICE_INIT DeviceInit
)
{
    debug("[CALL]: DriverDeviceAdd");
    NTSTATUS status = STATUS_SUCCESS;
    UNICODE_STRING symlink = { 0 };
    WDF_PNPPOWER_EVENT_CALLBACKS configPnP = { 0 };
    WDF_IO_QUEUE_CONFIG configIO = { 0 };
    WDFDEVICE device = { 0 };

    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&configPnP);
    configPnP.EvtDevicePrepareHardware = DevicePrepareHardware;
    configPnP.EvtDeviceReleaseHardware = DeviceReleaseHardware;
    configPnP.EvtDeviceD0Entry = DevicePowerUp;
    configPnP.EvtDeviceD0Exit = DevicePowerDown;
    WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &configPnP);

    status = WdfDeviceCreate(&DeviceInit, WDF_NO_OBJECT_ATTRIBUTES, &device);
    if (NT_ERROR(status)) { debug("[WARN]: WdfDeviceCreate Failed (0x%08lX)", status); return status; }

    return STATUS_SUCCESS;
    UNREFERENCED_PARAMETER(Driver);
}

void DriverUnload
(
    WDFDRIVER Driver
)
{
    debug("[CALL]: DriverUnload");
    UNREFERENCED_PARAMETER(Driver);
}

NTSTATUS DevicePrepareHardware
(
    WDFDEVICE Device,
    WDFCMRESLIST ResourcesRaw,
    WDFCMRESLIST ResourcesTranslated
)
{
    debug("[CALL]: DevicePrepareHardware");
    int argc = 3;
    char* argv[] = { "10", "10", "10" };
    main(argc, argv);
    return STATUS_SUCCESS;
    UNREFERENCED_PARAMETER(Device);
    UNREFERENCED_PARAMETER(ResourcesRaw);
    UNREFERENCED_PARAMETER(ResourcesTranslated);
}

NTSTATUS DeviceReleaseHardware
(
    WDFDEVICE Device,
    WDFCMRESLIST ResourcesTranslated
)
{
    debug("[CALL]: DeviceReleaseHardware");
    return STATUS_SUCCESS;
    UNREFERENCED_PARAMETER(Device);
    UNREFERENCED_PARAMETER(ResourcesTranslated);
}

NTSTATUS DevicePowerUp
(
    WDFDEVICE Device,
    WDF_POWER_DEVICE_STATE PreviousState
)
{
    debug("[CALL]: DevicePowerUp");
    return STATUS_SUCCESS;
    UNREFERENCED_PARAMETER(Device);
    UNREFERENCED_PARAMETER(PreviousState);
}

NTSTATUS DevicePowerDown
(
    WDFDEVICE Device,
    WDF_POWER_DEVICE_STATE TargetState
)
{
    debug("[CALL]: DevicePowerDown");
    return STATUS_SUCCESS;
    UNREFERENCED_PARAMETER(Device);
    UNREFERENCED_PARAMETER(TargetState);
}

static BOOLEAN _DebugOverwrite = TRUE;
void DebugLogToFile(LPSTR format, ...)
{
    NTSTATUS status = STATUS_SUCCESS;

    char buffer[255] = "";
    va_list va;
    va_start(va, format);
    vsprintf(buffer, format, va);
    va_end(va);
    strcat(buffer, "\n");

    KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL, buffer));
    
    HANDLE file;
    UNICODE_STRING path;
    WCHAR pathbuffer[1024];
    IO_STATUS_BLOCK result;
    OBJECT_ATTRIBUTES attributes;

    wcscpy(pathbuffer, DEBUG_FILEROOT);
    wcscat(pathbuffer, DEBUG_FILENAME);
    RtlInitUnicodeString(&path, pathbuffer);
    InitializeObjectAttributes(&attributes, &path, OBJ_CASE_INSENSITIVE | OBJ_KERNEL_HANDLE, NULL, NULL);
    status = ZwCreateFile(&file, FILE_APPEND_DATA, &attributes, &result, NULL, FILE_ATTRIBUTE_NORMAL, FILE_SHARE_READ,
        (_DebugOverwrite) ? FILE_OVERWRITE_IF : FILE_OPEN_IF, FILE_SYNCHRONOUS_IO_NONALERT, NULL, 0);
    if (!NT_SUCCESS(status)) { return; }
    _DebugOverwrite = FALSE;

    status = ZwWriteFile(file, NULL, NULL, NULL, &result, buffer, (ULONG)strlen(buffer), NULL, NULL);
    for (ULONG i = 0; i < 10; ++i)
    {
        if (result.Status != STATUS_PENDING) { break; }
        KeStallExecutionProcessor(100);
    }
    ZwClose(file);
}