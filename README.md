# KMDFQPU
V3D Windows Driver Source for accessing the QPU's on the Raspberry Pi 4 (Development Version)

Mailbox Reliability Test

This test disproves the following statement:
`MB 0 is always for communication from VC to ARM and MB 1 is for ARM to VC.` \
`The ARM should never write MB 0 or read MB 1.`

The Read register of Mailbox 0 needs to be set to 0x0 before read/write operations.

https://github.com/raspberrypi/firmware/tree/master/opt/vc/src/hello_pi/hello_fft

![qpudebug](https://github.com/TheMindVirus/KMDFQPU/blob/mailbox_drv/MailboxDebug.png)