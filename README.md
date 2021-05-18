# KMDFQPU
V3D Windows Driver Source for accessing the QPU's on the Raspberry Pi 4 (Development Version)

Work In Progress, Please see: https://github.com/TheMindVirus/KMDFQPU/tree/hello_fft \
which needs heavy restructuring and simplification to use only 3 registers for control:

```
0x00430 V3D_SRQPC QPU User Program Request Program Address 32 
0x00434 V3D_SRQUA QPU User Program Request Uniforms Address 32 
0x00438 V3D_SRQUL QPU User Program Request Uniforms Length 32
```

More Information on the V3D can be found in the Broadcom VideoCoreIV 3D Architecture Guide \
https://docs.broadcom.com/doc/12358545
