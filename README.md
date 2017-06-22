SIS3302 Driver for Transient Recorder Framework
=========

This is a driver for the Struck SIS3302 digitizer using the Transient Recorder Framework.

For documentation, go to the [transRecorder](https://github.com/epics-modules/transRecorder)
repository page and follow the link to the documentation, and there go to Related Pages,
SIS3302 Digitizer Driver.

# IOC Booting Notes

## Using the SLAC Bootloader

Get to the Motload shell.

Set the time (MMDDYYHHMMSS).

```
set -t050407080600
```

Download the netboot image.

```
tftpGet -d/dev/enet0 -c<cpu_ip> -s1<server_ip> -g<gateway_ip> -m255.255.252.0 -f/rtems-slac/4.9.4/target/ssrlApps/powerpc-rtems/mvme3100/img/netboot.flashimg.bin
```

Copy the netboot image to flash. Here `-s` is the load address as printed by tftpGet.

```
flashProgram -s014D3000 -v
```

Copy back from flash to RAM.
Here `-a` and `-b` are the Destination Starting/Ending Addresses as printer by flashProgram.

```
bmw -aF8000000 -bF80FFFFF -c4000000
```

Test the boot process by jumping to the RAM location where the data was copied back.
When prompted press any key to interrupt the boot!

```
go -a4000000
```

Press `c` to change boot settings.

```
Boot file (e.g., '/TFTP/1.2.3.4/path', '~rshuser/path' or 'nfshost:/dir:path'):
 >10.68.5.15:/home/slac-admin:/rtems-slac/4.9.4/target/ssrlApps/powerpc-rtems/mvme3100/bin/rtems.ralf
Command line parameters:
 >INIT=/boot/genstds-epics/TRSIS/iocBoot/iocSISTestIoc/startup.cexp
Server IP:    >10.68.5.15
Gateway IP:   >10.68.0.1
My media (e.g., '100baseTX-full' ['?' for help])
              >auto
My IP:        >10.68.5.10
My netmask:   >255.255.0.0
My name:      >mvme3100slac
My domain:    >
Loghost IP:   >
DNS server 1: >10.68.0.1
DNS server 2: >
DNS server 3: >
NTP server 1: >
NTP server 2: >
NTP server 3: >
Use BOOTP: Yes, No or Partial (-> file and
          command line from NVRAM) [Y/N/P]>N
Autoboot Delay: [0...30secs] (0==forever) >5
```

Reboot and in MotLoad interrupt boot, then set the boot script.

```
gevEdit mot-script-boot
netShut
bmw -aF8000000 -bF80FFFFF -c4000000
go -a4000000
```

After this is done, MotLoad is set to automatically start the SLAC bootloader from flash.

The SLAC bootloader will automatically download the RTEMS binary from NFS and execute
the startup script (startup.cexp).

The startup script is interpreted by Cexp, and it does only three things:

- Load the application binary: `cexpModuleLoad("../../bin/RTEMS-mvme3100/SISTestIoc.obj")`
- Set the TOP variable (relative to the IOC boot directory): `epicsEnvSet("TOP", "../..")`
- Run the IOC's st.cmd using the IOC shell interpreter: `iocsh("st.cmd")`.

## Vanilla EPICS Environment (no SLAC stuff)

```
gevEdit mot-script-boot
buf = malloc 0x300000
tftpGet -d/dev/enet0 -f/genstds-epics/TRSIS/bin/RTEMS-mvme3100/SISTestIoc.boot -abuf
netShut
go -abuf

gevEdit epics-script
10.68.5.15:/home/slac-admin:/genstds-epics/TRSIS/iocBoot/iocSISTestIoc/st.cmd
```

You also need to setup the network configuration environment variables or add parameters
to the tftpGet command.
