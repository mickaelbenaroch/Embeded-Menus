# MPLAB IDE generated this makefile for use with GNU make.
# Project: Lab1.mcp
# Date: Sat Dec 29 15:35:20 2018

AS = MPASMWIN.exe
CC = mcc18.exe
LD = mplink.exe
AR = mplib.exe
RM = rm

Lab1.cof : main.o oled.o
	$(LD) /p18F46J50 /l"..\..\MPLAB C18\lib" "rm18f46j50_g.lkr" "main.o" "oled.o" "D:\Workspace\Embeded\mplab\Microchip\Obj\BMA150.o" "D:\Workspace\Embeded\mplab\Microchip\Obj\mtouch.o" "D:\Workspace\Embeded\mplab\Microchip\Obj\soft_start.o" /u_CRUNTIME /z__MPLAB_BUILD=1 /m"Lab1.map" /w /o"Lab1.cof"

main.o : main.c C:/Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/stdio.h C:/Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/stdlib.h C:/Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/string.h ../../Microchip/mTouch/mtouch.h ../../Microchip/BMA150\ driver/BMA150.h oled.h main.c ../../Microchip/Include/GenericTypeDefs.h ../../Microchip/Include/Compiler.h C:/Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/p18cxxx.h C:/Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/p18f46j50.h C:/Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/stdarg.h C:/Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/stddef.h ../../Microchip/Include/HardwareProfile.h ../../Microchip/Include/HardwareProfile\ -\ PIC18F\ Starter\ Kit.h ../../Microchip/Soft\ Start/soft_start.h
	$(CC) -p=18F46J50 /i"..\..\Microchip\Soft Start" -I"..\..\Microchip\BMA150 driver" -I"..\..\Microchip\OLED driver" -I"..\..\Microchip\mTouch" -I"..\..\Microchip\Include" "main.c" -fo="main.o" -w1 -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

oled.o : oled.c C:/Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/stdio.h C:/Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/stdlib.h C:/Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/string.h oled.h oled.c ../../Microchip/Include/GenericTypeDefs.h ../../Microchip/Include/Compiler.h C:/Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/p18cxxx.h C:/Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/p18f46j50.h C:/Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/stdarg.h C:/Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/stddef.h
	$(CC) -p=18F46J50 /i"..\..\Microchip\Soft Start" -I"..\..\Microchip\BMA150 driver" -I"..\..\Microchip\OLED driver" -I"..\..\Microchip\mTouch" -I"..\..\Microchip\Include" "oled.c" -fo="oled.o" -w1 -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

clean : 
	$(RM) "main.o" "oled.o" "Lab1.cof" "Lab1.hex" "Lab1.map"

