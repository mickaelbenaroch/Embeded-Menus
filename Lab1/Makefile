# MPLAB IDE generated this makefile for use with GNU make.
# Project: Lab1.mcp
# Date: Wed Jan 16 19:59:32 2019

AS = MPASMWIN.exe
CC = mcc18.exe
LD = mplink.exe
AR = mplib.exe
RM = rm

Lab1.cof : main.o oled.o
	$(LD) /p18F46J50 /l"..\..\MPLAB C18\lib" "rm18f46j50_g.lkr" "main.o" "oled.o" "C:\Users\Mickael\Desktop\Microchip\Obj\BMA150.o" "C:\Users\Mickael\Desktop\Microchip\Obj\mtouch.o" "C:\Users\Mickael\Desktop\Microchip\Obj\soft_start.o" /u_CRUNTIME /z__MPLAB_BUILD=1 /m"Lab1.map" /w /o"Lab1.cof"

main.o : main.c ../../../../../Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/stdio.h ../../../../../Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/stdlib.h ../../../../../Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/string.h ../../Microchip/mTouch/mtouch.h ../../Microchip/BMA150\ driver/BMA150.h ../../Microchip/OLED\ driver/oled.h main.c ../../Microchip/Include/GenericTypeDefs.h ../../Microchip/Include/Compiler.h ../../../../../Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/p18cxxx.h ../../../../../Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/p18f46j50.h ../../../../../Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/stdarg.h ../../../../../Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/stddef.h ../../Microchip/Include/HardwareProfile.h ../../Microchip/Include/HardwareProfile\ -\ PIC18F\ Starter\ Kit.h ../../Microchip/Soft\ Start/soft_start.h
	$(CC) -p=18F46J50 /i"..\..\Microchip\Soft Start" -I"..\..\Microchip\BMA150 driver" -I"..\..\Microchip\OLED driver" -I"..\..\Microchip\mTouch" -I"..\..\Microchip\Include" "main.c" -fo="main.o" -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

oled.o : ../../Microchip/OLED\ driver/oled.c ../../../../../Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/stdio.h ../../../../../Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/stdlib.h ../../../../../Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/string.h ../../Microchip/OLED\ driver/oled.h ../../Microchip/OLED\ driver/oled.c ../../Microchip/Include/GenericTypeDefs.h ../../Microchip/Include/Compiler.h ../../../../../Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/p18cxxx.h ../../../../../Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/p18f46j50.h ../../../../../Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/stdarg.h ../../../../../Program\ Files\ (x86)/Microchip/mplabc18/v3.47/h/stddef.h
	$(CC) -p=18F46J50 /i"..\..\Microchip\Soft Start" -I"..\..\Microchip\BMA150 driver" -I"..\..\Microchip\OLED driver" -I"..\..\Microchip\mTouch" -I"..\..\Microchip\Include" "C:\Users\Mickael\Desktop\Microchip\OLED driver\oled.c" -fo="oled.o" -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

clean : 
	$(RM) "main.o" "oled.o" "Lab1.cof" "Lab1.hex" "Lab1.map"

