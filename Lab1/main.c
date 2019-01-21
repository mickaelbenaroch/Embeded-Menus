/********************************************************************
 FileName:     main.c
 Dependencies: See INCLUDES section
 Processor:   PIC18 or PIC24 USB Microcontrollers
 Hardware:    The code is natively intended to be used on the following
        hardware platforms: PICDEM™ FS USB Demo Board, 
        PIC18F87J50 FS USB Plug-In Module, or
        Explorer 16 + PIC24 USB PIM.  The firmware may be
        modified for use on other USB platforms by editing the
        HardwareProfile.h file.
 Complier:    Microchip C18 (for PIC18) or C30 (for PIC24)
 Company:   Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the “Company”) for its PIC® Microcontroller is intended and
 supplied to you, the Company’s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
********************************************************************/


//	========================	INCLUDES	========================
#ifdef _VISUAL
#include "VisualSpecials.h"
#endif // VISUAL

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"

#include "mtouch.h"

#include "BMA150.h"

#include "oled.h"

#include "soft_start.h"


//	========================	CONFIGURATION	========================

#if defined(PIC18F46J50_PIM)
   //Watchdog Timer Enable bit:
     #pragma config WDTEN = OFF          //WDT disabled (control is placed on SWDTEN bit)
   //PLL Prescaler Selection bits:
     #pragma config PLLDIV = 3           //Divide by 3 (12 MHz oscillator input)
   //Stack Overflow/Underflow Reset Enable bit:
     #pragma config STVREN = ON            //Reset on stack overflow/underflow enabled
   //Extended Instruction Set Enable bit:
     #pragma config XINST = OFF          //Instruction set extension and Indexed Addressing mode disabled (Legacy mode)
   //CPU System Clock Postscaler:
     #pragma config CPUDIV = OSC1        //No CPU system clock divide
   //Code Protection bit:
     #pragma config CP0 = OFF            //Program memory is not code-protected
   //Oscillator Selection bits:
     #pragma config OSC = ECPLL          //HS oscillator, PLL enabled, HSPLL used by USB
   //Secondary Clock Source T1OSCEN Enforcement:
     #pragma config T1DIG = ON           //Secondary Oscillator clock source may be selected
   //Low-Power Timer1 Oscillator Enable bit:
     #pragma config LPT1OSC = OFF        //Timer1 oscillator configured for higher power operation
   //Fail-Safe Clock Monitor Enable bit:
     #pragma config FCMEN = OFF           //Fail-Safe Clock Monitor disabled
   //Two-Speed Start-up (Internal/External Oscillator Switchover) Control bit:
     #pragma config IESO = OFF           //Two-Speed Start-up disabled
   //Watchdog Timer Postscaler Select bits:
     #pragma config WDTPS = 32768        //1:32768
   //DSWDT Reference Clock Select bit:
     #pragma config DSWDTOSC = INTOSCREF //DSWDT uses INTOSC/INTRC as reference clock
   //RTCC Reference Clock Select bit:
     #pragma config RTCOSC = T1OSCREF    //RTCC uses T1OSC/T1CKI as reference clock
   //Deep Sleep BOR Enable bit:
     #pragma config DSBOREN = OFF        //Zero-Power BOR disabled in Deep Sleep (does not affect operation in non-Deep Sleep modes)
   //Deep Sleep Watchdog Timer Enable bit:
     #pragma config DSWDTEN = OFF        //Disabled
   //Deep Sleep Watchdog Timer Postscale Select bits:
     #pragma config DSWDTPS = 8192       //1:8,192 (8.5 seconds)
   //IOLOCK One-Way Set Enable bit:
     #pragma config IOL1WAY = OFF        //The IOLOCK bit (PPSCON<0>) can be set and cleared as needed
   //MSSP address mask:
     #pragma config MSSP7B_EN = MSK7     //7 Bit address masking
   //Write Protect Program Flash Pages:
     #pragma config WPFP = PAGE_1        //Write Protect Program Flash Page 0
   //Write Protection End Page (valid when WPDIS = 0):
     #pragma config WPEND = PAGE_0       //Write/Erase protect Flash Memory pages starting at page 0 and ending with page WPFP[5:0]
   //Write/Erase Protect Last Page In User Flash bit:
     #pragma config WPCFG = OFF          //Write/Erase Protection of last page Disabled
   //Write Protect Disable bit:
     #pragma config WPDIS = OFF          //WPFP[5:0], WPEND, and WPCFG bits ignored
  
#else
    #error No hardware board defined, see "HardwareProfile.h" and __FILE__
#endif



//	========================	Global VARIABLES	========================
#pragma udata
//You can define Global Data Elements here

//	========================	PRIVATE PROTOTYPES	========================
static void InitializeSystem(void);
static void ProcessIO(void);
static void UserInit(void);
static void YourHighPriorityISRCode();
static void YourLowPriorityISRCode();

BOOL CheckButtonPressed(void);

//	========================	VECTOR REMAPPING	========================
#if defined(__18CXX)
  //On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
  //the reset, high priority interrupt, and low priority interrupt
  //vectors.  However, the current Microchip USB bootloader 
  //examples are intended to occupy addresses 0x00-0x7FF or
  //0x00-0xFFF depending on which bootloader is used.  Therefore,
  //the bootloader code remaps these vectors to new locations
  //as indicated below.  This remapping is only necessary if you
  //wish to program the hex file generated from this project with
  //the USB bootloader.  If no bootloader is used, edit the
  //usb_config.h file and comment out the following defines:
  //#define PROGRAMMABLE_WITH_SD_BOOTLOADER
  
  #if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
    #define REMAPPED_RESET_VECTOR_ADDRESS     0xA000
    #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0xA008
    #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0xA018
  #else 
    #define REMAPPED_RESET_VECTOR_ADDRESS     0x00
    #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0x08
    #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0x18
  #endif
  
  #if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
  extern void _startup (void);        // See c018i.c in your C18 compiler dir
  #pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
  void _reset (void)
  {
      _asm goto _startup _endasm
  }
  #endif
  #pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
  void Remapped_High_ISR (void)
  {
       _asm goto YourHighPriorityISRCode _endasm
  }
  #pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
  void Remapped_Low_ISR (void)
  {
       _asm goto YourLowPriorityISRCode _endasm
  }
  
  #if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
  //Note: If this project is built while one of the bootloaders has
  //been defined, but then the output hex file is not programmed with
  //the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
  //As a result, if an actual interrupt was enabled and occured, the PC would jump
  //to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
  //executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
  //(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
  //would effective reset the application.
  
  //To fix this situation, we should always deliberately place a 
  //"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
  //"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
  //hex file of this project is programmed with the bootloader, these sections do not
  //get bootloaded (as they overlap the bootloader space).  If the output hex file is not
  //programmed using the bootloader, then the below goto instructions do get programmed,
  //and the hex file still works like normal.  The below section is only required to fix this
  //scenario.
  #pragma code HIGH_INTERRUPT_VECTOR = 0x08
  void High_ISR (void)
  {
       _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
  }
  #pragma code LOW_INTERRUPT_VECTOR = 0x18
  void Low_ISR (void)
  {
       _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
  }
  #endif  //end of "#if defined(||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER))"

  #pragma code
  
//	========================	Application Interrupt Service Routines	========================
  //These are your actual interrupt handling routines.
  #pragma interrupt YourHighPriorityISRCode
  void YourHighPriorityISRCode()
  {
    //Check which interrupt flag caused the interrupt.
    //Service the interrupt
    //Clear the interrupt flag
    //Etc.
  
  } //This return will be a "retfie fast", since this is in a #pragma interrupt section 
  #pragma interruptlow YourLowPriorityISRCode
  void YourLowPriorityISRCode()
  {
    //Check which interrupt flag caused the interrupt.
    //Service the interrupt
    //Clear the interrupt flag
    //Etc.
  
  } //This return will be a "retfie", since this is in a #pragma interruptlow section 
#endif




//	========================	Board Initialization Code	========================
#pragma code
#define ROM_STRING rom unsigned char*

/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the application code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/
void UserInit(void)
{

  /* Initialize the mTouch library */
  mTouchInit();

  /* Call the mTouch callibration function */
  mTouchCalibrate();

  /* Initialize the accelerometer */
  InitBma150(); 

  /* Initialize the oLED Display */
   ResetDevice();  
   FillDisplay(0x00);
   //oledPutROMString((ROM_STRING)" PIC18F Starter Kit  ",0,0);
}//end UserInit


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
	// Soft Start the APP_VDD
	while(!AppPowerReady())
		;

    #if defined(PIC18F46J50_PIM)
  //Configure all I/O pins to use digital input buffers
    ANCON0 = 0xFF;                  // Default all pins to digital
    ANCON1 = 0xFF;                  // Default all pins to digital
    #endif
       
    UserInit();

}//end InitializeSystem


//	========================	Application Code	========================

BOOL CheckButtonPressed(void)
{
    static char buttonPressed = FALSE;
    static unsigned long buttonPressCounter = 0;

    if(PORTBbits.RB0 == 0)
    {
        if(buttonPressCounter++ > 10)
        {
            buttonPressCounter = 0;
            buttonPressed = TRUE;
        }
    }
    else
    {
        if(buttonPressed == TRUE)
        {
            if(buttonPressCounter == 0)
            {
                buttonPressed = FALSE;
                return TRUE;
            }
            else
            {
                buttonPressCounter--;
            }
        }
    }

    return FALSE;
}


int accelerometer()
{
	BMA150_XYZ xyz;
	char xyArr[20] = {0};	
	BYTE lsb, msb;
	int i, val, repeat = 0;
	int counterX = 0;
	int counterY = 0;

	lsb = BMA150_ReadByte(BMA150_ACC_X_LSB); 	
	msb = BMA150_ReadByte(BMA150_ACC_X_MSB); 	
	xyz.x = 0;
	xyz.x = (short)msb << 8;
	xyz.x |= lsb;
	xyz.x >>= 6;
	if(xyz.x & 0x200)
		xyz.x |= 0xFC00;

	xyz.x = xyz.x << 2;		
	itoa(xyz.x, xyArr);
	val = atoi(xyArr);
	if(val > counterX)
		counterX = val;

	repeat = counterX / 50;
	lsb = BMA150_ReadByte(BMA150_ACC_Y_LSB); 	
	msb = BMA150_ReadByte(BMA150_ACC_Y_MSB); 	
	xyz.y = 0;
	xyz.y = (short)msb << 8;
	xyz.y |= lsb;
	xyz.y >>= 6;
	if(xyz.y & 0x200)
		xyz.y |= 0xFC00;

	xyz.x = xyz.x << 2;
	itoa(xyz.y, xyArr);
	val = atoi(xyArr);
	if(val > counterY)
		counterY = val;

	//z
	lsb = BMA150_ReadByte(BMA150_ACC_Z_LSB); //LSB	
	msb = BMA150_ReadByte(BMA150_ACC_Z_MSB); //MSB	
	xyz.z = 0;
	xyz.z = (short)msb << 8;
	xyz.z |= lsb;
	xyz.z >>= 6;
	if(xyz.z & 0x200)
		xyz.z |= 0xFC00;

	itoa(xyz.z, xyArr);
	val = atoi(xyArr);

	//chack if microchip is upside down
	if(val < -58)
	{
		counterY = 0;
		counterX = 0;
		repeat = 1;
		oledRepeatByte(0x18,5,55, repeat);
		oledRepeatByte(0x18,4,55, repeat);
		return 1;
	}
	else
	{
		return 0;
	}
}



int accelerometer2()
{
	BMA150_XYZ xyz;
	char xyArr[20] = {0};	
	BYTE lsb, msb;
	int i, val, repeat = 0;
	int counterX = 0;
	int counterY = 0;

	//accX
	for(i=13;i <= 40;i++)
		oledWriteChar1x(0x20, 4 + 0xB0, i);						//clear garbage oled parameter

	lsb = BMA150_ReadByte(BMA150_ACC_X_LSB); 	
	msb = BMA150_ReadByte(BMA150_ACC_X_MSB); 	
	xyz.x = 0;
	xyz.x = (short)msb << 8;
	xyz.x |= lsb;
	xyz.x >>= 6;
	if(xyz.x & 0x200)
		xyz.x |= 0xFC00;

	xyz.x = xyz.x << 2;		
	
	itoa(xyz.x, xyArr);
	//oledPutROMString((ROM_STRING)"X: ",4,0);
	//oledPutString(xyArr, 4, 15);
	if(xyz.x >= 0 && xyz.x <= 50)
		return 1;
	else if(xyz.x >= 51  && xyz.x <= 100)
		return 2;
	else if(xyz.x >= 101 && xyz.x <= 150)
		return 3;
	else if(xyz.x >= 151 && xyz.x <= 200)
		return 4;
	else 
		return 1;
}


char touchButtons()
{
	unsigned int left, right,scrollU, scrollD;;

	mTouchCalibrate();
	right  = mTouchReadButton(0);
	left   = mTouchReadButton(3);
	scrollU = mTouchReadButton(1);
	scrollD = mTouchReadButton(2);
	ADCON0 = 0b00010011;								//potentiometer declire again 								


	//check  scroll			
	if(scrollD > 980)
		return 'd';
	if(scrollD < 975)
		return 'x';
	if(scrollU > 965)
		return 'u';
	if(scrollU < 960)
		return 'x';

}

char touchButtons2()
{
	unsigned int left, right,scrollU, scrollD;;

	mTouchCalibrate();
	right  = mTouchReadButton(0);
	left   = mTouchReadButton(3);
	scrollU = mTouchReadButton(1);
	scrollD = mTouchReadButton(2);
	ADCON0 = 0b00010011;
	
	//check left touch
	if(left > 800)
	  return 'x';
	else
      return 'l';

}

char touchButtons3()
{
	unsigned int left, right,scrollU, scrollD;;

	mTouchCalibrate();
	right  = mTouchReadButton(0);
	left   = mTouchReadButton(3);
	scrollU = mTouchReadButton(1);
	scrollD = mTouchReadButton(2);
	ADCON0 = 0b00010011;
	
	//check right touch
	if(right > 800)
	  return 'x';
	else
      return 'r';

}


int potentiometer()
{
	int i;
	char str[30];

	ADCON0bits.CHS = 4;		
	ADCON0bits.GO = 1;	
	while(ADCON0bits.GO);

	itoa(ADRES, str);					

	//Fill the selected item in main menu bt potentimeter current value
	if(ADRES < 1000 && ADRES > 750){FillDisplayItem(0xFF,0xB2,0xB3); return 1;}
	if(ADRES < 750 && ADRES > 500){FillDisplayItem(0xFF,0xB3,0xB4); return 2;}
	if(ADRES < 500 && ADRES > 250){FillDisplayItem(0xFF,0xB4,0xB5); return 3;}
	if(ADRES < 250 && ADRES > 0){FillDisplayItem(0xFF,0xB5,0xB6); return 4;}
	
}

int potentiometerSubMenu2()
{
	int i;
	char str[30];

	ADCON0bits.CHS = 4;		
	ADCON0bits.GO = 1;	
	while(ADCON0bits.GO);

	itoa(ADRES, str);					

	//Fill the selected item in main menu bt potentimeter current value
	if(ADRES < 1000 && ADRES > 750){FillDisplayItem(0xFF,0xB2,0xB3); return 1;}
	else if(ADRES < 750 && ADRES > 600){FillDisplayItem(0xFF,0xB3,0xB4); return 2;}
	else if(ADRES < 600 && ADRES > 450){FillDisplayItem(0xFF,0xB4,0xB5); return 3;}
	else if(ADRES < 450 && ADRES > 300){FillDisplayItem(0xFF,0xB5,0xB6); return 4;}
	else if(ADRES < 300 ){FillDisplayItem(0xFF,0xB6,0xB7); return 5;}
    else if(ADRES < 150 && ADRES >0){FillDisplayItem(0xFF,0xB7,0xB8); return 6;}
	else{FillDisplayItem(0xFF,0xB2,0xB3); return 6;}
}

void DrawMainMenu(void)
{
	oledPutROMString("Welcome To Amazon    ",0,0) ;
	oledPutROMString(" ",1,0) ;
	oledPutROMString("1 - Home&Kitchen     ",2,0) ;
	oledPutROMString("2 - Electronics      ",3,0) ;
	oledPutROMString("3 - Books            ",4,0) ;	
	oledPutROMString("4 - Clothes          ",5,0) ;
	oledPutROMString("                     ",6,0) ;
	oledPutROMString("                     ",7,0);
}

void DrawSubMenu1()
{
	int res = 0;
	char button = 'q';
	int action = 0;
	int select = 0;
	char response = 'q';
	while(1)
	{
		response = touchButtons();
		if(	response == 0x75)
			break;

			button = touchButtons3();
			if(button == 'r')
			{
				action = 1;
				
				while(action == 1)
				{			
					action = drawActionOnScreen(select);	
				}				
			}
		
		oledPutROMString("Home&Kitchen         ",0,0) ;
		oledPutROMString(" ",1,0) ;
		oledPutROMString("1 - Furnitures       ",2,0) ;
		oledPutROMString("2 - Kitchen Tools    ",3,0) ;
		oledPutROMString("3 - Garden           ",4,0) ;	
		oledPutROMString("4 - Other            ",5,0) ;
		oledPutROMString("                     ",6,0) ;
		res = accelerometer2();
		switch(res)
			{
				case 1:
				FillDisplayItem2(0xFF,0xB2,0xB3);
				select = 1;
				break;
				case 2:
				FillDisplayItem2(0xFF,0xB3,0xB4);
				select = 2;
				break;
				case 3:
				FillDisplayItem2(0xFF,0xB4,0xB5);
				select = 3;				
				break;
				case 4: 
				FillDisplayItem2(0xFF,0xB5,0xB6);
				select = 4;				
				break;
			}
		
	}
}

int drawActionOnScreen(int action)
{

	int response2 = 0;
	while(1)
	{
		char str[30];
		itoa(action, str);					
		oledPutString(str, 4, 40);	
		oledPutROMString("                     ",2,0) ;
		oledPutROMString("                     ",3,0) ;
		oledPutROMString("Action   was pressed ",4,0) ;
		oledPutROMString("      press left     ",5,0) ;
		oledPutROMString("      to go back     ",6,0) ;
		oledPutROMString("                     ",7,0);

		response2 = touchButtons2();
	if(1)
	{
	char str[30];
	itoa(response2, str);					
	//oledPutString(str, 2, 0);	
	}

		if(response2 != 'x')
		{
			return 0;
		}
		else
		{
			return 1;
			break;
		}
	}
}

int DrawSubMenu2()
{
	BOOL button;
	char response;
	char response2;
	int counterDown = 0;
	int action = 0;
	int acceler = 0;
	response = 'q';
	response2 = 'q';
	while(1)
	{
			acceler = accelerometer();				
			if(acceler == 1)
				break;

			button = CheckButtonPressed();
			if(button)
			{
				action = 1;
				
				while(action == 1)
				{			
					action = drawActionOnScreen(counterDown);	

				}				
			}

		if(counterDown >=0  && counterDown <=5)
		{
			oledPutROMString("Electronics          ",0,0) ;
			oledPutROMString(" ",1,0) ;
			oledPutROMString("1.Electronics Item1  ",2,0) ;
			oledPutROMString("2.Electronics Item2  ",3,0) ;
			oledPutROMString("3.Electronics Item3  ",4,0) ;	
			oledPutROMString("4.Electronics Item4  ",5,0) ;
			oledPutROMString("5.Electronics Item5  ",6,0) ;
			oledPutROMString("6.Electronics Item6  ",7,0) ;
		}
	
		response = touchButtons();
		if(response == 0x64){
			counterDown = counterDown + 1;	
		}
			
		if(	response == 0x75)
			counterDown = counterDown - 1;

		if(counterDown >=6 && counterDown <=11 )
		{
			oledPutROMString("Electronics          ",0,0) ;
			oledPutROMString(" ",1,0) ;
			oledPutROMString("7.Electronics Item7  ",2,0) ;
			oledPutROMString("8.Electronics Item8  ",3,0) ;
			oledPutROMString("9.Electronics Item9  ",4,0) ;	
			oledPutROMString("10.Electronics Item10",5,0) ;
			oledPutROMString("11.Electronics Item11",6,0) ;
			oledPutROMString("12.Electronics Item12",7,0) ;
		}
		if(counterDown >=12 && counterDown <=15 )
		{
			oledPutROMString("Electronics          ",0,0) ;
			oledPutROMString(" ",1,0) ;
			oledPutROMString("11.Electronics Item11",2,0) ;
			oledPutROMString("12.Electronics Item12",3,0) ;
			oledPutROMString("13.Electronics Item13",4,0) ;	
			oledPutROMString("14.Electronics Item14",5,0) ;
			oledPutROMString("15.Electronics Item15",6,0) ;
			oledPutROMString("16.Electronics Item16",7,0) ;
		}

		if(counterDown >= 15)
			counterDown = 15;
			
		if(counterDown <= 0){
			counterDown = 0;
		}

		
			switch(counterDown)
			{
				case 0: case 10: case 6:
				FillDisplayItem2(0xFF,0xB2,0xB3);
				break;
				case 1: case 11: case 7:
				FillDisplayItem2(0xFF,0xB3,0xB4);
				break;
				case 2: case 12: case 8:
				FillDisplayItem2(0xFF,0xB4,0xB5);
				break;
				case 3: case 13: case 9:
				FillDisplayItem2(0xFF,0xB5,0xB6);
				break;
				case 4: case 14:
				FillDisplayItem2(0xFF,0xB6,0xB7);
				break;
				case 5:case 15:
				FillDisplayItem2(0xFF,0xB7,0xB8);
				break;
				default:
				break;
			}		
  	}
	return 0;
}

void DrawSubMenu3()
{
	char response = 'q';
	char response2 = 'q';
	char response3 = 'q';
	int counter = 0;
	int action = 0;
	BOOL button;
	while(1)
	{
		response3 = touchButtons();
		if(	response3 == 0x75)
			break;

		button = CheckButtonPressed();
			if(button)
			{
				action = 1;
				
				while(action == 1)
				{			
					action = drawActionOnScreen(counter + 1);	

				}				
			}

		oledPutROMString("Books                ",0,0) ;
		oledPutROMString(" ",1,0) ;
		oledPutROMString("1 - Romans           ",2,0) ;
		oledPutROMString("2 - Action           ",3,0) ;
		oledPutROMString("3 - Kids             ",4,0) ;	
		oledPutROMString("4 - Cooking          ",5,0) ;
		oledPutROMString("                     ",6,0);
	
	//left
	response = touchButtons2();
	//right
	response2 = touchButtons3();
	
	if(response != 'x' && counter > 0)
	{
		counter -= 1;
	}
	if(response2 != 'x')
	{
		counter +=1;
	} 
	if(counter <= 0)
		counter = 0;
	if(counter >= 3)
		counter =3;

	switch(counter)
			{
				case 0:
				FillDisplayItem(0xFF,0xB2,0xB3);
				break;
				case 1: 
				FillDisplayItem(0xFF,0xB3,0xB4);
				break;
				case 2: 
				FillDisplayItem(0xFF,0xB4,0xB5);
				break;
				case 3: 
				FillDisplayItem(0xFF,0xB5,0xB6);
				break;
				default:
				break;
			}
	}
	
}

void DrawSubMenu4()
{
char response = 'q';
	char response2 = 'q';
	char response3 = 'q';
	char response4 = 'q';
	int counter = 0;
	int action = 0;
	int action2 = 0;
	int action3 = 0;
	int selection = 0;
	BOOL button;
	BOOL button2;
	BOOL button3;
	while(1)
	{
		response3 = touchButtons();
		if(	response3 == 0x75)
			break;

		button = CheckButtonPressed();
		if(button)
		{
			action = 1;
		
			while(action == 1 && (counter == 1 || counter == 2 || counter == 3))
			{			
				action = drawActionOnScreen(counter+1);	
		
			}				

			while(action == 1 && counter == 0)
			{		
				selection = potentiometer();

				response4 = touchButtons();
				if(	response4 == 0x75)
					break;

				oledPutROMString("Men                  ",0,0);
				oledPutROMString(" ",1,0);
				oledPutROMString("1 - Casual           ",2,0);
				oledPutROMString("2 - Shoes            ",3,0);
				oledPutROMString("3 - Pants            ",4,0);	
				oledPutROMString("4 - T-Shirts         ",5,0);
				oledPutROMString("                     ",6,0);
			
					switch(selection)
					{
						case 1:
						FillDisplayItem(0xFF,0xB2,0xB3);
						break;
						case 2: 
						FillDisplayItem(0xFF,0xB3,0xB4);
						break;
						case 3: 
						FillDisplayItem(0xFF,0xB4,0xB5);
						break;
						case 4: 
						FillDisplayItem(0xFF,0xB5,0xB6);
						break;
						default:
						break;
					}
				button2 = CheckButtonPressed();
				if(button2)
				{
					action2 = 1;
					
					while(action2 == 1)
					{			
						action2 = drawActionOnScreen(selection);	
	
					}				
				}
			}				
		}


		oledPutROMString("Clothes              ",0,0);
		oledPutROMString(" ",1,0);
		oledPutROMString("1 - Men              ",2,0);
		oledPutROMString("2 - Women            ",3,0);
		oledPutROMString("3 - TRF              ",4,0);	
		oledPutROMString("4 - Kids             ",5,0);
		oledPutROMString("                     ",6,0);

		//left
		response = touchButtons2();
		//right
		response2 = touchButtons3();
		
		if(response != 'x' && counter > 0)
		{
			counter -= 1;
		}
		if(response2 != 'x')
		{
			counter +=1;
		} 
		if(counter <= 0)
			counter = 0;
		if(counter >= 3)
			counter =3;
	
		switch(counter)
		{
			case 0:
			FillDisplayItem(0xFF,0xB2,0xB3);
			break;
			case 1: 
			FillDisplayItem(0xFF,0xB3,0xB4);
			break;
			case 2: 
			FillDisplayItem(0xFF,0xB4,0xB5);
			break;
			case 3: 
			FillDisplayItem(0xFF,0xB5,0xB6);
			break;
			default:
			break;
		}
	}
}

/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
void main(void)
{
	int selection;
	BOOL button;

    InitializeSystem();

    while(1) //Main is Usualy an Endless Loop
    {

		DrawMainMenu();
		selection = potentiometer();

		button = CheckButtonPressed();
		if(button)
		{
			switch(selection)
			{
				case 1:
				DrawSubMenu1();
				break;
				case 2:
				DrawSubMenu2();
				break;
				case 3:
				DrawSubMenu3();
				break;
				case 4:
				DrawSubMenu4();
				break;
			
			}
		}
//touchButtons();
    }
}//end main


/** EOF main.c *************************************************/
//#endif
