#include <stdio.h>
#include "M051Series.h"
#include "initialization.h"
#define daLength 6
#define daHEAD 0x01
#define daEND 0xFE
#define CMD 1
#define DATA1 2
#define DATA2 3
#define CHKSUM 4
#define ADC1_BASE_ADDR  0x60010000UL
#define ADC2_BASE_ADDR  0x60012000UL
#define ADC3_BASE_ADDR  0x60014000UL
#define _8255_BASE_ADDR 0x60016000UL

#ifdef tDebug
    #define tprint(A) printf(A);
#else
    #define tprint(A) //tprint(A);
#endif

uint8_t u32array[daLength];							//appect array
uint8_t RETu32array[daLength];					//return array
uint16_t oPB;												//
uint16_t oPC;												//

//EBI Pointer
/*---------------------------------------------------------------------------------------------------------*/
/* Global file scope (static) variables                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint16_t *g_pu8NorBaseAddr;

/*------------------------------------------------ -----------------*/
/* 初始化系統時鐘*/
/*------------------------------------------------ -----------------*/
void SYS_Init(void)
{
    /* 解鎖保護的寄存器 */
    SYS_UnlockReg();

    /* 使能 XTL12M,OSC22M */
    SYSCLK->PWRCON |= SYSCLK_PWRCON_XTL12M_EN_Msk
                   | SYSCLK_PWRCON_OSC22M_EN_Msk;

    /* 等待時鐘穩定 */
    SYS_WaitingForClockReady(SYSCLK_CLKSTATUS_XTL12M_STB_Msk
                           | SYSCLK_CLKSTATUS_OSC22M_STB_Msk);

    /* 切換HCLK和SysTick的時鐘源 */
    SYSCLK->CLKSEL0 = SYSCLK_CLKSEL0_HCLK_IRC22M | SYSCLK_CLKSEL0_STCLK_XTAL;
    /* 切換IP模塊的時鐘源 */
    SYSCLK->CLKSEL1 = SYSCLK_CLKSEL1_UART_IRC22M;

    SYSCLK->CLKDIV = (0 << SYSCLK_CLKDIV_HCLK_N_Pos)
                   | (0 << SYSCLK_CLKDIV_UART_N_Pos)
                   | (0 << SYSCLK_CLKDIV_ADC_N_Pos);
    /* !!! SysTick被設定為來自"XTL12M", 當開始該時鐘時，
       請不要在SysTick->CTRL中使用SysTick_CTRL_CLKSOURCE_Msk位。 */

    /* PLL掉電模式 */
    SYSCLK->PLLCON = 0x0005C22E;

    /* 現在可以安全的關閉沒使用的時鐘了！ */
    SYSCLK->PWRCON &= ~(SYSCLK_PWRCON_OSC10K_EN_Msk);

    /* 使能外圍設備時鐘 */
    SYSCLK->APBCLK = SYSCLK_APBCLK_UART0_EN_Msk;

    /* 時鐘AHB設備對鐘 */
    SYSCLK->AHBCLK |= SYSCLK_AHBCLK_EBI_EN_Msk;

    /* 重置EBI */
    SYS->IPRSTC1 = SYS_IPRSTC1_EBI_RST_Msk;
    SYS->IPRSTC1 = 0;


    /* 重置外圍設備 */
    SYS->IPRSTC2 = SYS_IPRSTC2_UART0_RST_Msk
                 | SYS_IPRSTC2_GPIO_RST_Msk;
    SYS->IPRSTC2 = 0;

    /* 鎖定保護的寄存器 */
    SYS_LockReg();
}

/*------------------------------------------------ -----------------*/
/* 初始化IO引腳*/
/*------------------------------------------------ -----------------*/
void IO_Init(void)
{
	
	  /* 設定引腳復用功能 */
    SYS->P0_MFP = SYS_MFP_P00_AD0
                | SYS_MFP_P01_AD1
                | SYS_MFP_P02_AD2
                | SYS_MFP_P03_AD3
                | SYS_MFP_P04_AD4
                | SYS_MFP_P05_AD5
                | SYS_MFP_P06_AD6
                | SYS_MFP_P07_AD7;
//     SYS->P1_MFP = SYS_MFP_P10_nWRL
//                 | SYS_MFP_P11_nWRH;
    SYS->P2_MFP = SYS_MFP_P20_AD8
                | SYS_MFP_P21_AD9
                | SYS_MFP_P22_AD10
                | SYS_MFP_P23_AD11
                | SYS_MFP_P24_AD12
                | SYS_MFP_P25_AD13
                | SYS_MFP_P26_AD14
                | SYS_MFP_P27_AD15;
    SYS->P3_MFP = SYS_MFP_P30_RXD0
                | SYS_MFP_P31_TXD0
                | SYS_MFP_P33_MCLK
                | SYS_MFP_P36_nWR
                | SYS_MFP_P37_nRD;
    SYS->P4_MFP = SYS_MFP_P44_nCS
                | SYS_MFP_P45_ALE
                | SYS_MFP_P46_ICE_CLK
                | SYS_MFP_P47_ICE_DAT;

//P4->PMD  = 0x0000FF03;
P1->PMD  = 0x00005555;   //p01~p07 input mode
P4->PMD  = 0x0000FFC0;   //p40~p43 output mode

}

/*------------------------------------------------ -----------------*/
/* 初始化UART0 */
/*------------------------------------------------ -----------------*/
void UART0_Init(void)
{
    /* UART0設定 */
    UART0->BAUD = UART_BAUD_MODE0 | UART_BAUD_DIV_MODE0(22118400, 9600);     //Freq   , BandRate
    _UART_SET_DATA_FORMAT(UART0, UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1);
    UART0->FCR = UART_FCR_RTS_TRI_LEV_1BYTE
               | UART_FCR_RFITL_1BYTE
               | UART_FCR_RX_DIS_Msk;

}

/*-----------------------------------------------------------------*/
/* 初始化EBI (外部接口)                                            */
/*-----------------------------------------------------------------*/
void EBI_Init()
{
    EBI->EBICON = EBI_EBICON_ExttALE(7)
                | EBI_EBICON_MCLKDIV_1
                | EBI_EBICON_ExtBW16_Msk
                | EBI_EBICON_ExtEN_Msk;

    EBI->EXTIME = EBI_EXTIME_ExtIR2R(15)
                | EBI_EXTIME_ExtIW2X(15)
                | EBI_EXTIME_ExttAHD(8)
                | EBI_EXTIME_ExttACC(15);
}




void delay(unsigned int n)
{
	   uint16_t j;
     while(n--)
     for(j=0;j<1600;j++);//1600是實驗測試所得！
}

void delayns(unsigned int n)
{
	   uint16_t j;
     while(n--)
     for(j=0;j<16;j++);//1600是實驗測試所得！
}

uint8_t checkSUM()  //unsigned   char = uint8_t = 1 byte
{
	  int i;
	  uint8_t sum;
	  sum=0;
	  for (i=CMD;i<CHKSUM;i++)
			sum+=u32array[i];	  
	 	return sum;		
}

uint8_t calSUM()     //unsigned   char = uint8_t = 1 byte
{
	int i; 
	uint8_t sum;
	sum=0;
	for (i=CMD;i<CHKSUM;i++)
		sum+=RETu32array[i];
	return sum;
}
retCODE()
{
	int i;
	for (i=0;i<daLength;i++)
 	printf("%c",RETu32array[i]);
}

clrCODE()
{
	int i;
	for (i=0;i<daLength;i++) RETu32array[i]=0;
	RETu32array[0]=0x01;
	RETu32array[daLength-1]=0xFE;
}

resetDAQ()			//0
{
	    tprint("*");
	    tprint("\n reset DAQ\n");
    	RETu32array[DATA1]=0xEE;
 	 clrCODE();
	    _SYS_RESET_CPU();
	    _SYS_RESET_CHIP();  //likes MCU RESET
      init8255(); 
    	initCPU();
       RETu32array[DATA2]=0xaa;
	retCODE();
}

void readV()					//1
{     
      uint16_t read;
		  uint8_t chan;
	    uint8_t schan;
	    uint8_t jjj;
	
	    chan=u32array[DATA1];	
	    schan=u32array[DATA2];	

	for(jjj=1;jjj<=2;jjj++)
	{
switch(chan)
	{
   case 0:																														
		 g_pu8NorBaseAddr = (uint16_t *)(ADC1_BASE_ADDR + (schan << 1)) ;    
	   break;
	 case 1:
		 g_pu8NorBaseAddr = (uint16_t *)(ADC2_BASE_ADDR + (schan << 1)) ;
	   break;
	 case 2:
		 g_pu8NorBaseAddr = (uint16_t *)(ADC3_BASE_ADDR + (schan << 1)) ;
  }		
  
  *(g_pu8NorBaseAddr)=0x0000;		
	
  do{		
  }while(GPIO_PIN_DATA(4, chan)==0);
	
//Read fuction  to ADC adrees is need ! 	
	switch(chan)
	{
   case 0:
		 g_pu8NorBaseAddr = (uint16_t *)(ADC1_BASE_ADDR) ;      //read ,  a1=0  a0=0  to prevent from sleep mode
	   break;
	 case 1:
		 g_pu8NorBaseAddr = (uint16_t *)(ADC2_BASE_ADDR) ;			 //read ,  a1=0  a0=0  to prevent from sleep mode
	   break;
	 case 2:
		 g_pu8NorBaseAddr = (uint16_t *)(ADC3_BASE_ADDR) ;			 //read ,  a1=0  a0=0  to prevent from sleep mode
  }		

  delayns(1);
	read = *g_pu8NorBaseAddr & 0x0FFF;       //12bits ADC

#if tDebug
		printf("[%d]",read);  
#endif
	
    	delay(10);
	}
	
    	clrCODE();
      RETu32array[1]=1; 
 	    RETu32array[DATA1]= (uint8_t)(read >> 8);				
      RETu32array[DATA2]= read & 0x00FF;
	
 	    retCODE();
}


readI()			//2       
{
		  uint32_t chan;
	    chan=u32array[DATA1];
			tprint("\n readI\n");
	    clrCODE();
	    RETu32array[DATA1]=0x01;		//test return			
	    RETu32array[DATA2]=0x44;	  
	    retCODE();
}


void writePIO()   //3
{
   uint16_t data1;
	 uint16_t data2;
	
	 data1=u32array[DATA1];	
	 data2=u32array[DATA2];	
	
	   //P43=0;
 //SET MODE 0,PA=IN  PB=OUT  PC=OUT                                                                      
// g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0xB003UL << 1)) ;    	
// 		*(g_pu8NorBaseAddr)=0x0090;       //65424                           
                                                         	
	
g_pu8NorBaseAddr = (uint16_t *)(_8255_BASE_ADDR + (1 << 1)) ;   	//左移回來     systemADDR[16:1]    EDI[16:0]																																				//   (  <<  )(要括)
		*(g_pu8NorBaseAddr)=data1;       //PB=date1

g_pu8NorBaseAddr = (uint16_t *)(_8255_BASE_ADDR + (2 << 1)) ;   	//左移回來     systemADDR[16:1]    EDI[16:0]																																				//   (  <<  )(要括)
		*(g_pu8NorBaseAddr)=data2;       //PC=date2
	
//  g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0xB001UL << 1)) ;   	//左移回來     systemADDR[16:1]    EDI[16:0]																																				//   (  <<  )(要括)
// 		*(g_pu8NorBaseAddr)=0x0055;       //PB=55
// 	
// 	g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0xB002UL << 1)) ;   	//左移回來     systemADDR[16:1]    EDI[16:0]																																				//   (  <<  )(要括)
// 		*(g_pu8NorBaseAddr)=0x00AA;       //PC=55	
// 	// PORT A  READ 
// 	g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0xB000UL << 1)) ;   	//左移回來     systemADDR[16:1]    EDI[16:0]		
// 	 printf(" %d ",*g_pu8NorBaseAddr );

 	   clrCODE();
	   RETu32array[1]=0x03;
	   RETu32array[DATA1]=0x00;		//test return			
	   RETu32array[DATA2]=0xaa;	  
	    retCODE();
}

readPIO()  //4
{
	 uint16_t read;
	 uint8_t chan;
	 uint8_t schan;	
 //8255
// 	   P43=0;
//  //SET MODE 0,PA=IN  PB=OUT  PC=OUT
// 	g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0xB003UL << 1)) ;   	//左移回來     systemADDR[16:1]    EDI[16:0]																																				//   (  <<  )(要括)
// 		*(g_pu8NorBaseAddr)=0x0090;       //65424
// //	  
// 	g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0xB001UL << 1)) ;   	//左移回來     systemADDR[16:1]    EDI[16:0]																																				//   (  <<  )(要括)
// 		*(g_pu8NorBaseAddr)=0x0055;       //PB=55
// 	
// 	g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0xB002UL << 1)) ;   	//左移回來     systemADDR[16:1]    EDI[16:0]																																				//   (  <<  )(要括)
// 		*(g_pu8NorBaseAddr)=0x00AA;       //PC=55
	
	// PORT A  READ 
//	g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0xB000UL << 1)) ;   	//左移回來     systemADDR[16:1]    EDI[16:0]		
	g_pu8NorBaseAddr = (uint16_t *)(_8255_BASE_ADDR + (0 << 1)) ;   	//左移回來     systemADDR[16:1]    EDI[16:0]		
	 
	  read = *g_pu8NorBaseAddr;

#if tDebug		
    printf("PORTA: %d ",*g_pu8NorBaseAddr & 0x00FF );	 
#endif
 
     clrCODE();
	   RETu32array[1]=0x04;
	   RETu32array[DATA1]=read & 0x00FF;		// 8 Bytes
	   RETu32array[DATA2]=0xAA;	            
	    retCODE();
}
	

writeCPUio()  //5
{
	uint32_t chan;
	uint32_t value;
	chan=u32array[DATA1];       //ex: 36 = P3.6
	value=u32array[DATA2];      //0 or 1 
	 
     GPIO_PIN_DATA(chan>>4, chan&0xf)=value;    //ex: 36 = P3.6        
	   clrCODE();
	   RETu32array[CMD]=5;						   //test return	
     RETu32array[DATA1]=chan;	  	 //test return			
	   RETu32array[DATA2]=0xAA;
	   RETu32array[CHKSUM]=calSUM();
     retCODE();

	//tprint("\n writeCPUio\n");
}


readCPUio()      //6     
{
	uint8_t chan;
	uint8_t value;
	uint8_t result;
	    chan=u32array[DATA1];    // 01 
	    value=u32array[DATA2]; 
	
     result=GPIO_PIN_DATA(chan>>4, chan&0xf);
	   
	   clrCODE();
     RETu32array[CMD]=6;		//test return	
	   RETu32array[DATA1]=result;		//test return			
	   RETu32array[DATA2]=0xAA;
     RETu32array[CHKSUM]=calSUM();
	    retCODE();
	tprint("\n readCPUio\n");
}

/* sort data
sort()
{
	  uchar i, num, temp;
	  for (num = 11; num > 0; num--) {
      for (i = 0; i < num; i++) {
        if (I_RAM[i] > I_RAM[i + 1]) {
          temp = I_RAM[i];
          I_RAM[i] = I_RAM[i + 1];
          I_RAM[i + 1] = temp;
    } } }
}
*/

init8255()
{
	//8255   
	   P43=0;   			//8255 RESET
 //SET MODE 0,PA=IN  PB=OUT  PC=OUT
	g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0xB003UL << 1)) ;   	//左移回來     systemADDR[16:1]    EDI[16:0]																																				//   (  <<  )(要括)
		*(g_pu8NorBaseAddr)=0x0090;        //65424 
}

test8255()
{
// 	//8255
// 	   P43=0;
//  //SET MODE 0,PA=IN  PB=OUT  PC=OUT
// 	g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0xB003UL << 1)) ;   	//左移回來     systemADDR[16:1]    EDI[16:0]																																				//   (  <<  )(要括)
// 		*(g_pu8NorBaseAddr)=0x0090;       //65424
//	  
		//8255
	   //P43=0;
 //SET MODE 0,PA=IN  PB=OUT  PC=OUT                                                                                                        
	//g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0xB003UL << 1)) ;    	
//		*(g_pu8NorBaseAddr)=0x0090;       //65424      Controll Word                      
                                                                                                               
	g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0xB001UL << 1)) ;    	
		*(g_pu8NorBaseAddr)=0x0055;       //PB=55                           
	                                                                      
	g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0xB002UL << 1)) ;    	
		*(g_pu8NorBaseAddr)=0x00AA;       //PC=AA                         
	                                                                      
	// PORT A  READ                                                       
	g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0xB000UL << 1)) ;     
	//printf(" %d ",*g_pu8NorBaseAddr);                                  	 	
}

testADC()
{
 uint16_t read;
 uint32_t reads[3];
 uint8_t j;	

//ADC
g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0x9000UL << 1)) ;   	// write左移回來     systemADDR[16:1]  ->  EDI[16:0]    Left Shift A1 ,A0 
 																											 						  //write  1  0   have RefV 
	*(g_pu8NorBaseAddr)=0x0000;

//for (j=0;j<3;j++)
//{		
 do{
	#if tDebug
     printf("testADC busy pin");
	#endif
 }while(P41==0);

 g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0x9000UL << 1)) ;     //read a1=0  a0=0  to prevent from sleep mode
 //read = *g_pu8NorBaseAdd & 0x0FFF;
 //reads[i]=read;
 
  delayns(1);
	printf(" %d ",*g_pu8NorBaseAddr  & 0x0FFF );    //指標++ 
	delay(1000); 
//}

// 	  uchar i, num, temp;
// 	  for (num = 2; num > 0; num--) {
//       for (i = 0; i < num; i++) {
//         if (I_RAM[i] > I_RAM[i + 1]) {
//           temp = I_RAM[i];
//           I_RAM[i] = I_RAM[i + 1];
//           I_RAM[i + 1] = temp;
//     } } }

//printf(" %d %d %d ",reads[0],reads[1],reads[2] );    //指標++
// printf(" %d %d %d ",reads[0],reads[1],reads[2] );    //指標++
}

void printlist(int list[],int n)
{
   int i;
   for(i=0;i<n;i++)
      printf("%d\t",list[i]);
}

testADC_MID()
{
 uint16_t read;
 uint16_t reads[3];   //sort   must be same bits 
 uint8_t j;	
 uint16_t i, num, temp;

for (j=0;j<3;j++)
{	
//ADC
g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0x8001UL << 1)) ;   	// write左移回來     systemADDR[16:1]  ->  EDI[16:0]    Left Shift A1 ,A0 
 																											 						  //write  1  0   have RefV 
	*(g_pu8NorBaseAddr)=0x0000;

	
 do{
	#if tDebug
     printf("testADC busy pin");
	#endif
 }while(P40==0);

 g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0x8000UL << 1)) ;     //read a1=0  a0=0  to prevent from sleep mode
 read = *g_pu8NorBaseAddr & 0x0FFF;
 reads[j]=read;
 
  delayns(1);
	//printf(" %d ",*g_pu8NorBaseAddr  & 0x0FFF );    //指標++ 
	delay(1000); 
}

//bobule sort	
	  for (num = 2; num > 0; num--) {
      for (i = 0; i < num; i++) {
        if (reads[i] > reads[i + 1]) {
          temp = reads[i];
          reads[i] = reads[i + 1];
          reads[i + 1] = temp;
    } } }


printf("[ %d %d %d ] ",reads[0],reads[1],reads[2] );    //由小排到大

}


initCPU()
{
	g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0xB001UL << 1)) ;    	
	 *(g_pu8NorBaseAddr)=0x0003;       //PB=00                           


	g_pu8NorBaseAddr = (uint16_t *)(0x60000000UL + (0xB002UL << 1)) ;    	
	 *(g_pu8NorBaseAddr)=0x000C;       //PC=0C
  //P10=1;
	P10=0;   //第2片之後改為0
  P11=0;
  P12=0;
  P13=0;
	//LED
	P34=0;
	P35=1;
	//---
	
}

int32_t main(void)
{ 
	uint32_t u32Item;
	int i,j;	i=0,j=0;
    SYS_Init();
    IO_Init();
    UART0_Init();
		EBI_Init();
init8255();
initCPU();
 clrCODE();

//while(1)
//{
	//testADC();
	//testADC_MID();
    //test8255();
	//printf("test 8255");
//}

	// 	printf("test......");

 while(u32Item!=27)
 { 
   //tprint("\nPlease input DQA String(6Bytes)\n");
 
	 u32array[0]=getchar();             //fetch header 
	 if (u32array[0]==0x01)	
	 {
	 
	 for (i=CMD;i<daLength;i++) 
	 u32array[i]=getchar();
	 
// 	if(u32array[CHKSUM]!=checkSUM())     //check sum
// 		printf("error chksum:[%d]",checkSUM());
// 		   clrCODE();
// 					  RETu32array[DATA1]=0xaa;
// 					  RETu32array[DATA2]=0xaa;
// 					  retCODE();

 } 
	 

 
 //detect HEAD & END bytes.
		 if (u32array[0]==daHEAD && u32array[daLength-1]==daEND ) 
	 {
		   
		  switch (u32array[CMD])
				{
					case 0x00: 
				    resetDAQ();
					  break;
					case 0x01:
						//tprint("\nREAD V \n");
			         readV();
					  break;
					case 0x02: 
						//tprint("\nREAD I \n");
					  readI();
					  break;
					case 0x03:
						//tprint("\nWrite PIO V \n");
					  writePIO();
					  break;
					case 0x04:
						//tprint("\nRead PIO I \n");
					  readPIO();
					  break;
					case 0x05:
						//tprint("\nWrite CPU IO\n");
					  writeCPUio();
					  break;
					case 0x06: 
						//tprint("\nRead CPU IO\n");
					  readCPUio();
					  break;					
					default:       //x07-0xFF  no use ...
						printf("error");
					  clrCODE();
					  RETu32array[DATA1]=0xaa;
					  RETu32array[DATA2]=0xaa;
					  retCODE();
				}
			//tprint("\nDQA Queery is  ok\n");
// 				fine();
	 }
 }
    return 0;
}