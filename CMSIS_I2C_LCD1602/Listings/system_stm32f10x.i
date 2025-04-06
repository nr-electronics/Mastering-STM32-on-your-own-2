# 1 "RTE\\Device\\STM32F103C8\\system_stm32f10x.c"


















































 



 



   
  


 

# 1 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"






































 



 



 
    






  


 
  


 

# 74 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"


















 










 
# 1 ".\\RTE\\_Target_1\\RTE_Components.h"







 







 




# 106 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"












            
# 126 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"




 










 
# 150 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 



 



 
# 170 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"




 
typedef enum IRQn
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      

 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMPER_IRQn                 = 2,       
  RTC_IRQn                    = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Channel1_IRQn          = 11,      
  DMA1_Channel2_IRQn          = 12,      
  DMA1_Channel3_IRQn          = 13,      
  DMA1_Channel4_IRQn          = 14,      
  DMA1_Channel5_IRQn          = 15,      
  DMA1_Channel6_IRQn          = 16,      
  DMA1_Channel7_IRQn          = 17,      

# 229 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 250 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"


  ADC1_2_IRQn                 = 18,      
  USB_HP_CAN1_TX_IRQn         = 19,      
  USB_LP_CAN1_RX0_IRQn        = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_IRQn               = 24,      
  TIM1_UP_IRQn                = 25,      
  TIM1_TRG_COM_IRQn           = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,      
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTCAlarm_IRQn               = 41,      
  USBWakeUp_IRQn              = 42       


# 304 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 349 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 389 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 434 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 480 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"
} IRQn_Type;



 

# 1 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\core_cm3.h"
 




 
















 










# 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
# 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











# 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
# 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



# 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











# 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
# 35 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\core_cm3.h"

















 




 



 

# 1 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\cmsis_version.h"
 




 
















 










 
# 64 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\core_cm3.h"

 









 







# 114 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\core_cm3.h"

# 1 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\cmsis_compiler.h"
 




 
















 




# 29 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\cmsis_compiler.h"



 
# 1 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\cmsis_armcc.h"
 




 
















 









 













   
   
   

 




 
# 111 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\cmsis_armcc.h"

 





















 



 




 






 







 






 








 






 






 








 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 

__attribute__((section(".revsh_text"))) static __inline __asm int16_t __REVSH(int16_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 
# 277 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\cmsis_armcc.h"







 











 












 












 














 














 














 










 









 









 









 

__attribute__((section(".rrx_text"))) static __inline __asm uint32_t __RRX(uint32_t value)
{
  rrx r0, r0
  bx lr
}








 








 








 








 








 








 


# 525 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\cmsis_armcc.h"

   


 



 





 
 






 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
  __isb(0xF);
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}









 







 







 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}






 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xFFU);
}







 
static __inline void __set_BASEPRI_MAX(uint32_t basePri)
{
  register uint32_t __regBasePriMax      __asm("basepri_max");
  __regBasePriMax = (basePri & 0xFFU);
}






 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}






 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1U);
}









 
static __inline uint32_t __get_FPSCR(void)
{





   return(0U);

}






 
static __inline void __set_FPSCR(uint32_t fpscr)
{





  (void)fpscr;

}


 


 



 

# 885 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\cmsis_armcc.h"
 


# 35 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\cmsis_compiler.h"




 
# 280 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\cmsis_compiler.h"




# 116 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\core_cm3.h"

















 
# 160 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\core_cm3.h"

 






 
# 176 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\core_cm3.h"

 




 












 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:27;               
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 


















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:1;                
    uint32_t ICI_IT_1:6;                  
    uint32_t _reserved1:8;                
    uint32_t T:1;                         
    uint32_t ICI_IT_2:2;                  
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 






























 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t _reserved1:30;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 







 



 
typedef struct
{
  volatile uint32_t ISER[8U];                
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];                
        uint32_t RESERVED1[24U];
  volatile uint32_t ISPR[8U];                
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];                
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];                
        uint32_t RESERVED4[56U];
  volatile uint8_t  IP[240U];                
        uint32_t RESERVED5[644U];
  volatile  uint32_t STIR;                    
}  NVIC_Type;

 



 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    
  volatile uint32_t VTOR;                    
  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
  volatile uint8_t  SHP[12U];                
  volatile uint32_t SHCSR;                   
  volatile uint32_t CFSR;                    
  volatile uint32_t HFSR;                    
  volatile uint32_t DFSR;                    
  volatile uint32_t MMFAR;                   
  volatile uint32_t BFAR;                    
  volatile uint32_t AFSR;                    
  volatile const  uint32_t PFR[2U];                 
  volatile const  uint32_t DFR;                     
  volatile const  uint32_t ADR;                     
  volatile const  uint32_t MMFR[4U];                
  volatile const  uint32_t ISAR[5U];                
        uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;                   
} SCB_Type;

 















 






























 




# 462 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\core_cm3.h"

 





















 









 


















 










































 









 















 


















 


















 









 















 







 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile const  uint32_t ICTR;                    

  volatile uint32_t ACTLR;                   



} SCnSCB_Type;

 



 

















 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 







 



 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                  
    volatile  uint16_t   u16;                 
    volatile  uint32_t   u32;                 
  }  PORT [32U];                          
        uint32_t RESERVED0[864U];
  volatile uint32_t TER;                     
        uint32_t RESERVED1[15U];
  volatile uint32_t TPR;                     
        uint32_t RESERVED2[15U];
  volatile uint32_t TCR;                     
        uint32_t RESERVED3[32U];
        uint32_t RESERVED4[43U];
  volatile  uint32_t LAR;                     
  volatile const  uint32_t LSR;                     
        uint32_t RESERVED5[6U];
  volatile const  uint32_t PID4;                    
  volatile const  uint32_t PID5;                    
  volatile const  uint32_t PID6;                    
  volatile const  uint32_t PID7;                    
  volatile const  uint32_t PID0;                    
  volatile const  uint32_t PID1;                    
  volatile const  uint32_t PID2;                    
  volatile const  uint32_t PID3;                    
  volatile const  uint32_t CID0;                    
  volatile const  uint32_t CID1;                    
  volatile const  uint32_t CID2;                    
  volatile const  uint32_t CID3;                    
} ITM_Type;

 



 



























 









   







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t CYCCNT;                  
  volatile uint32_t CPICNT;                  
  volatile uint32_t EXCCNT;                  
  volatile uint32_t SLEEPCNT;                
  volatile uint32_t LSUCNT;                  
  volatile uint32_t FOLDCNT;                 
  volatile const  uint32_t PCSR;                    
  volatile uint32_t COMP0;                   
  volatile uint32_t MASK0;                   
  volatile uint32_t FUNCTION0;               
        uint32_t RESERVED0[1U];
  volatile uint32_t COMP1;                   
  volatile uint32_t MASK1;                   
  volatile uint32_t FUNCTION1;               
        uint32_t RESERVED1[1U];
  volatile uint32_t COMP2;                   
  volatile uint32_t MASK2;                   
  volatile uint32_t FUNCTION2;               
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP3;                   
  volatile uint32_t MASK3;                   
  volatile uint32_t FUNCTION3;               
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   







 



 
typedef struct
{
  volatile const  uint32_t SSPSR;                   
  volatile uint32_t CSPSR;                   
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;                    
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;                    
        uint32_t RESERVED2[131U];
  volatile const  uint32_t FFSR;                    
  volatile uint32_t FFCR;                    
  volatile const  uint32_t FSCR;                    
        uint32_t RESERVED3[759U];
  volatile const  uint32_t TRIGGER;                 
  volatile const  uint32_t FIFO0;                   
  volatile const  uint32_t ITATBCTR2;               
        uint32_t RESERVED4[1U];
  volatile const  uint32_t ITATBCTR0;               
  volatile const  uint32_t FIFO1;                   
  volatile uint32_t ITCTRL;                  
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;                
  volatile uint32_t CLAIMCLR;                
        uint32_t RESERVED7[8U];
  volatile const  uint32_t DEVID;                   
  volatile const  uint32_t DEVTYPE;                 
} TPI_Type;

 



 



 












 






 



 





















 






 





















 






 



 


















 






   


# 1239 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\core_cm3.h"







 



 
typedef struct
{
  volatile uint32_t DHCSR;                   
  volatile  uint32_t DCRSR;                   
  volatile uint32_t DCRDR;                   
  volatile uint32_t DEMCR;                   
} CoreDebug_Type;

 




































 






 







































 







 






 







 


 







 

 
# 1388 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\core_cm3.h"

# 1397 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\core_cm3.h"






 










 


 



 





 

# 1448 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\core_cm3.h"

# 1458 "C:\\Arm\\Packs\\ARM\\CMSIS\\5.8.0\\CMSIS\\Core\\Include\\core_cm3.h"




 













 
static __inline void __NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);              

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((uint32_t)((0xFFFFUL << 16U) | (7UL << 8U)));  
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << 16U) |
                (PriorityGroupTmp << 8U)  );               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}






 
static __inline uint32_t __NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) >> 8U));
}







 
static __inline void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    __memory_changed();
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __memory_changed();
  }
}









 
static __inline uint32_t __NVIC_GetEnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}







 
static __inline void __NVIC_DisableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __dsb(0xF);
    __isb(0xF);
  }
}









 
static __inline uint32_t __NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}







 
static __inline void __NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}







 
static __inline void __NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}









 
static __inline uint32_t __NVIC_GetActive(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}










 
static __inline void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)IRQn)]               = (uint8_t)((priority << (8U - 4)) & (uint32_t)0xFFUL);
  }
  else
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4)) & (uint32_t)0xFFUL);
  }
}










 
static __inline uint32_t __NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) >= 0)
  {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)IRQn)]               >> (8U - 4)));
  }
  else
  {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] >> (8U - 4)));
  }
}












 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4)) ? (uint32_t)(4) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}












 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4)) ? (uint32_t)(4) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority     = (Priority                   ) & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL);
}










 
static __inline void __NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{
  uint32_t *vectors = (uint32_t *)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;
  vectors[(int32_t)IRQn + 16] = vector;
   
}









 
static __inline uint32_t __NVIC_GetVector(IRQn_Type IRQn)
{
  uint32_t *vectors = (uint32_t *)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;
  return vectors[(int32_t)IRQn + 16];
}





 
__declspec(noreturn) static __inline void __NVIC_SystemReset(void)
{
  __dsb(0xF);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = (uint32_t)((0x5FAUL << 16U)    |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) |
                            (1UL << 2U)    );          
  __dsb(0xF);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 


 








 





 








 
static __inline uint32_t SCB_GetFPUType(void)
{
    return 0U;            
}


 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  __NVIC_SetPriority (SysTick_IRQn, (1UL << 4) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 



 





 

extern volatile int32_t ITM_RxBuffer;                               










 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&       
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL               ) != 0UL)   )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0U].u32 == 0UL)
    {
      __nop();
    }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}







 
static __inline int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;                            

  if (ITM_RxBuffer != ((int32_t)0x5AA55AA5U))
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = ((int32_t)0x5AA55AA5U);        
  }

  return (ch);
}







 
static __inline int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == ((int32_t)0x5AA55AA5U))
  {
    return (0);                               
  }
  else
  {
    return (1);                               
  }
}

 










# 487 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"
# 1 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\system_stm32f10x.h"


















 



 



   
  


 









 



 




 

extern uint32_t SystemCoreClock;           



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
# 488 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"
# 489 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



   

 
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;   
typedef const int16_t sc16;   
typedef const int8_t sc8;    

typedef volatile int32_t  vs32;
typedef volatile int16_t  vs16;
typedef volatile int8_t   vs8;

typedef volatile const int32_t vsc32;   
typedef volatile const int16_t vsc16;   
typedef volatile const int8_t vsc8;    

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;   
typedef const uint16_t uc16;   
typedef const uint8_t uc8;    

typedef volatile uint32_t  vu32;
typedef volatile uint16_t vu16;
typedef volatile uint8_t  vu8;

typedef volatile const uint32_t vuc32;   
typedef volatile const uint16_t vuc16;   
typedef volatile const uint8_t vuc8;    

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

 





 



    



 

typedef struct
{
  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
} ADC_TypeDef;



 

typedef struct
{
  uint32_t  RESERVED0;
  volatile uint16_t DR1;
  uint16_t  RESERVED1;
  volatile uint16_t DR2;
  uint16_t  RESERVED2;
  volatile uint16_t DR3;
  uint16_t  RESERVED3;
  volatile uint16_t DR4;
  uint16_t  RESERVED4;
  volatile uint16_t DR5;
  uint16_t  RESERVED5;
  volatile uint16_t DR6;
  uint16_t  RESERVED6;
  volatile uint16_t DR7;
  uint16_t  RESERVED7;
  volatile uint16_t DR8;
  uint16_t  RESERVED8;
  volatile uint16_t DR9;
  uint16_t  RESERVED9;
  volatile uint16_t DR10;
  uint16_t  RESERVED10; 
  volatile uint16_t RTCCR;
  uint16_t  RESERVED11;
  volatile uint16_t CR;
  uint16_t  RESERVED12;
  volatile uint16_t CSR;
  uint16_t  RESERVED13[5];
  volatile uint16_t DR11;
  uint16_t  RESERVED14;
  volatile uint16_t DR12;
  uint16_t  RESERVED15;
  volatile uint16_t DR13;
  uint16_t  RESERVED16;
  volatile uint16_t DR14;
  uint16_t  RESERVED17;
  volatile uint16_t DR15;
  uint16_t  RESERVED18;
  volatile uint16_t DR16;
  uint16_t  RESERVED19;
  volatile uint16_t DR17;
  uint16_t  RESERVED20;
  volatile uint16_t DR18;
  uint16_t  RESERVED21;
  volatile uint16_t DR19;
  uint16_t  RESERVED22;
  volatile uint16_t DR20;
  uint16_t  RESERVED23;
  volatile uint16_t DR21;
  uint16_t  RESERVED24;
  volatile uint16_t DR22;
  uint16_t  RESERVED25;
  volatile uint16_t DR23;
  uint16_t  RESERVED26;
  volatile uint16_t DR24;
  uint16_t  RESERVED27;
  volatile uint16_t DR25;
  uint16_t  RESERVED28;
  volatile uint16_t DR26;
  uint16_t  RESERVED29;
  volatile uint16_t DR27;
  uint16_t  RESERVED30;
  volatile uint16_t DR28;
  uint16_t  RESERVED31;
  volatile uint16_t DR29;
  uint16_t  RESERVED32;
  volatile uint16_t DR30;
  uint16_t  RESERVED33; 
  volatile uint16_t DR31;
  uint16_t  RESERVED34;
  volatile uint16_t DR32;
  uint16_t  RESERVED35;
  volatile uint16_t DR33;
  uint16_t  RESERVED36;
  volatile uint16_t DR34;
  uint16_t  RESERVED37;
  volatile uint16_t DR35;
  uint16_t  RESERVED38;
  volatile uint16_t DR36;
  uint16_t  RESERVED39;
  volatile uint16_t DR37;
  uint16_t  RESERVED40;
  volatile uint16_t DR38;
  uint16_t  RESERVED41;
  volatile uint16_t DR39;
  uint16_t  RESERVED42;
  volatile uint16_t DR40;
  uint16_t  RESERVED43;
  volatile uint16_t DR41;
  uint16_t  RESERVED44;
  volatile uint16_t DR42;
  uint16_t  RESERVED45;    
} BKP_TypeDef;
  


 

typedef struct
{
  volatile uint32_t TIR;
  volatile uint32_t TDTR;
  volatile uint32_t TDLR;
  volatile uint32_t TDHR;
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;
  volatile uint32_t RDTR;
  volatile uint32_t RDLR;
  volatile uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;
  volatile uint32_t FR2;
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t MCR;
  volatile uint32_t MSR;
  volatile uint32_t TSR;
  volatile uint32_t RF0R;
  volatile uint32_t RF1R;
  volatile uint32_t IER;
  volatile uint32_t ESR;
  volatile uint32_t BTR;
  uint32_t  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t  RESERVED1[12];
  volatile uint32_t FMR;
  volatile uint32_t FM1R;
  uint32_t  RESERVED2;
  volatile uint32_t FS1R;
  uint32_t  RESERVED3;
  volatile uint32_t FFA1R;
  uint32_t  RESERVED4;
  volatile uint32_t FA1R;
  uint32_t  RESERVED5[8];

  CAN_FilterRegister_TypeDef sFilterRegister[14];



} CAN_TypeDef;



 
typedef struct
{
  volatile uint32_t CFGR;
  volatile uint32_t OAR;
  volatile uint32_t PRES;
  volatile uint32_t ESR;
  volatile uint32_t CSR;
  volatile uint32_t TXD;
  volatile uint32_t RXD;  
} CEC_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;
  volatile uint8_t  IDR;
  uint8_t   RESERVED0;
  uint16_t  RESERVED1;
  volatile uint32_t CR;
} CRC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t SWTRIGR;
  volatile uint32_t DHR12R1;
  volatile uint32_t DHR12L1;
  volatile uint32_t DHR8R1;
  volatile uint32_t DHR12R2;
  volatile uint32_t DHR12L2;
  volatile uint32_t DHR8R2;
  volatile uint32_t DHR12RD;
  volatile uint32_t DHR12LD;
  volatile uint32_t DHR8RD;
  volatile uint32_t DOR1;
  volatile uint32_t DOR2;



} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;
  volatile uint32_t CR;	
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CCR;
  volatile uint32_t CNDTR;
  volatile uint32_t CPAR;
  volatile uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  volatile uint32_t ISR;
  volatile uint32_t IFCR;
} DMA_TypeDef;



 

typedef struct
{
  volatile uint32_t MACCR;
  volatile uint32_t MACFFR;
  volatile uint32_t MACHTHR;
  volatile uint32_t MACHTLR;
  volatile uint32_t MACMIIAR;
  volatile uint32_t MACMIIDR;
  volatile uint32_t MACFCR;
  volatile uint32_t MACVLANTR;              
       uint32_t RESERVED0[2];
  volatile uint32_t MACRWUFFR;              
  volatile uint32_t MACPMTCSR;
       uint32_t RESERVED1[2];
  volatile uint32_t MACSR;                  
  volatile uint32_t MACIMR;
  volatile uint32_t MACA0HR;
  volatile uint32_t MACA0LR;
  volatile uint32_t MACA1HR;
  volatile uint32_t MACA1LR;
  volatile uint32_t MACA2HR;
  volatile uint32_t MACA2LR;
  volatile uint32_t MACA3HR;
  volatile uint32_t MACA3LR;                
       uint32_t RESERVED2[40];
  volatile uint32_t MMCCR;                  
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;                
       uint32_t RESERVED3[14];
  volatile uint32_t MMCTGFSCCR;             
  volatile uint32_t MMCTGFMSCCR;
       uint32_t RESERVED4[5];
  volatile uint32_t MMCTGFCR;
       uint32_t RESERVED5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
       uint32_t RESERVED6[10];
  volatile uint32_t MMCRGUFCR;
       uint32_t RESERVED7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
       uint32_t RESERVED8[567];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
       uint32_t RESERVED9[9];
  volatile uint32_t DMACHTDR;
  volatile uint32_t DMACHRDR;
  volatile uint32_t DMACHTBAR;
  volatile uint32_t DMACHRBAR;
} ETH_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
# 928 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"
} FLASH_TypeDef;



 
  
typedef struct
{
  volatile uint16_t RDP;
  volatile uint16_t USER;
  volatile uint16_t Data0;
  volatile uint16_t Data1;
  volatile uint16_t WRP0;
  volatile uint16_t WRP1;
  volatile uint16_t WRP2;
  volatile uint16_t WRP3;
} OB_TypeDef;



 

typedef struct
{
  volatile uint32_t BTCR[8];   
} FSMC_Bank1_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t BWTR[7];
} FSMC_Bank1E_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR2;
  volatile uint32_t SR2;
  volatile uint32_t PMEM2;
  volatile uint32_t PATT2;
  uint32_t  RESERVED0;   
  volatile uint32_t ECCR2; 
} FSMC_Bank2_TypeDef;  



 
  
typedef struct
{
  volatile uint32_t PCR3;
  volatile uint32_t SR3;
  volatile uint32_t PMEM3;
  volatile uint32_t PATT3;
  uint32_t  RESERVED0;   
  volatile uint32_t ECCR3; 
} FSMC_Bank3_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t PCR4;
  volatile uint32_t SR4;
  volatile uint32_t PMEM4;
  volatile uint32_t PATT4;
  volatile uint32_t PIO4; 
} FSMC_Bank4_TypeDef; 



 

typedef struct
{
  volatile uint32_t CRL;
  volatile uint32_t CRH;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t BRR;
  volatile uint32_t LCKR;
} GPIO_TypeDef;



 

typedef struct
{
  volatile uint32_t EVCR;
  volatile uint32_t MAPR;
  volatile uint32_t EXTICR[4];
  uint32_t RESERVED0;
  volatile uint32_t MAPR2;  
} AFIO_TypeDef;


 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t OAR1;
  uint16_t  RESERVED2;
  volatile uint16_t OAR2;
  uint16_t  RESERVED3;
  volatile uint16_t DR;
  uint16_t  RESERVED4;
  volatile uint16_t SR1;
  uint16_t  RESERVED5;
  volatile uint16_t SR2;
  uint16_t  RESERVED6;
  volatile uint16_t CCR;
  uint16_t  RESERVED7;
  volatile uint16_t TRISE;
  uint16_t  RESERVED8;
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
} IWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CSR;
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;










} RCC_TypeDef;



 

typedef struct
{
  volatile uint16_t CRH;
  uint16_t  RESERVED0;
  volatile uint16_t CRL;
  uint16_t  RESERVED1;
  volatile uint16_t PRLH;
  uint16_t  RESERVED2;
  volatile uint16_t PRLL;
  uint16_t  RESERVED3;
  volatile uint16_t DIVH;
  uint16_t  RESERVED4;
  volatile uint16_t DIVL;
  uint16_t  RESERVED5;
  volatile uint16_t CNTH;
  uint16_t  RESERVED6;
  volatile uint16_t CNTL;
  uint16_t  RESERVED7;
  volatile uint16_t ALRH;
  uint16_t  RESERVED8;
  volatile uint16_t ALRL;
  uint16_t  RESERVED9;
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;
  volatile uint32_t CLKCR;
  volatile uint32_t ARG;
  volatile uint32_t CMD;
  volatile const uint32_t RESPCMD;
  volatile const uint32_t RESP1;
  volatile const uint32_t RESP2;
  volatile const uint32_t RESP3;
  volatile const uint32_t RESP4;
  volatile uint32_t DTIMER;
  volatile uint32_t DLEN;
  volatile uint32_t DCTRL;
  volatile const uint32_t DCOUNT;
  volatile const uint32_t STA;
  volatile uint32_t ICR;
  volatile uint32_t MASK;
  uint32_t  RESERVED0[2];
  volatile const uint32_t FIFOCNT;
  uint32_t  RESERVED1[13];
  volatile uint32_t FIFO;
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SR;
  uint16_t  RESERVED2;
  volatile uint16_t DR;
  uint16_t  RESERVED3;
  volatile uint16_t CRCPR;
  uint16_t  RESERVED4;
  volatile uint16_t RXCRCR;
  uint16_t  RESERVED5;
  volatile uint16_t TXCRCR;
  uint16_t  RESERVED6;
  volatile uint16_t I2SCFGR;
  uint16_t  RESERVED7;
  volatile uint16_t I2SPR;
  uint16_t  RESERVED8;  
} SPI_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SMCR;
  uint16_t  RESERVED2;
  volatile uint16_t DIER;
  uint16_t  RESERVED3;
  volatile uint16_t SR;
  uint16_t  RESERVED4;
  volatile uint16_t EGR;
  uint16_t  RESERVED5;
  volatile uint16_t CCMR1;
  uint16_t  RESERVED6;
  volatile uint16_t CCMR2;
  uint16_t  RESERVED7;
  volatile uint16_t CCER;
  uint16_t  RESERVED8;
  volatile uint16_t CNT;
  uint16_t  RESERVED9;
  volatile uint16_t PSC;
  uint16_t  RESERVED10;
  volatile uint16_t ARR;
  uint16_t  RESERVED11;
  volatile uint16_t RCR;
  uint16_t  RESERVED12;
  volatile uint16_t CCR1;
  uint16_t  RESERVED13;
  volatile uint16_t CCR2;
  uint16_t  RESERVED14;
  volatile uint16_t CCR3;
  uint16_t  RESERVED15;
  volatile uint16_t CCR4;
  uint16_t  RESERVED16;
  volatile uint16_t BDTR;
  uint16_t  RESERVED17;
  volatile uint16_t DCR;
  uint16_t  RESERVED18;
  volatile uint16_t DMAR;
  uint16_t  RESERVED19;
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint16_t SR;
  uint16_t  RESERVED0;
  volatile uint16_t DR;
  uint16_t  RESERVED1;
  volatile uint16_t BRR;
  uint16_t  RESERVED2;
  volatile uint16_t CR1;
  uint16_t  RESERVED3;
  volatile uint16_t CR2;
  uint16_t  RESERVED4;
  volatile uint16_t CR3;
  uint16_t  RESERVED5;
  volatile uint16_t GTPR;
  uint16_t  RESERVED6;
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
} WWDG_TypeDef;



 
  


 











 




# 1320 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 1343 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



# 1362 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"




















 
  


   

# 1462 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 

 



 



 


 
 
 
 
 

 











 
# 1523 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"




 





 
 
 
 
 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 





 



 






 
 
 
 
 

 
# 1699 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 1706 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
 








 








 






# 1742 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 











 











 













 






# 1858 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"




# 1878 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 





# 1891 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 1910 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 1919 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 1927 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



















# 1952 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"












 













# 1984 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"





# 1998 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 2005 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 2015 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"











 


















# 2051 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 2059 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



















# 2084 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"












 













# 2116 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"





# 2130 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 2137 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 2147 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"











 








 








   
# 2186 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 2281 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 2308 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"
 
 
 
 
 
 

 




































































 




































































 
# 2470 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 2488 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 2506 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 2523 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 2541 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 2560 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 

 






 
# 2587 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"






 








 









 








 








 









 










 




# 2662 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 










# 2693 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 





 
# 2708 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 2717 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

   
# 2726 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 2735 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 





 
# 2750 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 2759 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

   
# 2768 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 2777 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 





 
# 2792 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 2801 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

   
# 2810 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 2819 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 





 
# 2834 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 2843 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

   
# 2852 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 2861 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 2870 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 2879 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 2889 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
 
 
 
 

 





 


 


 




 
 
 
 
 

 
# 2953 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 2988 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 3023 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 3058 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 3093 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 





 





 





 





 





 





 





 





 






 
# 3160 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 



 









 
# 3184 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"




 




 
# 3200 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 





 
# 3222 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
 





 
# 3237 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"
 
# 3244 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 




 






 


 


 


 
 
 
 
 

 
# 3293 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 3315 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 3337 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 3359 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 3381 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 3403 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
 
 
 
 

 
# 3439 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 3469 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 3479 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"















 
# 3503 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"















 
# 3527 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"















 
# 3551 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"















 
# 3575 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"















 
# 3599 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"















 
# 3623 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"















 


 


 


 


 


 


 


 


 


 



 


 


 



 


 


 


 



 


 


 


 


 
 
 
 
 

 






 
# 3724 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 3733 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"















  
 
# 3756 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"


















 








































 


















































 


 


 


 


 


 


 
# 3891 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 3898 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 3905 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 3912 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"







 
# 3926 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 3933 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 3940 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 3947 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 3954 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 3961 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 3969 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 3976 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 3983 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 3990 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 3997 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 4004 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 4012 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 4019 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 4026 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 4033 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"





 


 


 


 


 



 
 
 
 
 

 














































 



 


 


 


 


 


 


 



 



 



 


 


 



 
 
 
 
 
 





 






 


 
# 4180 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 4190 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 


 


 
 
 
 
 

 
















 









# 4238 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 

























 
# 4281 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 4295 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 4305 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 




























 





















 




























 





















 
# 4424 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
# 4459 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"





# 4470 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 4478 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 4485 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 


 
 
 
 
 

 




 
# 4507 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
 
 
 
 

 


 





 


 



 
 
 
 
 

 
# 4569 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 
# 4581 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"







 


 
 
 
 
 

 











# 4619 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 











# 4642 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 











# 4665 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 











# 4688 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 












# 4711 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"























 












# 4756 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"























 












# 4801 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"























 












# 4846 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"























 












# 4891 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

















 












# 4930 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

















 












# 4969 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

















 












# 5008 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

















 



























 



























 



























 
# 5117 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 5126 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 5135 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 5146 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5156 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5166 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5176 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 5187 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5197 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5207 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5217 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 5228 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5238 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5248 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5258 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 5269 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5279 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5289 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5299 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 5310 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5320 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5330 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5340 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 5351 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5361 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5371 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5381 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 5392 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5402 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5412 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

# 5422 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 


 


 
 
 
 
 

 




 












 


 






# 5470 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
















 


 
# 5540 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 5555 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 5581 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 


 


 
 
 
 
 

 
 























 























 























 























 























 























 























 























 
 
# 5802 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 5814 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 






 
# 5831 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



     


 
 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


# 5975 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 5987 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 5999 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6011 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6023 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6035 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6047 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6059 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 

 


# 6073 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6085 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6097 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6109 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6121 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6133 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6145 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6157 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6169 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6181 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6193 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6205 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6217 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6229 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6241 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 


# 6253 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 
 
 
 
 

 
 
# 6273 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 6284 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 6302 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"











 





 





 
# 6340 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 












 
# 6361 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
 






 




 





 





 






 




 





 





 






   




 





 





 





 




 





 





 





 




 





 





 
 


 
# 6501 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 6518 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 6535 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 6552 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 6586 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 6620 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 6654 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 6688 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 6722 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 6756 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 6790 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 6824 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 6858 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 6892 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 6926 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 6960 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 6994 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7028 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7062 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7096 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7130 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7164 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7198 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7232 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7266 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7300 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7334 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7368 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7402 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7436 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7470 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7504 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
 
 
 
 

 









# 7531 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7539 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7549 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 


 


 


 


 





















 




 
 
 
 
 

 
# 7610 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7619 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"







 



# 7640 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 



 


 
# 7665 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7675 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 




 


 
 
 
 
 

 
# 7701 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 


 



 
# 7725 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7734 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"







 
# 7754 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
# 7765 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 
 
 
 
 

 


# 7794 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 









# 7828 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 
 
 
 
 

 









 


 




 


 





 
# 7873 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"

 


 









 


 

 



 



 



 



 



 



 



 



# 8337 "C:\\Arm\\Packs\\Keil\\STM32F1xx_DFP\\2.4.0\\Device\\Include\\stm32f10x.h"



 

 

  







 

















 









 

  

 

# 66 "RTE\\Device\\STM32F103C8\\system_stm32f10x.c"



 



 



 



 






















 
    




 
  
 
 
 
 




  





  
 






 



 



 



 



 
# 164 "RTE\\Device\\STM32F103C8\\system_stm32f10x.c"
  uint32_t SystemCoreClock         = ((uint32_t)8000000);         


volatile const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};


 



 

static void SetSysClock(void);

# 191 "RTE\\Device\\STM32F103C8\\system_stm32f10x.c"







 



 








 
void SystemInit (void)
{
   
   
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CR |= (uint32_t)0x00000001;

   

  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CFGR &= (uint32_t)0xF8FF0000;



  
   
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CR &= (uint32_t)0xFEF6FFFF;

   
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CR &= (uint32_t)0xFFFBFFFF;

   
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CFGR &= (uint32_t)0xFF80FFFF;

# 250 "RTE\\Device\\STM32F103C8\\system_stm32f10x.c"
   
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CIR = 0x009F0000;

    






   
   
  SetSysClock();




  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR = ((uint32_t)0x08000000) | 0x0;  

}



































 
void SystemCoreClockUpdate (void)
{
  uint32_t tmp = 0, pllmull = 0, pllsource = 0;








    
   
  tmp = ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CFGR & ((uint32_t)0x0000000C);
  
  switch (tmp)
  {
    case 0x00:   
      SystemCoreClock = ((uint32_t)8000000);
      break;
    case 0x04:   
      SystemCoreClock = ((uint32_t)8000000);
      break;
    case 0x08:   

       
      pllmull = ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CFGR & ((uint32_t)0x003C0000);
      pllsource = ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CFGR & ((uint32_t)0x00010000);
      

      pllmull = ( pllmull >> 18) + 2;
      
      if (pllsource == 0x00)
      {
         
        SystemCoreClock = (((uint32_t)8000000) >> 1) * pllmull;
      }
      else
      {





         
        if ((((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CFGR & ((uint32_t)0x00020000)) != (uint32_t)RESET)
        { 
          SystemCoreClock = (((uint32_t)8000000) >> 1) * pllmull;
        }
        else
        {
          SystemCoreClock = ((uint32_t)8000000) * pllmull;
        }

      }
# 400 "RTE\\Device\\STM32F103C8\\system_stm32f10x.c"
      break;

    default:
      SystemCoreClock = ((uint32_t)8000000);
      break;
  }
  
   
   
  tmp = AHBPrescTable[((((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CFGR & ((uint32_t)0x000000F0)) >> 4)];
   
  SystemCoreClock >>= tmp;  
}





 
static void SetSysClock(void)
{
# 434 "RTE\\Device\\STM32F103C8\\system_stm32f10x.c"
 
 
  
}






  
# 491 "RTE\\Device\\STM32F103C8\\system_stm32f10x.c"

# 1082 "RTE\\Device\\STM32F103C8\\system_stm32f10x.c"



 



 
  


     
 
