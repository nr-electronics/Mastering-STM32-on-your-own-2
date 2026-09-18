/**
  ******************************************************************************
  * File Name          : readme.txt
  * Description        : Readme file
  * Date               : Mar-22-2017
  ******************************************************************************
  *
  * Copyright (c) 2017 Waveshare
  * All rights reserved.
  *
  * THIS SOFTWARE IS PROVIDED BY COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  
  == Development Environment ==
  * KEILMDK Version 5.12 or above.
  * Programmer/Debugger: ST-LINK V2
  * Programming/Debugginginterface: JTAG/SWD

  == Example Description ==
  This example provides a description of how to set a communication with the CAN
  in loopback mode.

  The CAN cell first performs a transmission and a reception of a standard data
  frame by polling. The received frame is checked and the debug information indicates
  whether the communication was successful.

  == How to use ==
  Connect the board to your PC using a serial module via the USART1 port for debug.
     * Baud rate: 115200
     * Data bits: 8
     * Stop bits: 1
     * Parity: None
     * Flow control: None
  */