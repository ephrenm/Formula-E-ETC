//============================================================================
#include <Arduino.h>
#include <SPI.h>
#include <stdarg.h>
#include <string.h>
//Definitions for our circuit board and display.
#include "CFA10100_defines.h"

#include <SD.h>

// The very simple EVE library files
#include "EVE_base.h"
#include "EVE_draw.h"

// Our demonstrations of various EVE functions
#include "demos.h"

//Vars need to be global to be accessible to EVE functions
String currentDisplay = "Menu";
uint16_t A = 150, B = 150, counter=0;;
char MainScreen[] = "Main";
//char* MainScreen = MainScreen_string;
char ReturnMenuScreen[] = "Back";
char TestingScreen[] = "Testing";
char Switch1Screen[] = "Switch1";
char Switch2Screen[] = "Switch2";
char AccelerationPD[] = "AP: ";
char BrakePD[] = "BP: ";


//===========================================================================
void setup() {
#if (DEBUG_LEVEL != DEBUG_NONE)
  // Initialize UART for debugging messages
  Serial.begin(115200);
#endif // (DEBUG_LEVEL != DEBUG_NONE)
  DBG_STAT("Begin\n");

  //Initialize GPIO port states
  // Set CS# high to start - SPI inactive
  SET_EVE_CS_NOT;
  // Set PD# high to start
  SET_EVE_PD_NOT;

  SET_SD_CS_NOT;

  //Initialize port directions
  // EVE interrupt output (not used in this example)
  pinMode(EVE_INT, INPUT_PULLUP);
  // EVE Power Down (reset) input
  pinMode(EVE_PD_NOT, OUTPUT);
  // EVE SPI bus CS# input
  pinMode(EVE_CS_NOT, OUTPUT);
  // USD card CS
  pinMode(SD_CS, OUTPUT);
  // Optional pin used for LED or oscilloscope debugging.
  pinMode(DEBUG_LED, OUTPUT);

  // Initialize SPI
  SPI.begin();
  SPI.setMOSI(SPI_MOSI);
  SPI.setMISO(SPI_MISO);
  SPI.setSCK(SPI_SCK);

  //Bump the clock to 8MHz. Appears to be the maximum.
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  DBG_GEEK("SPI initialzed to: 8MHz\n");

  
#if BUILD_SD
  // The prototype hardware appears to functon fine at 8MHz which
  // also appears to be the max that the ATmega328P can do.
  if (!SD.begin(8000000,SD_CS)) {
    DBG_STAT("uSD card failed to initialize, or not present\n");
    //Reset the SPI clock to fast. SD card library does not clean up well.
    //Bump the clock to 8MHz. Appears to be the maximum.
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    }
  else {
    DBG_STAT("uSD card initialized.\n");
    }
#endif

  //See if we can find the FTDI/BridgeTek EVE processor
  if(0 != EVE_Initialize()) {
    DBG_STAT("Failed to initialize %s8%02X. Stopping.\n",EVE_DEVICE<0x14?"FT":"BT",EVE_DEVICE);
    while(1);
  }
  else {
    DBG_STAT("%s8%02X initialized.\n",EVE_DEVICE<0x14?"FT":"BT",EVE_DEVICE);
  }
  }

//===========================================================================

void loop() {
  
  DBG_GEEK("Loop initialization.\n");
  //Get the current write pointer from the EVE
  uint16_t FWo = EVE_REG_Read_16(EVE_REG_CMD_WRITE);
  DBG_GEEK("Initial Offset Read: 0x%04X = %u\n",FWo ,FWo);
  //Keep track of the RAM_G memory allocation
  uint32_t RAM_G_Unused_Start=0;
  DBG_GEEK("Initial RAM_G: 0x%08lX = %lu\n",RAM_G_Unused_Start,RAM_G_Unused_Start);
  // We need to keep track of the bitmap handles and where they are used.
  //
  // By default, bitmap handles 16 to 31 are used for built-in font and 15
  // is used as scratch bitmap handle by co-processor engine commands
  // CMD_GRADIENT, CMD_BUTTON and CMD_KEYS.
  // For whatever reason, I am going to allocate handles from 14 to 0.
  uint8_t next_bitmap_handle_available=14;

  DBG_GEEK("EVE_Initialize_Flash() . . . ");
  FWo=EVE_Initialize_Flash(FWo);
  DBG_GEEK("done.\n");
  DBG_GEEK("Not programming flash.\n");
  

  uint8_t flash_status = EVE_REG_Read_8(EVE_REG_FLASH_STATUS);
  DBG_GEEK_Decode_Flash_Status(flash_status);

  uint8_t points_touched_mask;
  int16_t x_points[1], y_points[1];
  //int16_t x_points, y_points;

  DBG_STAT("Initialization complete, entering main loop.\n");

  //FWo = Calibrate_Touch(FWo); //#1 test for callibration to work(try multiple posititons)

  //Initialize bounce circle
  DBG_STAT("Initialize_Bounce_Demo() . . .");
  Initialize_Bounce_Demo();
  DBG_STAT(" done.\n");

  //Initialize logo 
  DBG_STAT("Initialize_Logo_Demo() . . .");
  FWo=Initialize_Logo_Demo(FWo,&RAM_G_Unused_Start,next_bitmap_handle_available);
  //Keep track that we used a bitmap handle
  next_bitmap_handle_available--;
  DBG_STAT("  done.\n");
  DBG_GEEK("RAM_G after logo: 0x%08lX = %lu\n",RAM_G_Unused_Start,RAM_G_Unused_Start);
  
  
  while(1) {
    FWo=Wait_for_EVE_Execution_Complete(FWo);
    
    points_touched_mask=Read_Touch(x_points,y_points);

    //Testing number of points recognized
    Serial.print("Number of points recognized: ");
    Serial.println(points_touched_mask); //Only one bc resistive touch not capacitive
    Serial.print(counter);
    counter++;
    if (counter < 500) {
      currentDisplay = "Menu";
    }
    if (counter == 500) {
      currentDisplay = "Main";
    }
    else if (counter == 1000) {
      currentDisplay = "TESTING";
    }
    else if (counter == 1500) {
      currentDisplay = "Switch1";
    }
    else if (counter == 2000) {
      currentDisplay = "Switch1";
    }
    else if (counter > 2000) {
      counter = 0;
    }
    

    
    //========== START THE DISPLAY LIST ==========
    // Start the display list
    FWo=EVE_Cmd_Dat_0(FWo, (EVE_ENC_CMD_DLSTART));
  
    // Set the default clear color to black
    FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_CLEAR_COLOR_RGB(0,0,0));
    // Clear the screen - this and the previous prevent artifacts between lists
    FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_CLEAR(1 /*CLR_COL*/,1 /*CLR_STN*/,1 /*CLR_TAG*/));
    //========== ADD GRAPHIC ITEMS TO THE DISPLAY LIST ==========
    //Fill background with white
    FWo=EVE_Filled_Rectangle(FWo,0,0,LCD_WIDTH-1,LCD_HEIGHT-1);
    

    //End of display list setup
    if (currentDisplay == "Menu") {
      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(255,0,0)); //Change color for rect
      FWo=EVE_Filled_Rectangle(FWo,0,0,LCD_WIDTH-400,LCD_HEIGHT-240);
      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(0,0,0)); //Change color for text
      FWo=EVE_Text(FWo,200,120,25,EVE_OPT_CENTER,MainScreen); //Plain text

      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(255,125,0)); //Change color for rect
      FWo=EVE_Filled_Rectangle(FWo,400,0,LCD_WIDTH,LCD_HEIGHT-240);
      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(0,0,0)); //Change color for text
      FWo=EVE_Text(FWo,600,120,25,EVE_OPT_CENTER,TestingScreen); //Plain text

      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(50,50,50)); //Change color for rect
      FWo=EVE_Filled_Rectangle(FWo,0,240,LCD_WIDTH-400,LCD_HEIGHT);
      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(0,0,0)); //Change color for text
      FWo=EVE_Text(FWo,200,360,25,EVE_OPT_CENTER,Switch1Screen); //Plain text

      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(0,255,0)); //Change color for rect
      FWo=EVE_Filled_Rectangle(FWo,400,240,LCD_WIDTH,LCD_HEIGHT);
      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(0,0,0)); //Change color for text
      FWo=EVE_Text(FWo,600,360,25,EVE_OPT_CENTER,Switch2Screen); //Plain text

      if(0 != points_touched_mask) {
        //FWo=EVE_Point(FWo, x_points[0]*16, y_points[0]*16, 30*16);
        if (x_points[0] >= 0 && x_points[0] <= LCD_WIDTH-400 && y_points[0] >= 0 && y_points[0] <= LCD_HEIGHT-240) {
          currentDisplay = "Main";
        }
        else if (x_points[0] >= 400 && x_points[0] <= LCD_WIDTH && y_points[0] >= 0 && y_points[0] <= LCD_HEIGHT-240) {
          currentDisplay = "TESTING";
        }
        else if (x_points[0] >= 0 && x_points[0] <= LCD_WIDTH-400 && y_points[0] >= 240 && y_points[0] <= LCD_HEIGHT) {
          currentDisplay = "Switch1";
        }
        else if (x_points[0] >= 400 && x_points[0] <= LCD_WIDTH && y_points[0] >= 240 && y_points[0] <= LCD_HEIGHT) {
          currentDisplay = "Switch2";
        }
        while(0 != points_touched_mask) {
          points_touched_mask=Read_Touch(x_points,y_points);
        }
      }
    }
    else if (currentDisplay == "Main") {
      //Drawing back button
      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(255,0,0)); //Change color for rect
      FWo=EVE_Filled_Rectangle(FWo,0,0,100,100);
      // FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(0,0,0)); //Change color for text
      // FWo=EVE_Text(FWo,20,10,15,EVE_OPT_CENTER,"ReturnMenuScreen"); //Plain text

      //Drawing pedal rect
      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(50,0,0)); //Change color for rect
      FWo=EVE_Filled_Rectangle(FWo,250,380,350,480);
      FWo=EVE_Filled_Rectangle(FWo,450,380,550,480);

      //Pedal text
      FWo=EVE_Text(FWo,65,125,25,EVE_OPT_CENTER,AccelerationPD); //Plain text
      FWo=EVE_Text(FWo,65,200,25, EVE_OPT_CENTER,BrakePD); //Plain text

      //Percentage rectangles
      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(0,100,0)); //Change color for rect
      FWo=EVE_Open_Rectangle(FWo,150,80,750,160,3); //Accelerator rect
      FWo=EVE_Open_Rectangle(FWo,150,180,750,260,3); //Brake rect

      FWo=EVE_Filled_Rectangle(FWo,150,80,A,160);
      FWo=EVE_Filled_Rectangle(FWo,150,180,B,260);

      if(0 != points_touched_mask) {
        if (x_points[0] >= 0 && x_points[0] <= 100 && y_points[0] >= 0 && y_points[0] <= 100) {
          currentDisplay = "Menu";
          while(0 != points_touched_mask) {
            points_touched_mask=Read_Touch(x_points,y_points);
          }
        }
        else {
          if (x_points[0] >= 250 && x_points[0] <= 350 && y_points[0] >= 380 && y_points[0] <= 480) { //Brake
            if (B < 750) {
              B+=2;
            }
            if(A>150) {
              A-=2;
            }
          }
          else if (x_points[0] >= 450 && x_points[0] <= 550 && y_points[0] >= 380 && y_points[0] <= 480) {//Accelerator 
            if (A < 750) {
              A+=2;
            }
            if(B>150) {
              B-=2;
            }
          }
          else {
            if(B>150) {
              B-=2;
            }
            if(A>150) {
              A-=2;
            }
          }
        }
      }
      else {
        if(B>150) {
          B-=2;
        }
        if(A>150) {
          A-=2;
        }
      }
    }
    else if (currentDisplay == "Switch1") {
      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(255,0,0)); //Change color for rect
      FWo=EVE_Filled_Rectangle(FWo,0,0,100,100);
      // FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(0,0,0)); //Change color for text
      // FWo=EVE_Text(FWo,400,240,25,EVE_OPT_CENTER,"ReturnMenuScreen"); //Plain text

      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(90,90,0)); //Change color for rect
      FWo=EVE_Filled_Rectangle(FWo,100,100,LCD_WIDTH-100,LCD_HEIGHT-100);
      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(255,0,0)); //Change color for text
      FWo=EVE_Text(FWo,400,240,25,EVE_OPT_CENTER,Switch2Screen); //Plain text
      if(0 != points_touched_mask) {
        //FWo=EVE_Point(FWo, x_points[0]*16, y_points[0]*16, 30*16);
        if (x_points[0] >= 100 && x_points[0] <= LCD_WIDTH-100 && y_points[0] >= 100 && y_points[0] <= LCD_HEIGHT-100) {
          currentDisplay = "Switch2";
        }
        else if (x_points[0] >= 0 && x_points[0] <= 100 && y_points[0] >= 0 && y_points[0] <= 100) {
          currentDisplay = "Menu";
        }
        while(0 != points_touched_mask) {
          points_touched_mask=Read_Touch(x_points,y_points);
        }
      }
    }
    else if (currentDisplay == "Switch2") {
      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(255,0,0)); //Change color for rect
      FWo=EVE_Filled_Rectangle(FWo,0,0,100,100);
      // FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(0,0,0)); //Change color for text
      // FWo=EVE_Text(FWo,400,240,15,EVE_OPT_CENTER,"ReturnMenuScreen"); //Plain text

      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(40,40,40)); //Change color for rect
      FWo=EVE_Filled_Rectangle(FWo,100,100,LCD_WIDTH-100,LCD_HEIGHT-100);
      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(255,0,0)); //Change color for text
      FWo=EVE_Text(FWo,400,240,25,EVE_OPT_CENTER, Switch1Screen); //Plain text
      if(0 != points_touched_mask) {
        //FWo=EVE_Point(FWo, x_points[0]*16, y_points[0]*16, 30*16);
        if (x_points[0] >= 100 && x_points[0] <= LCD_WIDTH-100 && y_points[0] >= 100 && y_points[0] <= LCD_HEIGHT-100) {
          currentDisplay = "Switch1";
        }
        else if (x_points[0] >= 0 && x_points[0] <= 100 && y_points[0] >= 0 && y_points[0] <= 100) {
          currentDisplay = "Menu";
        }
        while(0 != points_touched_mask) {
          points_touched_mask=Read_Touch(x_points,y_points);
        }
      }
    }
    else if (currentDisplay == "TESTING"){
      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(255,0,0)); //Change color for rect
      FWo=EVE_Filled_Rectangle(FWo,0,0,100,100);
      // FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(0,0,0)); //Change color for text
      // FWo=EVE_Text(FWo,400,240,15,EVE_OPT_CENTER,"ReturnMenuScreen"); //Plain text
      
      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(50,50,0)); //Change color for rect
      //Makes circle at finger
      if(0 != points_touched_mask) {
        if (x_points[0] >= 0 && x_points[0] <= 100 && y_points[0] >= 0 && y_points[0] <= 100) {
          currentDisplay = "Menu";
          counter=0;
          while(0 != points_touched_mask) {
            points_touched_mask=Read_Touch(x_points,y_points);
          }
        }
        else {
          static uint32_t color = EVE_ENC_COLOR_RGB(0x00,0xFF,0xFF);
          FWo=EVE_Cmd_Dat_0(FWo, color); //Set color

          FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_A(0xFF)); //Make color solid

          FWo=EVE_Point(FWo, x_points[0]*16, y_points[0]*16, 60*16);// Draw the touch dot -- a 60px point (filled circle)

          //Drawing text under dot
          FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(0xFF,0x00,0xFF));//Tag the touch point with magenta text to show off EVE_PrintF.
          
          int16_t xoffset, yoffset;//Used to move the the text out from under the user's finger
          if(x_points[0] < (LCD_WIDTH/2)) {
              xoffset=160; 
          }
          else {
              xoffset=-160;
          }
          if(y_points[0] < (LCD_HEIGHT/2)) {
              yoffset=80; 
          }
          else {
              yoffset=-80;
          }
          //Put the text into the display list
          FWo=EVE_PrintF(FWo,x_points[0]+xoffset,y_points[0]+yoffset,25,EVE_OPT_CENTER, //Options
                        "T[%d]@(%d,%d)",1,x_points[0],y_points[0]);
        } //if(0 != points_touched_mask)
      }
      FWo=EVE_Text(FWo,130,200,25,EVE_OPT_CENTER,MainScreen); //Plain text

      FWo=EVE_PrintF(FWo,400,40,25,EVE_OPT_CENTER,"CAN BUS: [%s]@(%d)","This is a message",counter); //Dynamic text
      counter+=1;
      //Serial.println(counter);

      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(0,255,0)); //Change color for rect
      FWo=EVE_Filled_Rectangle(FWo,500,400,600,480);

      FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(70,30,120)); //Change color for open rect
      FWo = EVE_Open_Rectangle(FWo,50,150,200,250,2);

      FWo=Add_Bounce_To_Display_List(FWo); //Add spinning circle

      FWo=Add_Logo_To_Display_List(FWo);//Add logo
      
      }
    

    //========== FINSH AND SHOW THE DISPLAY LIST ==========
    // Instruct the graphics processor to show the list
    FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_DISPLAY());
    // Make this list active
    FWo=EVE_Cmd_Dat_0(FWo, EVE_ENC_CMD_SWAP);
    // Update the ring buffer pointer so the graphics processor starts executing
    EVE_REG_Write_16(EVE_REG_CMD_WRITE, (FWo));
    if (currentDisplay == "TESTING"){
      Bounce_Ball(); //Bounce circle
    }
  
  }  // while(1)
  } // loop()
//===========================================================================
