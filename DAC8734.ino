
#include <SPI.h>

// comment out the next line if your using a launch pad
#define Arduino // for selecting the right code parts

// define the buffer size...
#define serialbufferSize 50
#define commandDelimeters "|,.- "

// Now the real varibles
char inputBuffer[serialbufferSize]   ;
int serialIndex = 0; // keep track of where we are in the buffer
// End of real variables

//********************************************************
// DAC8734 code starts here
//********************************************************

// PICK THE RIGHT chip select for the right board

#define DAC_8734_CS_PIN  SS  // All the boards tested so far has this pin defined and it is in a consistent place on the expansion headers

#define VREF 5.000
// Default values below assume 0-10V Unipolar or -5 to +5 bipolar output
#define DAC_VFSR VREF*2 // may be 0 - 10 or -5 to +5
#define DAC_8734_Command 0 // command register
#define DAC_8734_DataBase 0x04 // register offset to zero cal base register
#define DAC_8734_CalZeroBase 0x08 // register offset to zero cal base register
#define DAC_8734_CalGainBase 0x0C // register offset to gain cal base register
#define DACMAX 0xFFFF
#define DACMIN 0x0000

// default setup of DAC8734, these can be adjusted if needed, some functions also change them to change mode etc.
int DAC_Gain0 = 0; // 0 = *2, 1 = *4
int DAC_Gain1 = 0; // 0 = *2, 1 = *4
int DAC_Gain2 = 0; // 0 = *2, 1 = *4
int DAC_Gain3 = 0; // 0 = *2, 1 = *4
int DAC_GPIO0 = 1; // 1 = Group A in Unipolar 0=Bipolar (External connection to control pin)
int DAC_GPIO1 = 1; // 1 = Group B in Unipolar 0=Bipolar (External connection to control pin)
int DAC_PD_A = 0;  // 1 = group A power down
int DAC_PD_B = 0;  // 1 = group B power down
int DAC_DSDO = 0;  // 1 = Disable SDO bit.

// set up the speed, data order and data mode
// See https://www.arduino.cc/en/Reference/SPI for more details

#ifdef Arduino
// not available on energia yet
SPISettings settingsDAC8734(5000000, MSBFIRST, SPI_MODE0);
#endif

// Sine table, used to create a reasonable sinewave on a dac output, 256 16bit values, one complete cycle
byte val = 0;
unsigned int Sin_tab[256] = {  32768, 33572, 34376, 35178, 35980, 36779, 37576, 38370, 39161, 39947, 40730, 41507, 42280,
                               43046, 43807, 44561, 45307, 46047, 46778, 47500, 48214, 48919, 49614, 50298, 50972, 51636,
                               52287, 52927, 53555, 54171, 54773, 55362, 55938, 56499, 57047, 57579, 58097, 58600, 59087,
                               59558, 60013, 60451, 60873, 61278, 61666, 62036, 62389, 62724, 63041, 63339, 63620, 63881,
                               64124, 64348, 64553, 64739, 64905, 65053, 65180, 65289, 65377, 65446, 65496, 65525, 65535,
                               65525, 65496, 65446, 65377, 65289, 65180, 65053, 64905, 64739, 64553, 64348, 64124, 63881,
                               63620, 63339, 63041, 62724, 62389, 62036, 61666, 61278, 60873, 60451, 60013, 59558, 59087,
                               58600, 58097, 57579, 57047, 56499, 55938, 55362, 54773, 54171, 53555, 52927, 52287, 51636,
                               50972, 50298, 49614, 48919, 48214, 47500, 46778, 46047, 45307, 44561, 43807, 43046, 42280,
                               41507, 40730, 39947, 39161, 38370, 37576, 36779, 35980, 35178, 34376, 33572, 32768, 31964,
                               31160, 30358, 29556, 28757, 27960, 27166, 26375, 25589, 24806, 24029, 23256, 22490, 21729,
                               20975, 20229, 19489, 18758, 18036, 17322, 16617, 15922, 15238, 14564, 13900, 13249, 12609,
                               11981, 11365, 10763, 10174, 9598, 9037, 8489, 7957, 7439, 6936, 6449, 5978, 5523, 5085, 4663,
                               4258, 3870, 3500, 3147, 2812, 2495, 2197, 1916, 1655, 1412, 1188, 983, 797, 631, 483, 356, 247,
                               159, 90, 40, 11, 1, 11, 40, 90, 159, 247, 356, 483, 631, 797, 983, 1188, 1412, 1655, 1916, 2197,
                               2495, 2812, 3147, 3500, 3870, 4258, 4663, 5085, 5523, 5978, 6449, 6936, 7439, 7957, 8489, 9037,
                               9598, 10174, 10763, 11365, 11981, 12609, 13249, 13900, 14564, 15238, 15922, 16617, 17322,
                               18036, 18758, 19489, 20229, 20975, 21729, 22490, 23256, 24029, 24806, 25589, 26375, 27166,
                               27960, 28757, 29556, 30358, 31160, 31964
                            };

//Calibration table, you will need to adjust these values to suit your build
int DAC_CAL_tab[8] = { 0x08, 0xfF, 0x0F, 0x4F,   0x3f, 0x80, 0x80, 0x80}; // zero's then gain's

// The DAC8734 uses Mode 1 SPI, has an 8 bit address byte followed by 16bit data
// high byte first, high bit first
// This is the register descriptions
// Reg   description
// 0    Control Register
// 1    Monitor
// 2    Not Used
// 3    Not Used
// 4    DAC 0 Data Register
// 5    DAC 1 Data Register
// 6    DAC 2 Data Register
// 7    DAC 3 Data Register
// 8    DAC 0 Zero Cal Register
// 9    DAC 1 Zero Cal Register
// a    DAC 2 Zero Cal Register
// b    DAC 3 Zero Cal Register
// c    DAC 0 Zero Gain Register
// d    DAC 1 Zero Gain Register
// e    DAC 2 Zero Gain Register
// f    DAC 3 Zero Gain Register


// the most basic function, write to register "reg", with a value "val"
void WriteDACRegister(byte reg, unsigned int val)
{ 
#ifdef Arduino 
  SPI.beginTransaction(settingsDAC8734); // ensures the SPI bus is in the right mode for this device irrespective of other devices used in the code
#endif
  digitalWrite(DAC_8734_CS_PIN, LOW); // Select the Chip
  SPI.transfer(reg); // Select the target register
  SPI.transfer(val >> 8); // Send the High Data Byte
  SPI.transfer(val & 0xFF); // Send the Low Data Byte
  digitalWrite(DAC_8734_CS_PIN, HIGH); // Release the Chip
#ifdef Arduino
  SPI.endTransaction();
#endif
}

// Setup a default DAC using the variables set above
// note the variables can be chaned then recall InitDac to change mode
// of operation of the DAC
// Currently set to Unipolar, 2* Gain and Powered up
void InitDAC()
{
  // for now, gain of 2, gpio hiZ,
  int DAC_INIT = 0x0000 | DAC_PD_A << 12 | DAC_PD_B << 11 | DAC_GPIO1 << 9 | DAC_GPIO0 << 8 | DAC_DSDO << 7 | DAC_Gain3 << 5 | DAC_Gain2 << 4 | DAC_Gain1 << 3 | DAC_Gain0 << 2 ;
  Serial.print("Command Reg = "); Serial.print(DAC_INIT, HEX); Serial.print(" "); Serial.println(DAC_INIT, BIN);
  WriteDACRegister(DAC_8734_Command, DAC_INIT);
}

// Helper Function to output to a specified DAC, a Desired value between 0 and 65535
// this could represent 0V - 10V or -5 to +5V depending on mode
void SetDAC( byte channel, unsigned int value)
{
  if (channel > 3)
  {
    Serial.println("DAC  must be 0 - 3");
  }
  else
  {
    //Serial.print("DAC "); Serial.print(channel); Serial.print(" Value "); Serial.println(value, HEX);
    WriteDACRegister(DAC_8734_DataBase + channel, value);
  }
}

//Output the Calibration table to the DAC for all channels
void CalibrateDAC()
{
  for (int x = 0 ; x < 4 ; x++)
  {
    WriteDACRegister(DAC_8734_CalZeroBase + x, DAC_CAL_tab[x]) ;// Zero cal
    WriteDACRegister(DAC_8734_CalGainBase + x, DAC_CAL_tab[x + 4]); // Gain cal
  }
}
// Simple itteration through the sine table above to simulate a sinewave
// you can change the table contents above to send whatever wave shape you want
void sine( byte channel, unsigned int loopcount)
{
  if (loopcount > 65535) loopcount = 65535;
  if (channel > 3) channel = 0;
  Serial.print("Sine on DAC ");Serial.print(channel ); Serial.print(" Cycles= ");Serial.println(loopcount);
  for (int y = 0 ; y < loopcount ; y++)
  {
    for (int x = 0 ; x < 256 ; x++)
    {
      SetDAC(channel, Sin_tab[x]);
    }
  }
  Serial.println("Complete");
}

// Send to Serial port the Help Menu
void help()
{
  // Print some pretty instructions
  Serial.println("DAC8734 Test Program:");
  Serial.println("Peter Oakes, July 2015\n");
  Serial.println("Commands:");
  Serial.println("help\" for this menu" );
  Serial.println("dacx  x is from 0 to 3, value 0 to 65535" );
  Serial.println("bpA or bpB mode  Bipolar");
  Serial.println("upA or upB mode Unipolar" );
  Serial.println("offA or offB Power Off group A or B" );
  Serial.println("onA or onB   Power On for Group A or B" );
  Serial.println("sinex cycles, x is DAC from 0 to 3, cycles 0 to 65535" );
  Serial.println();
  Serial.println("make sure \"NewLine\" is on in the console setup" );

}
//********************************************************
// Standard Arduino / Energia setup() and loop() functions
// do not delete
//********************************************************
void setup()
{
  Serial.begin(115200);
  // print the help menu for the DAC8734 tester
  help();
  // setup the chip select pin for the DAC
  pinMode(DAC_8734_CS_PIN, OUTPUT);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  // Initialisze the DAC8734
  InitDAC();
  // output the DAC_CAL_tab to the DAC
  CalibrateDAC();
  // set the DAC outputs to Zero
  SetDAC(0, 0x0000);
  SetDAC(1, 0x0000);
  SetDAC(2, 0x0000);
  SetDAC(3, 0x0000);
}

void loop()
{
  // see if there is a command come in on the serial port and if so call the command processor
    if (CheckSerial()) DoCommand(inputBuffer);
}

//********************************************************
// UTILITY FUNCTIONS TO MAKE LIFE EASY
//********************************************************
/*
Checks the serial input for a string, returns true once a '\n' is seen
users can always look at the global variable "serialIndex" to see if characters have been received already
*/
boolean CheckSerial()
{
  boolean lineFound = false;
  // if there's any serial available, read it:
  while (Serial.available() > 0) {
    //Read a character as it comes in:
    //currently this will throw away anything after the buffer is full or the \n is detected
    char charBuffer = Serial.read();
    if (charBuffer == '\n') {
      inputBuffer[serialIndex] = 0; // terminate the string
      lineFound = (serialIndex > 0); // only good if we sent more than an empty line
      serialIndex = 0; // reset for next line of data
    }
    else if (charBuffer == '\r') {
      // Just ignore the Carrage return, were only interested in new line
    }
    else if (serialIndex < serialbufferSize && lineFound == false) {
      /*Place the character in the string buffer:*/
      inputBuffer[serialIndex++] = charBuffer; // auto increment index
    }
  }// End of While
  return lineFound;
}// End of CheckSerial()


// Enhanced Command Processor using strtok to strip out command from multi parameter string
boolean DoCommand(char * commandBuffer)
{
  Serial.println("Command");
  char* Command; // Command Parameter
  char* Parameter; // Additional Parameter
  unsigned int analogVal = 0; // additional parameter converted to analog if possible

  // Get the command from the input string
  Command = strtok(commandBuffer, commandDelimeters); // get the command
  Parameter = strtok(NULL, commandDelimeters); // get the parameter if any
  //if there are more than one parameter they will be ignored for now

  // Make sure we have an analog value if we are to allow PWM output
  unsigned int outparameter = isNumeric (Parameter);

  //if it is a number then convert it
  if (outparameter)
  {
    analogVal = atoi(Parameter);
    // check the analog value is in the correct range
    if (analogVal < DACMIN || analogVal > DACMAX) outparameter = false;
  }
  // Standard way to handle commands
  if (strcmp(Command, "help") == 0) { help(); }
  //DAC Outputs if we have a valid analog parameter
  else if (strcmp(Command, "dac0") == 0  && outparameter ) { SetDAC(0, analogVal);}   // Set the DAC 0 output 
  else if (strcmp(Command, "dac1") == 0  && outparameter ) { SetDAC(1, analogVal);}  // Set the DAC 1 output 
  else if (strcmp(Command, "dac2") == 0  && outparameter ) { SetDAC(2, analogVal);}   // Set the DAC 2 output 
  else if (strcmp(Command, "dac3") == 0  && outparameter ) { SetDAC(3, analogVal);}   // Set the DAC 3 output 
  //Sine Wave Outputs
  else if (strcmp(Command, "sine0") == 0  && outparameter ) { sine(0,analogVal);}    // Set the DAC 3 output 
  else if (strcmp(Command, "sine1") == 0  && outparameter ) { sine(1,analogVal);}   // Set the DAC 3 output 
  else if (strcmp(Command, "sine2") == 0  && outparameter ) { sine(2,analogVal);}   // Set the DAC 3 output 
  else if (strcmp(Command, "sine3") == 0  && outparameter ) { sine(3,analogVal);}   // Set the DAC 3 output 
  // UNIPOLAR
  else if (strcmp(Command, "upA") == 0 ) { Serial.println("Setting UniPolar  Group A");  DAC_GPIO0 = 1; InitDAC(); }
  else if (strcmp(Command, "upB") == 0 ) { Serial.println("Setting UniPolar  Group B");  DAC_GPIO1 = 1; InitDAC(); }
  // BIPOLAR
  else if (strcmp(Command, "bpA") == 0 ) { Serial.println("Setting BiPolar  Group A");   DAC_GPIO0 = 0; InitDAC();  }
  else if (strcmp(Command, "bpB") == 0 ) { Serial.println("Setting BiPolar  Group B");   DAC_GPIO1 = 0; InitDAC();  }
  // Power Down
  else if (strcmp(Command, "offA") == 0 ) {Serial.println("GrpA Off "); DAC_PD_A = 1; InitDAC(); }
  else if (strcmp(Command, "offB") == 0 ) {Serial.println("GrpB Off "); DAC_PD_B = 1; InitDAC(); }
  // Power Up
  else if (strcmp(Command, "onA") == 0 ) { Serial.println("GrpA On"); DAC_PD_A = 0; InitDAC(); }
  else if (strcmp(Command, "onB") == 0 ) { Serial.println("GrpB On"); DAC_PD_B = 0; InitDAC(); }
  // Catch All
  else {  Serial.print("Error "); Serial.println(commandBuffer); }
  return true;
}

// Utility function to make sure the string is a numneric one
int isNumeric (const char * s)
{
  while (*s)
  {
    if (!isdigit(*s)) return 0;
    s++;
  }
  return 1;
}
