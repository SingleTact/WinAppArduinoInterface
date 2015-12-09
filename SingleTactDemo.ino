//-----------------------------------------------------------------------------
//  Copyright (c) 2015 Pressure Profile Systems
//
//  Licensed under the MIT license. This file may not be copied, modified, or
//  distributed except according to those terms.
//-----------------------------------------------------------------------------

#include <Wire.h> //For I2C/SMBus
#include <Timer1.h> //For timestamp

#define CMD_READ        0x01
#define CMD_WRITE       0x02
#define CMD_TOGGLE_GPIO 0x03
#define CMD_WRITE_CAL   0x04
#define TOGGLE_ALL      0x08
#define TOGGLE_NONE     0x00

//Constants
//General I2C packet structure
const byte I2C_HEADER1 = 0;
const byte I2C_HEADER2 = 1;
const byte I2C_HEADER3 = 2;
const byte I2C_HEADER4 = 3;
const byte I2C_ADDRESS_BYTE = 4;
const byte I2C_TIMOUT_BYTE = 5;
const byte I2C_ID_BYTE = 6;
const byte I2C_NUM_FOOTER_BYTES = 4;
const byte I2C_TO_SENSOR_BUFFER_LENGTH = 32;
const byte I2C_TIMEOUT = 100; //100ms

//From PC
const byte I2C_FROMPC_CMD = 7;
const byte I2C_FROMPC_READWRITE_LOCATION = 8;
const byte I2C_FROMPC_NUM_BYTES_TO_READWRITE = 9;
const byte SERIAL_FROMPC_BUFFER_LENGTH = 43;
const byte MINIMUM_FROMPC_PACKETLENGTH = 13; //I2C_FROMPC_NUM_BYTES_TO_READWRITE + 4 footer bytes

//To PC
const byte I2C_TOPC_TIMESTAMP = 7;
const byte I2C_TOPC_NBYTES = 11;
const byte SERIAL_TOPC_BUFFER_LENGTH = 77;

// Pin 13 has an LED connected.
int led = 13;

// Pin 2, 3, 4, 5, 6, 7 are reserved for Power Line
int singleTact1 = 2;
int singleTact2 = 3;
int singleTact3 = 4;
int singleTact4 = 5;
int singleTact5 = 6;
int singleTact6 = 7;

//From Arduino to sensor
byte outgoingI2CBuffer[I2C_TO_SENSOR_BUFFER_LENGTH];

//From Arduino to PC host
byte serialToPCBuffer[SERIAL_TOPC_BUFFER_LENGTH];
byte serialToPCBufferIndex_ = 0;

//From PC host to Arduino
byte serialIncomingBuffer[SERIAL_FROMPC_BUFFER_LENGTH];
byte serialIncomingBufferIndex_ = 0;

unsigned long timeStamp_;

//Zero a buffer
void BlankBuffer(byte* buffer, byte length)
{
  for(int i = 0; i < length; i++)
  {
    buffer[i] = 0;
  }
}

void ResetSerialBuffer()
{
  for(int i = 0; i < SERIAL_FROMPC_BUFFER_LENGTH; i++)
  {
    serialIncomingBuffer[i] = 0;
  }

  serialIncomingBufferIndex_ = 0;
}

void setup()
{
  int i;

  Wire.begin(); // join i2c bus (address optional for master)

  pinMode(led, OUTPUT);
  Serial.begin(115200);  // start serial for output
  Serial.flush();

  pinMode(singleTact1, OUTPUT);
  pinMode(singleTact2, OUTPUT);
  pinMode(singleTact3, OUTPUT);
  pinMode(singleTact4, OUTPUT);
  pinMode(singleTact5, OUTPUT);
  pinMode(singleTact6, OUTPUT);

  BlankBuffer(outgoingI2CBuffer, I2C_TO_SENSOR_BUFFER_LENGTH);

  //Never changes, so just set now
  serialToPCBuffer[I2C_HEADER1] = 0xFF;
  serialToPCBuffer[I2C_HEADER2] = 0xFF;
  serialToPCBuffer[I2C_HEADER3] = 0xFF;
  serialToPCBuffer[I2C_HEADER4] = 0xFF;

  digitalWrite(led, HIGH);

  timeStamp_ = 0;
  startTimer1(100); //Timer for timestamp
}


//Check the full footer
boolean Checkfooter(int endOfPacket)
{
  for(int i = 0; i < 4; i++)
  {
    if(serialIncomingBuffer[endOfPacket - i] != 0xFE)
    {
      return false;  //Footer corrupt
    }
  }

  return true; //Footer all good
}


//Check available header bytes (called as each one comes in, building upto 4)
boolean Checkheader(int checkMaxIndex)
{
  for(int j = 0; j <= checkMaxIndex && j < 4; j ++)
  {
    if(serialIncomingBuffer[j] != 0xFF)
    {
      return false; //Header corrupt
    }
  }
  return true; //Header all good
}

//Returns true if we have a new serial data packet, otherwise returns false
//Corrupt data is removed from buffer
boolean ProcessIncomingSerialData()
{
  byte i2cPacketLength; //Working variable

  if(Serial.available() > 0)
  {
    //Read the new byte
    serialIncomingBuffer[serialIncomingBufferIndex_] = Serial.read();
  }
  else
  {
    //No new data
    return false;
  }

  if(false == Checkheader(serialIncomingBufferIndex_))
  {
    ResetSerialBuffer();  //Header not correct, reset buffer
    return false;
  }

  //Do we have enough data to process
  if(serialIncomingBufferIndex_ >MINIMUM_FROMPC_PACKETLENGTH)
  {
    if(CMD_READ == serialIncomingBuffer[I2C_FROMPC_CMD])
    {
      i2cPacketLength = 0;
    }
    else
    {
      i2cPacketLength = serialIncomingBuffer[I2C_FROMPC_NUM_BYTES_TO_READWRITE];
    }


    //Do we have a full packet
    if(serialIncomingBufferIndex_ >= (i2cPacketLength + MINIMUM_FROMPC_PACKETLENGTH + 1))
    {
      if(Checkfooter(serialIncomingBufferIndex_))
      {
        //We have a good packet
        return true;
      }
      else
      {
        //Corrupt packet, reset
        ResetSerialBuffer();
        return false;
      }
    }
  }

  //We have run out of buffer space - something has gone wrong, so just reset the buffer
  if(serialIncomingBufferIndex_ >= (SERIAL_FROMPC_BUFFER_LENGTH -1))
  {
    ResetSerialBuffer();
  }

  serialIncomingBufferIndex_++; //Move the index on
  return false;  //Not at the end of the packet yet
}

// Define the function which will handle the notifications
ISR(timer1Event)
{
  resetTimer1();
  timeStamp_++;
}

void loop()
{
  byte inputBuffer[43];
  byte finishedPacket = false;
  byte timeout = 5;
  byte i = 0;
  byte lengthReceived = 0;

  if(ProcessIncomingSerialData())
  {
    if(CMD_WRITE == serialIncomingBuffer[I2C_FROMPC_CMD])
    {
      //We are changing settings
      if(serialIncomingBuffer[I2C_FROMPC_NUM_BYTES_TO_READWRITE] > 0)
      {
        //Create return packet for the PC
        serialToPCBuffer[I2C_ADDRESS_BYTE] = serialIncomingBuffer[I2C_ADDRESS_BYTE];
        serialToPCBuffer[I2C_TIMOUT_BYTE] = 0; //We did not timeout
        serialToPCBuffer[I2C_ID_BYTE] = serialIncomingBuffer[I2C_ID_BYTE];
        serialToPCBuffer[I2C_TOPC_NBYTES] = 0; //No data, just ack

        serialToPCBuffer[I2C_TOPC_NBYTES+1] = 0xFE;
        serialToPCBuffer[I2C_TOPC_NBYTES+2] = 0xFE;
        serialToPCBuffer[I2C_TOPC_NBYTES+3] = 0xFE;
        serialToPCBuffer[I2C_TOPC_NBYTES+4] = 0xFE;

        //Transfer data
        for(int i = 0; i < serialIncomingBuffer[I2C_FROMPC_NUM_BYTES_TO_READWRITE]+4; i++)
        outgoingI2CBuffer[i] = serialIncomingBuffer[i+I2C_FROMPC_CMD];

        //Send I2C packet
        Wire.beginTransmission(serialIncomingBuffer[I2C_ADDRESS_BYTE]); // transmit to device
        Wire.write(outgoingI2CBuffer, I2C_TO_SENSOR_BUFFER_LENGTH);
        Wire.endTransmission();    // stop transmitting

        //Send serial ack to PC
        for(int i = 0; i < (I2C_TOPC_NBYTES+5); i++)
        Serial.write(serialToPCBuffer[i]);
      }
    }
    else if(CMD_READ == serialIncomingBuffer[I2C_FROMPC_CMD])
    {
      byte i2cPacketLength = serialIncomingBuffer[I2C_FROMPC_NUM_BYTES_TO_READWRITE];

      serialToPCBuffer[I2C_ADDRESS_BYTE] = serialIncomingBuffer[I2C_ADDRESS_BYTE];
      serialToPCBuffer[I2C_TIMOUT_BYTE] = 0;
      serialToPCBuffer[I2C_ID_BYTE] = serialIncomingBuffer[I2C_ID_BYTE];
      serialToPCBuffer[I2C_TOPC_NBYTES] = i2cPacketLength;

      serialToPCBuffer[I2C_TOPC_TIMESTAMP] = (timeStamp_ >> 24);
      serialToPCBuffer[I2C_TOPC_TIMESTAMP + 1 ] = (timeStamp_ >> 16) & 0xFF;
      serialToPCBuffer[I2C_TOPC_TIMESTAMP + 2 ] = (timeStamp_ >> 8) & 0xFF;
      serialToPCBuffer[I2C_TOPC_TIMESTAMP + 3 ] = timeStamp_ & 0xFF;

      //Perform I2C Read
      outgoingI2CBuffer[0] = CMD_READ;
      outgoingI2CBuffer[1] = serialIncomingBuffer[I2C_FROMPC_READWRITE_LOCATION];
      outgoingI2CBuffer[2] = serialIncomingBuffer[I2C_FROMPC_NUM_BYTES_TO_READWRITE];

      //Send I2C packet
      Wire.beginTransmission(serialIncomingBuffer[I2C_ADDRESS_BYTE]); // transmit to device
      Wire.write(outgoingI2CBuffer, 3); //Only requires 3 bytes
      Wire.endTransmission();    // stop transmitting

      Wire.requestFrom(serialIncomingBuffer[I2C_ADDRESS_BYTE], serialIncomingBuffer[I2C_FROMPC_NUM_BYTES_TO_READWRITE]);

      //Parse request
      int i = 0;
      int i2cTimeout = I2C_TIMEOUT; //1s

      while( i < i2cPacketLength && i2cTimeout > 0)    // slave may send less than requested
      {
        if(Wire.available())
        {
          serialToPCBuffer[I2C_TOPC_NBYTES + 1 + i] = Wire.read(); // receive a byte as character
          i++;
        }
        else
        {
          delay(1); //Wait 1ms
          i2cTimeout--;
        }
      }

      if(0 == timeout)
      {
        serialToPCBuffer[I2C_TIMOUT_BYTE] = 1;
      }

      //Add footer
      serialToPCBuffer[I2C_TOPC_NBYTES + 1 + i2cPacketLength] = 0xFE;
      serialToPCBuffer[I2C_TOPC_NBYTES + 2 + i2cPacketLength] = 0xFE;
      serialToPCBuffer[I2C_TOPC_NBYTES + 3 + i2cPacketLength] = 0xFE;
      serialToPCBuffer[I2C_TOPC_NBYTES + 4 + i2cPacketLength] = 0xFE;

      //Send to PC
      for(int i = 0; i < (I2C_TOPC_NBYTES + 5 + i2cPacketLength); i++)
      Serial.write(serialToPCBuffer[i]);
    }

    // Transmit the Calibration Data to the Interface Board
    if(CMD_WRITE_CAL == serialIncomingBuffer[I2C_FROMPC_CMD])
    {
      //We are changing settings
      if(serialIncomingBuffer[I2C_FROMPC_NUM_BYTES_TO_READWRITE] > 0)
      {
        //Create return packet for the PC
        serialToPCBuffer[I2C_ADDRESS_BYTE] = serialIncomingBuffer[I2C_ADDRESS_BYTE];
        serialToPCBuffer[I2C_TIMOUT_BYTE] = 0; //We did not timeout
        serialToPCBuffer[I2C_ID_BYTE] = serialIncomingBuffer[I2C_ID_BYTE];
        serialToPCBuffer[I2C_TOPC_NBYTES] = 0; //No data, just ack

        serialToPCBuffer[I2C_TOPC_NBYTES+1] = 0xFE;
        serialToPCBuffer[I2C_TOPC_NBYTES+2] = 0xFE;
        serialToPCBuffer[I2C_TOPC_NBYTES+3] = 0xFE;
        serialToPCBuffer[I2C_TOPC_NBYTES+4] = 0xFE;

        //Transfer data
        for(int i = 0; i < serialIncomingBuffer[I2C_FROMPC_NUM_BYTES_TO_READWRITE]+4; i++)
        {
          outgoingI2CBuffer[i] = serialIncomingBuffer[i+I2C_FROMPC_CMD];
        }

        //Send I2C packet
        Wire.beginTransmission(serialIncomingBuffer[I2C_ADDRESS_BYTE]); // transmit to device
        Wire.write(outgoingI2CBuffer, I2C_TO_SENSOR_BUFFER_LENGTH);
        Wire.endTransmission();    // stop transmitting

        //Send serial ack to PC
        for(int i = 0; i < (I2C_TOPC_NBYTES+5); i++)
        {
          Serial.write(serialToPCBuffer[i]);
        }
      }

    }
    // Toggle the GPIO Pin to set the I2C address when doing Calibration test
    else if (CMD_TOGGLE_GPIO == serialIncomingBuffer[I2C_FROMPC_CMD])
    {
      serialToPCBuffer[I2C_ADDRESS_BYTE] = serialIncomingBuffer[I2C_ADDRESS_BYTE];
      serialToPCBuffer[I2C_TIMOUT_BYTE] = 0; //We did not timeout
      serialToPCBuffer[I2C_ID_BYTE] = serialIncomingBuffer[I2C_ID_BYTE];
      serialToPCBuffer[I2C_TOPC_NBYTES] = 0; //No data, just ack

      int toggle = serialToPCBuffer[I2C_FROMPC_CMD+1];
      if (TOGGLE_ALL == serialIncomingBuffer[I2C_FROMPC_CMD+1])
      {
        digitalWrite(singleTact1, HIGH);
        digitalWrite(singleTact2, HIGH);
        digitalWrite(singleTact3, HIGH);
        digitalWrite(singleTact4, HIGH);
        digitalWrite(singleTact5, HIGH);
        digitalWrite(singleTact6, HIGH);
      }
      else if (TOGGLE_NONE == serialIncomingBuffer[I2C_FROMPC_CMD+1])
      {
        digitalWrite(singleTact1, LOW);
        digitalWrite(singleTact2, LOW);
        digitalWrite(singleTact3, LOW);
        digitalWrite(singleTact4, LOW);
        digitalWrite(singleTact5, LOW);
        digitalWrite(singleTact6, LOW);
        Wire.endTransmission();
        delay(50);
        Wire.begin();
      }
      else
      {
        digitalWrite(singleTact1, LOW);
        digitalWrite(singleTact2, LOW);
        digitalWrite(singleTact3, LOW);
        digitalWrite(singleTact4, LOW);
        digitalWrite(singleTact5, LOW);
        digitalWrite(singleTact6, LOW);
        digitalWrite(serialIncomingBuffer[I2C_FROMPC_CMD+1], HIGH);
      }

      serialToPCBuffer[I2C_TOPC_NBYTES+1] = 0xFE;
      serialToPCBuffer[I2C_TOPC_NBYTES+2] = 0xFE;
      serialToPCBuffer[I2C_TOPC_NBYTES+3] = 0xFE;
      serialToPCBuffer[I2C_TOPC_NBYTES+4] = 0xFE;

      //Send serial ack to PC
      for(int i = 0; i < (I2C_TOPC_NBYTES+5); i++)
      {
        Serial.write(serialToPCBuffer[i]);
      }
    }

    ResetSerialBuffer();  //Reset and wait for next command
  }
}
