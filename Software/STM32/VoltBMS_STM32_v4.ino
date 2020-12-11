/*
  Chevy Volt BMS slaves decode

  030618

  Tom de Bree
  D. Maguire

  Update : 23/10/20
  STM32F103
V3 using exocan library and rx int
Works on E31 battery. Gen 1 volt.
V4 Experimental for new display board
*/
#include <Arduino.h>
#include <eXoCAN.h>
#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN2(PB10);     // Set CAN2 CS to pin PB10

HardwareSerial Serial2(PA3, PA2);

uint8_t counter = 0;
uint8_t frameLength = 0;
unsigned long previousMillis = 0;     // stores last time output was updated
const long interval = 1000;           // transmission interval (milliseconds)

 int txMsgID = 0x200;
 uint8_t txData[3] {0x02, 0x00, 0x00};
 uint8_t txDataLen = 3;


//Can variables
uint32_t tlast = 0;
uint32_t tlast2 = 0;
float Cell[97]; //cells, raw 0.00125/V
float Temp[24]; 
float celllow, cellhigh, templow, temphigh,packVoltage;
int id, fltIdx;
uint8_t rxbytes[8];

//Can 2 Variables
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

int led = PC13;
uint16_t MaxIndex = 0;
uint16_t MinIndex= 0;
float delta;

// 11b IDs, 125k bit rate, portB pins 8,9, no external resistor
eXoCAN can(STD_ID_LEN, BR125K, PORTB_8_9_WIRE_PULLUP); 

void can1ISR() // get CAN bus frame passed by a filter into fifo0
{
    can.receive(id, fltIdx, rxbytes);  // empties fifo0 so that another another rx interrupt can take place
    candecode();

}

void can2ISR() // get CAN bus frame passed by a filter into fifo0
{
    CAN2.readMsgBuf(&rxId, &len, rxBuf);      // Read data from CAN2
    can2decode();
}




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  //Initialize our USB port.
    Serial2.begin(9600);  //Nextion
    can.attachInterrupt(can1ISR); //internal can rx int
    attachInterrupt(PB0,can2ISR,FALLING); //CAN 2 rx interrupt
  // Initialize MCP25625 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN2.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) Serial.println("MCP25625 Initialized Successfully!");
  else Serial.println("Error Initializing MCP25625...");
   CAN2.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

  pinMode(led, OUTPUT);
  pinMode(PA4, OUTPUT); //Out1 Output driver
  pinMode(PB3, INPUT); //IN1 digital input
  pinMode(PB0, INPUT); //CAN2 INT
  pinMode(PB1, OUTPUT); //CAN2 standby signal
  digitalWrite(PB1,LOW);

}

void loop()
{
  // put your main code here, to run repeatedly:
///////////////////////////////////////////////////////////////////////////////////////////////////////


  if (tlast <  (millis() - 500)) // 2hz loop
  {
    tlast = millis();
    sendcommand();  //sends master command to ask slaves to read back
    sendcommand2();  //test
    processData();
    printcells();
    dashupdate();
    packVoltage=0;  //reset array adder
  }



  
}
////////////////////////////////////////////////////////////////////////////////////////////////////



void sendcommand()
{

can.transmit(txMsgID, txData, txDataLen);
   digitalWrite(led,!digitalRead(led));//blink led every time we fire this interrrupt.

}


void sendcommand2()
{
 byte txBuf[3] {0xDC, 0x06, 0x00};
   CAN2.sendMsgBuf(0x3D8,0,3,txBuf);      // 

}


void printtest()
{
    Serial.print("High Cell= ");
    Serial.println(cellhigh);
    Serial.print("Low Cell= ");
    Serial.println(celllow);
    Serial.print("Delta Volts= ");
    Serial.println(delta);
    
}


void processData()
{
celllow=4;
cellhigh=0;

templow = 50;
temphigh = 0;

for (int y = 1; y < 97; y++) //cell
{
    if (Cell[y] < celllow)
    {
      celllow = Cell[y];
    }
    if (Cell[y] > cellhigh)
    {
      cellhigh = Cell[y];
    }
}
for (int y = 1; y < 17; y++)
{
    if (Temp[y] < templow)
    {
      templow = Temp[y];
    }
    if (Temp[y] > temphigh)
    {
      temphigh = Temp[y];
    }
}

for (int i=0; i<97; i++)
{
   packVoltage += Cell[i];
}

delta = cellhigh -celllow;
}

void printcells()
{

  int x = 0;
  Serial.println();
  Serial.println("Cell Voltages");
  for (int y = 1; y < 97; y++)
  {
    Serial.print(Cell[y], 2);
    Serial.print(" | ");
    x++;
    if (x > 7)
    {
      x = 0;
      Serial.println();
    }
  }
  Serial.println();
  Serial.print("Lowest Cell: ");
  Serial.print(celllow , 2);
  Serial.print("  Highest Cell: ");
  Serial.print(cellhigh , 2);
  Serial.println();


  x = 0;
  for (int y = 1; y < 17; y++)
  {
 
    Serial.print(Temp[y], 2);
    Serial.print(" | ");

    x++;
    if (x > 7)
    {
      x = 0;
      Serial.println();
    }
  }
  Serial.println();
  Serial.print("Lowest Temp: ");
  Serial.print(templow, 2);
  Serial.print("  Highest Temp: ");
  Serial.println(temphigh, 2);
  Serial.print("Pack voltage= ");
  Serial.println(packVoltage);
  
  
}


void candecode()
{

//  int x = 0;
  switch (id)
  {
    case 0x460: //Module 1 cells 1-3
      Cell[1] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[2] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[3] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      
      break;
    case 0x470: //Module 1 cells 4-6
      Cell[4] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[5] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[6] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      break;
    case 0x461: //Module 1 cells 7-10
      Cell[7] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[8] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[9] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      Cell[10] = (((rxbytes[6] & 0x0F) << 8) + (rxbytes[7]))*0.00125;
      break;
    case 0x471: //Module 1 cells 11-14
      Cell[11] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[12] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[13] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      Cell[14] = (((rxbytes[6] & 0x0F) << 8) + (rxbytes[7]))*0.00125;
      break;
    case 0x462: //Module 1 cells 15-18
      Cell[15] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[16] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[17] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      Cell[18] = (((rxbytes[6] & 0x0F) << 8) + (rxbytes[7]))*0.00125;
      break;
    case 0x472: //Module 1 cells 19-22
      Cell[19] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[20] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[21] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      Cell[22] = (((rxbytes[6] & 0x0F) << 8) + (rxbytes[7]))*0.00125;
      break;
    case 0x463: //Module 1 cells 23-26
      Cell[23] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[24] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[25] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      Cell[26] = (((rxbytes[6] & 0x0F) << 8) + (rxbytes[7]))*0.00125;
      
      break;
    case 0x473: //Module 1 cells 27-30
      Cell[27] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[28] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[29] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      Cell[30] = (((rxbytes[6] & 0x0F) << 8) + (rxbytes[7]))*0.00125;
      break;
    case 0x464: //Module 2 cells 31-34
      Cell[31] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[32] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[33] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      Cell[34] = (((rxbytes[6] & 0x0F) << 8) + (rxbytes[7]))*0.00125;
      break;
    case 0x474: //Module 2 cells 35-38
      Cell[35] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[36] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[37] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      Cell[38] = (((rxbytes[6] & 0x0F) << 8) + (rxbytes[7]))*0.00125;
      break;
    case 0x465: //Module 2 cells 39-42
      Cell[39] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[40] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[41] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      Cell[42] = (((rxbytes[6] & 0x0F) << 8) + (rxbytes[7]))*0.00125;
      break;
    case 0x475: //Module 2 cells 43-46
      Cell[43] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[44] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[45] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      Cell[46] = (((rxbytes[6] & 0x0F) << 8) + (rxbytes[7]))*0.00125;
      break;
    case 0x466: //Module 2 cells 47-50
      Cell[47] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[48] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[49] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      Cell[50] = (((rxbytes[6] & 0x0F) << 8) + (rxbytes[7]))*0.00125;
      break;
    case 0x476: //Module 2 cells 51-54
      Cell[51] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[52] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[53] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      Cell[54] = (((rxbytes[6] & 0x0F) << 8) + (rxbytes[7]))*0.00125;
      break;
    case 0x468: //Module 3 cells 55-58
      Cell[55] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[56] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[57] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      Cell[58] = (((rxbytes[6] & 0x0F) << 8) + (rxbytes[7]))*0.00125;
      break;
    case 0x478: //Module 3 cells 59-62
      Cell[59] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[60] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[61] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      Cell[62] = (((rxbytes[6] & 0x0F) << 8) + (rxbytes[7]))*0.00125;
      break;
    case 0x469: //Module 3 cells 63-66
      Cell[63] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[64] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[65] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      Cell[66] = (((rxbytes[6] & 0x0F) << 8) + (rxbytes[7]))*0.00125;
      break;
    case 0x479: //Module 3 cells 67-70
      Cell[67] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[68] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[69] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      Cell[70] = (((rxbytes[6] & 0x0F) << 8) + (rxbytes[7]))*0.00125;
      break;
    case 0x46A: //Module 3 cells 71-74
      Cell[71] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[72] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[73] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      Cell[74] = (((rxbytes[6] & 0x0F) << 8) + (rxbytes[7]))*0.00125;
      break;
    case 0x47A: //Module 3 cells 75-78
      Cell[75] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[76] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[77] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      Cell[78] = (((rxbytes[6] & 0x0F) << 8) + (rxbytes[7]))*0.00125;
      break;

    case 0x46C: //Module 4 cells 79-81
      Cell[79] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[80] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[81] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      break;
    case 0x47C: //Module 4 cells 82-84
      Cell[82] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[83] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[84] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      break;
    case 0x46D: //Module 4 cells 85-87
      Cell[85] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[86] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[87] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      break;
    case 0x47D: //Module 4 cells 88-90
      Cell[88] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[89] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[90] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      break;
    case 0x46E: //Module 4 cells 91-93
      Cell[91] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[92] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[93] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      break;
    case 0x47E: //Module 4 cells 94-96
      Cell[94] = (((rxbytes[0] & 0x0F) << 8) + (rxbytes[1]))*0.00125;
      Cell[95] = (((rxbytes[2] & 0x0F) << 8) + (rxbytes[3]))*0.00125;
      Cell[96] = (((rxbytes[4] & 0x0F) << 8) + (rxbytes[5]))*0.00125;
      break;

    case 0x7E0: //Module 1 Temp 1
      Temp[1] = ((((rxbytes[6]) << 8) + (rxbytes[7]))*-0.0324)+150;
      break;
    case 0x7E1: //Module 1 Temp 2
      Temp[2] = ((((rxbytes[6]) << 8) + (rxbytes[7]))*-0.0324)+150;
      break;
    case 0x7E2: //Module 1 Temp 3-4
      Temp[3] = ((((rxbytes[6]) << 8) + (rxbytes[7]))*-0.0324)+150;
      Temp[4] = ((((rxbytes[6]) << 8) + (rxbytes[7]))*-0.0324)+150;
      break;
    case 0x7E3: //Module 1 Temp 5
      Temp[5] = ((((rxbytes[6]) << 8) + (rxbytes[7]))*-0.0324)+150;
      break;

    case 0x7E4: //Module 2 Temp 6
      Temp[6] = ((((rxbytes[6]) << 8) + (rxbytes[7]))*-0.0324)+150;
      break;
    case 0x7E5: //Module 2 Temp 7-8
      Temp[7] = ((((rxbytes[6]) << 8) + (rxbytes[7]))*-0.0324)+150;
      Temp[8] = ((((rxbytes[6]) << 8) + (rxbytes[7]))*-0.0324)+150;
      break;
    case 0x7E6: //Module 2 Temp 9
      Temp[9] = ((((rxbytes[6]) << 8) + (rxbytes[7]))*-0.0324)+150;
      break;

    case 0x7E8: //Module 3 Temp 10
      Temp[10] = ((((rxbytes[6]) << 8) + (rxbytes[7]))*-0.0324)+150;
      break;
    case 0x7E9: //Module 3 Temp 11-12
      Temp[11] = ((((rxbytes[6]) << 8) + (rxbytes[7]))*-0.0324)+150;
      Temp[12] = ((((rxbytes[6]) << 8) + (rxbytes[7]))*-0.0324)+150;
      break;
    case 0x7EA: //Module 3 Temp 13
      Temp[13] = ((((rxbytes[6]) << 8) + (rxbytes[7]))*-0.0324)+150;
      break;

    case 0x7EC: //Module 4 Temp 14
      Temp[14] = ((((rxbytes[6]) << 8) + (rxbytes[7]))*-0.0324)+150;
      break;
    case 0x7ED: //Module 4 Temp 15
      Temp[15] = ((((rxbytes[6]) << 8) + (rxbytes[7]))*-0.0324)+150;
      break;
    case 0x7EE: //Module 4 Temp 16
      Temp[16] = ((((rxbytes[6]) << 8) + (rxbytes[7]))*-0.0324)+150;
      break;
  }
}

void can2decode()
{
  
}


void dashupdate()
{

    Serial2.write("stat.txt=");
    Serial2.write(0x22);
     Serial2.print(" Ready ");
    Serial2.write(0x22);
    Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("soc.val=");
    Serial2.print(50,DEC);//test value
    //Serial2.print(SOC);
    Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("soc1.val=");
    Serial2.print(50,DEC);//test value
    //Serial2.print(SOC);
    Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("current.val=");
    Serial2.print(500,DEC);//test value
    //Serial2.print(abs(currentact) / 1000, 0);
    Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("temp.val=");
    Serial2.print(temphigh, 0);
    //Serial2.print(bms.getAvgTemperature(), 0);
    Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("templow.val=");
    Serial2.print(templow, 0);
    Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("temphigh.val=");
    Serial2.print(temphigh, 0);
    Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("volt.val=");
    Serial2.print((packVoltage*10),0);//test value
    //Serial2.print(bms.getPackVoltage(), 0);
    Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("lowcell.val=");
    Serial2.print(celllow * 1000, 0);
    Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("highcell.val=");
    Serial2.print(cellhigh * 1000, 0);
    Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("firm.val=");
    Serial2.print(5,DEC);//test value
    //Serial2.print(firmver);
    Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);
    
  }
