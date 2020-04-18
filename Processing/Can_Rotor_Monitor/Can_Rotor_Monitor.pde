import processing.serial.*;
import controlP5.*; // controlP5 library
import processing.opengl.*; 

import java.lang.StringBuffer; // for efficient String concatemation
import javax.swing.SwingUtilities; // required for swing and EDT
import javax.swing.JFileChooser; // Saving dialogue
import javax.swing.filechooser.FileFilter; // for our configuration file filter "*.mwi"
import javax.swing.JOptionPane; // for message dialogue

//Added For  Processing 2.0.x compabillity
import java.util.*;
import java.io.*;
//****************************

Serial port;
ControlP5 controlP5;

static int PIDITEMS=3;
int tabHeight=20; // Extra height needed for Tabs
cGraph g_graph;
int windowsX    = 1000;       int windowsY    = 550+tabHeight;
int xGraph      = 10;         int yGraph      = 325+tabHeight;
int xObj        = 520;        int yObj        = 293+tabHeight;
int xCompass    = 920;        int yCompass    = 341+tabHeight;
int xLevelObj   = 920;        int yLevelObj   = 80+tabHeight; 
int xParam      = 120;        int yParam      = 5+tabHeight;
int xRC         = 690;        int yRC         = 10+tabHeight; 
int xMot        = 690;        int yMot        = 155+tabHeight;
int xGPS        = 853;        int yGPS        = 438+tabHeight;

String myString = null;

int i, j; // enummerators
int horizonInstrSize, GPS_distanceToHome, GPS_directionToHome,
    GPS_numSat, GPS_fix, GPS_update, GPS_altitude, GPS_speed,
    GPS_latitude, GPS_longitude;

int error_count;

int error, ARMED, HEADFREE_MODE, ANGLE_MODE, HORIZON_MODE, ACRO_MODE, cycleTime, VBAT, Temperature, EstAlt, graph_on;

int hours, minutes, seconds;

int armedTime;

int com_time;

int byteP[] = new int[PIDITEMS], byteI[] = new int[PIDITEMS], byteD[] = new int[PIDITEMS],
    byteP_outer[] = new int[PIDITEMS], byteI_outer[] = new int[PIDITEMS], byteP_inner[] = new int[PIDITEMS], byteI_inner[] = new int[PIDITEMS], byteD_inner[] = new int[PIDITEMS],
    byteP_rate[] = new int[PIDITEMS], byteI_rate[] = new int[PIDITEMS], byteD_rate[] = new int[PIDITEMS];

float roll, pitch, yaw, heading, kp[], ki[], kd[], kp_dual_outer[], ki_dual_outer[], kp_dual_inner[], ki_dual_inner[], kd_dual_inner[], kp_rate[], ki_rate[], kd_rate[];

int mot[] = new int[4],
    RCChan[] = new int[16];

int satellites, latitudeDegrees, longitudeDegrees;

float gx, gy, gz, ax, ay, az, magx, magy, magz, head, angx, angy,
      angyLevelControl, angCalc;

int alt;

boolean axGraph =true,ayGraph=true,azGraph=true,gxGraph=true,gyGraph=true,gzGraph=true,altGraph=true,headGraph=true, magxGraph =true,magyGraph=true,magzGraph=true,
        debug1Graph = false,debug2Graph = false,debug3Graph = false,debug4Graph = false,hideDraw=false,GraphicsInited=false,gimbalConfig=false,flapperons=false,
        flaps=false,InitServos=true;

private static final int ROLL = 0, PITCH = 1, YAW = 2;

static int RCThro = 3, RCRoll = 0, RCPitch =1, RCYaw =2, RCAUX1=4, RCAUX2=5, RCAUX3=6, RCAUX4=7;

cDataArray accPITCH   = new cDataArray(200), accROLL    = new cDataArray(200), accYAW     = new cDataArray(200),
           gyroPITCH  = new cDataArray(200), gyroROLL   = new cDataArray(200), gyroYAW    = new cDataArray(200),
           magxData   = new cDataArray(200), magyData   = new cDataArray(200), magzData   = new cDataArray(200),
           imuPITCH   = new cDataArray(200), imuROLL    = new cDataArray(200), imuYAW     = new cDataArray(200),
           altData    = new cDataArray(200), headData   = new cDataArray(200);

Numberbox confP[]   = new Numberbox[PIDITEMS],
          confI[]   = new Numberbox[PIDITEMS],
          confD[]   = new Numberbox[PIDITEMS],
          
          confP_dual_outer[]   = new Numberbox[PIDITEMS],
          confI_dual_outer[]   = new Numberbox[PIDITEMS],
          confP_dual_inner[]   = new Numberbox[PIDITEMS],
          confI_dual_inner[]   = new Numberbox[PIDITEMS],
          confD_dual_inner[]   = new Numberbox[PIDITEMS],
          
          confP_rate[]   = new Numberbox[PIDITEMS],
          confI_rate[]   = new Numberbox[PIDITEMS],
          confD_rate[]   = new Numberbox[PIDITEMS],
          confPowerTrigger
          ;
          
Slider axSlider, aySlider, azSlider, gxSlider, gySlider, gzSlider, magxSlider, magySlider,
       magzSlider, altSlider, headSlider;          

Slider headingSlider, rollSlider, pitchSlider, yawSlider, scaleSlider;
Slider motSlider[]        = new Slider[5],
       TX_StickSlider[]   = new Slider[8];
       
Button buttonREAD, buttonRESET, buttonWRITE, buttonSAVE, buttonSTART, buttonSTOP, buttonCALIBRATE_ACC, buttonCALIBRATE_MAG, buttonMANUAL_MOTOR;       
       
Toggle tACC_ROLL, tACC_PITCH, tACC_Z, tGYRO_ROLL, tGYRO_PITCH, tGYRO_YAW, tBARO,tHEAD, tMAGX, tMAGY, tMAGZ, 
        tDEBUG1, tDEBUG2, tDEBUG3, tDEBUG4;       
       
       
// Colors
color yellow_ = color(200, 200, 20), green_ = color(30, 120, 30), red_ = color(120, 30, 30), blue_ = color(50, 50, 100),
       grey_ = color(30, 30, 30),black_ = color(0, 0, 0),orange_ =color(200,128,0);

PFont font8, font9, font12, font15;

controlP5.Controller hideLabel(controlP5.Controller c) {
  c.setLabel("");
  c.setLabelVisible(false);
  return c;
}

void setup(){
size(1000, 570, P3D);
  font8 = createFont("Arial bold",8,false);
  font9 = createFont("Arial bold",9,false);
  font12 = createFont("Arial bold",12,false);
  font15 = createFont("Arial bold",15,false);
noStroke();
printArray(Serial.list());
port = new Serial(this,Serial.list()[1],57600);//57600
controlP5 = new ControlP5(this);

  color c,black;
  black = color(0,0,0);
  int xo = xGraph-7;
  int x = xGraph+40;
  int y1= yGraph+10;  //ACC
  int y2= yGraph+55;  //GYRO
  int y5= yGraph+100; //MAG
  int y3= yGraph+150; //ALT
  int y4= yGraph+165; //HEAD
  int y7= yGraph+185; //GPS
  int y6= yGraph+205; //DEBUG
  
  tACC_ROLL =       controlP5.addToggle("ACC_ROLL",true,x,y1+10,20,10).setColorActive(color(255, 0, 0)).setColorBackground(black).setLabel(""); 
  tACC_PITCH =      controlP5.addToggle("ACC_PITCH",true,x,y1+20,20,10).setColorActive(color(0, 255, 0)).setColorBackground(black).setLabel(""); 
  tACC_Z =          controlP5.addToggle("ACC_Z",true,x,y1+30,20,10).setColorActive(color(0, 0, 255)).setColorBackground(black).setLabel(""); 
  tGYRO_ROLL =      controlP5.addToggle("GYRO_ROLL",true,x,y2+10,20,10).setColorActive(color(200, 200, 0)).setColorBackground(black).setLabel(""); 
  tGYRO_PITCH =     controlP5.addToggle("GYRO_PITCH",true,x,y2+20,20,10).setColorActive(color(0, 255, 255)).setColorBackground(black).setLabel(""); 
  tGYRO_YAW =       controlP5.addToggle("GYRO_YAW",true,x,y2+30,20,10).setColorActive(color(255, 0, 255)).setColorBackground(black).setLabel(""); 
  tBARO   =         controlP5.addToggle("BARO",true,x,y3 ,20,10).setColorActive(color(125, 125, 125)).setColorBackground(black).setLabel(""); 
  tHEAD   =         controlP5.addToggle("HEAD",true,x,y4 ,20,10).setColorActive(color(225, 225, 125)).setColorBackground(black).setLabel(""); 
  tMAGX   =         controlP5.addToggle("MAGX",true,x,y5+10,20,10).setColorActive(color(50, 100, 150)).setColorBackground(black).setLabel(""); 
  tMAGY   =         controlP5.addToggle("MAGY",true,x,y5+20,20,10).setColorActive(color(100, 50, 150)).setColorBackground(black).setLabel(""); 
  tMAGZ   =         controlP5.addToggle("MAGZ",true,x,y5+30,20,10).setColorActive(color(150, 100, 50)).setColorBackground(black).setLabel(""); 

  controlP5.addTextlabel( "alarmLabel", "Alarm:", xGraph -5, yGraph -32).setFont(font12);

  controlP5.addTextlabel("acclabel","ACC",xo,y1 -4).setFont(font12);
  controlP5.addTextlabel("accrolllabel","   ROLL",xo,y1+10).setFont(font9);
  controlP5.addTextlabel("accpitchlabel","   PITCH",xo,y1+20).setFont(font9);
  controlP5.addTextlabel("acczlabel","   Z",xo,y1+30).setFont(font9);
  controlP5.addTextlabel("gyrolabel","GYRO",xo,y2 -4).setFont(font12);
  controlP5.addTextlabel("gyrorolllabel","   ROLL",xo,y2+10).setFont(font9);
  controlP5.addTextlabel("gyropitchlabel","   PITCH",xo,y2+20).setFont(font9);
  controlP5.addTextlabel("gyroyawlabel","   YAW",xo,y2+30).setFont(font9);
  controlP5.addTextlabel("maglabel","MAG",xo,y5 -4).setFont(font12);
  controlP5.addTextlabel("magrolllabel","   ROLL",xo,y5+10).setFont(font9);
  controlP5.addTextlabel("magpitchlabel","   PITCH",xo,y5+20).setFont(font9);
  controlP5.addTextlabel("magyawlabel","   YAW",xo,y5+30).setFont(font9);
  controlP5.addTextlabel("altitudelabel","ALT",xo,y3 -4).setFont(font12);
  controlP5.addTextlabel("headlabel","HEAD",xo,y4 -4).setFont(font12);
  
  axSlider      =  controlP5.addSlider("axSlider",-1000,+1000,0,x+20,y1+10,50,10).setDecimalPrecision(0).setLabel("");
  aySlider      =    controlP5.addSlider("aySlider",-1000,+1000,0,x+20,y1+20,50,10).setDecimalPrecision(0).setLabel("");
  azSlider      =    controlP5.addSlider("azSlider",-1000,+1000,0,x+20,y1+30,50,10).setDecimalPrecision(0).setLabel("");
  gxSlider      =    controlP5.addSlider("gxSlider",-5000,+5000,0,x+20,y2+10,50,10).setDecimalPrecision(0).setLabel("");
  gySlider      =    controlP5.addSlider("gySlider",-5000,+5000,0,x+20,y2+20,50,10).setDecimalPrecision(0).setLabel("");
  gzSlider      =    controlP5.addSlider("gzSlider",-5000,+5000,0,x+20,y2+30,50,10).setDecimalPrecision(0).setLabel("");
  altSlider     =    controlP5.addSlider("altSlider",-30000,+30000,0,x+20,y3 ,50,10).setDecimalPrecision(2).setLabel("");
  headSlider    =    controlP5.addSlider("headSlider",-370,+370,0,x+20,y4  ,50,10).setDecimalPrecision(0).setLabel("");
  magxSlider    =    controlP5.addSlider("magxSlider",-5000,+5000,0,x+20,y5+10,50,10).setDecimalPrecision(0).setLabel("");
  magySlider    =    controlP5.addSlider("magySlider",-5000,+5000,0,x+20,y5+20,50,10).setDecimalPrecision(0).setLabel("");
  magzSlider    =    controlP5.addSlider("magzSlider",-5000,+5000,0,x+20,y5+30,50,10).setDecimalPrecision(0).setLabel("");

g_graph  = new cGraph(xGraph+110,yGraph, 480, 200);
headingSlider  = controlP5.addSlider("heading", 0, 360, 0, 450+50, 170, 10, 100).setLabel("Heading").setDecimalPrecision(0);
rollSlider  = controlP5.addSlider("roll", -180, 180, 0, 450+100, 170, 10, 100).setLabel("ROLL").setDecimalPrecision(0);
pitchSlider  = controlP5.addSlider("pitch", -180, 180, 0, 450+150, 170, 10, 100).setLabel("PITCH").setDecimalPrecision(0);
yawSlider  = controlP5.addSlider("yaw", -180, 180, 0, 450+200, 170, 10, 100).setLabel("YAW").setDecimalPrecision(0);

  for( i=0;i<PIDITEMS;i++) {
    confP[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confP"+i,0,xParam+40,yParam+20+i*17,30,14));
    confP[i].setColorBackground(red_).setMin(0).setDirection(Controller.HORIZONTAL).setDecimalPrecision(1).setMultiplier(0.1).setMax(20);
    confI[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confI"+i,0,xParam+75,yParam+20+i*17,40,14));
    confI[i].setColorBackground(red_).setMin(0).setDirection(Controller.HORIZONTAL).setDecimalPrecision(1).setMultiplier(0.1).setMax(20);
    confD[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confD"+i,0,xParam+120,yParam+20+i*17,30,14));
    confD[i].setColorBackground(red_).setMin(0).setDirection(Controller.HORIZONTAL).setDecimalPrecision(2).setMultiplier(0.01).setMax(20);
  }
           
  for( i=0;i<PIDITEMS;i++) {
    confP_dual_outer[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confP_dual_outer"+i,0,xParam+40,yParam+80+i*17,30,14));
    confP_dual_outer[i].setColorBackground(red_).setMin(0).setDirection(Controller.HORIZONTAL).setDecimalPrecision(1).setMultiplier(0.1).setMax(20);
    confI_dual_outer[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confI_dual_outer"+i,0,xParam+75,yParam+80+i*17,40,14));
    confI_dual_outer[i].setColorBackground(red_).setMin(0).setDirection(Controller.HORIZONTAL).setDecimalPrecision(1).setMultiplier(0.1).setMax(20);
    confP_dual_inner[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confP_dual_inner"+i,0,xParam+40,yParam+130+i*17,30,14));
    confP_dual_inner[i].setColorBackground(red_).setMin(0).setDirection(Controller.HORIZONTAL).setDecimalPrecision(1).setMultiplier(0.1).setMax(20);
    confI_dual_inner[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confI_dual_inner"+i,0,xParam+75,yParam+130+i*17,40,14));
    confI_dual_inner[i].setColorBackground(red_).setMin(0).setDirection(Controller.HORIZONTAL).setDecimalPrecision(1).setMultiplier(0.1).setMax(20);
    confD_dual_inner[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confD_dual_inner"+i,0,xParam+120,yParam+130+i*17,30,14));
    confD_dual_inner[i].setColorBackground(red_).setMin(0).setDirection(Controller.HORIZONTAL).setDecimalPrecision(2).setMultiplier(0.01).setMax(20);
  }
  
  for( i=0;i<PIDITEMS;i++) {
    confP_rate[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confP_rate"+i,0,xParam+40,yParam+190+i*17,30,14));
    confP_rate[i].setColorBackground(red_).setMin(0).setDirection(Controller.HORIZONTAL).setDecimalPrecision(1).setMultiplier(0.1).setMax(20);
    confI_rate[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confI_rate"+i,0,xParam+75,yParam+190+i*17,40,14));
    confI_rate[i].setColorBackground(red_).setMin(0).setDirection(Controller.HORIZONTAL).setDecimalPrecision(1).setMultiplier(0.1).setMax(20);
    confD_rate[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confD_rate"+i,0,xParam+120,yParam+190+i*17,30,14));
    confD_rate[i].setColorBackground(red_).setMin(0).setDirection(Controller.HORIZONTAL).setDecimalPrecision(2).setMultiplier(0.01).setMax(20);
  }
  
  buttonREAD  = controlP5.addButton("READ" , 1, xParam+5,   yParam+260, 50, 16)  .setColorBackground(red_);
  buttonRESET = controlP5.addButton("RESET", 1, xParam+115,  yParam+260, 60, 16)  .setColorBackground(red_);
  buttonWRITE = controlP5.addButton("WRITE", 1, xParam+60, yParam+260, 50, 16)  .setColorBackground(red_);
  buttonSAVE  = controlP5.addButton("SAVE",  1,  xParam+290, yParam+260, 50, 16).setColorBackground(red_);
  buttonSTART  = controlP5.addButton("bSTART",1,xGraph+110,yGraph-25,45,19).setLabel("START").setColorBackground(red_);
  buttonSTOP   = controlP5.addButton("bSTOP",1,xGraph+160,yGraph-25,45,19).setLabel("STOP").setColorBackground(red_);
  buttonCALIBRATE_ACC = controlP5.addButton("CALIB_ACC",  1,  xParam+390, yParam+260, 70, 16).setColorBackground(red_);
  buttonCALIBRATE_MAG = controlP5.addButton("CALIB_MAG",  1,  xParam+470, yParam+260, 73, 16).setColorBackground(red_);
  buttonMANUAL_MOTOR = controlP5.addButton("MANUAL_MOTOR",  1,  xParam+600, yParam+310, 73, 16).setColorBackground(red_);
  
// Sliders for Transmitter
  TX_StickSlider[RCRoll ] =  controlP5.addSlider("Roll", -360,360,0,xRC,yRC+15,100,10)  .setDecimalPrecision(0);
  TX_StickSlider[RCPitch] =  controlP5.addSlider("Pitch",-360,360,0,xRC,yRC+30,100,10)  .setDecimalPrecision(0);
  TX_StickSlider[RCThro ] =  controlP5.addSlider("Throt",0,4500,0,xRC,yRC,100,10)     .setDecimalPrecision(0);
  TX_StickSlider[RCYaw ]  =  controlP5.addSlider("Yaw",  -200,200,0,xRC,yRC+45,100,10)  .setDecimalPrecision(0);
  TX_StickSlider[RCAUX1]  =  controlP5.addSlider("AUX1", 1000,2000,1500,xRC,yRC+60,100,10) .setDecimalPrecision(0);
  TX_StickSlider[RCAUX2]  =  controlP5.addSlider("AUX2", 1000,2000,1500,xRC,yRC+75,100,10) .setDecimalPrecision(0);
  TX_StickSlider[RCAUX3]  =  controlP5.addSlider("AUX3", 1000,2000,1500,xRC,yRC+90,100,10) .setDecimalPrecision(0);
  TX_StickSlider[RCAUX4]  =  controlP5.addSlider("AUX4", 1000,2000,1500,xRC,yRC+105,100,10).setDecimalPrecision(0);

  for( i=0;i<5;i++) {
    motSlider[i]     = controlP5.addSlider("motSlider"+i,2250,4500,0,0,0,10,100).setDecimalPrecision(0).hide();
  }
    motSlider[4].setPosition(xMot+50,yMot+20).setHeight(60).setCaptionLabel("MASTER_SET").hide();
background(100);

  scaleSlider = controlP5.addSlider("SCALE",0,10,1,xGraph+515,yGraph,75,20).setLabel("");// GraphScaler
  
   confPowerTrigger = controlP5.addNumberbox("",0,xGraph+50,yGraph-29,40,14).setDecimalPrecision(0).setMultiplier(10)
  .setDirection(Controller.HORIZONTAL).setMin(0).setMax(65535).setColorBackground(red_);
}

/******************************* Multiwii Serial Protocol **********************/
List<Character> payload;
private static final String MSP_HEADER = "$M<";

private static final int
  MSP_IDENT                =100,
  MSP_STATUS               =101,
  MSP_RAW_IMU              =102,
  MSP_SERVO                =103,
  MSP_MOTOR                =104,
  MSP_RC                   =105,
  MSP_RAW_GPS              =106,
  MSP_COMP_GPS             =107,
  MSP_ATTITUDE             =108,
  MSP_ALTITUDE             =109,
  MSP_ANALOG               =110,
  MSP_RC_TUNING            =111,
  MSP_PID                  =112,
  MSP_BOX                  =113,
  MSP_MISC                 =114,
  MSP_MOTOR_PINS           =115,
  MSP_BOXNAMES             =116,
  MSP_PIDNAMES             =117,
  MSP_SERVO_CONF           =120,
  MSP_RESET                =121,
    
  
  MSP_SET_RAW_RC           =200,
  MSP_SET_RAW_GPS          =201,
  MSP_SET_PID              =202,
  MSP_SET_BOX              =203,
  MSP_SET_RC_TUNING        =204,
  MSP_ACC_CALIBRATION      =205,
  MSP_MAG_CALIBRATION      =206,
  MSP_SET_MISC             =207,
  MSP_RESET_CONF           =208,
  MSP_SELECT_SETTING       =210,
  MSP_SET_HEAD             =211, // Not used
  MSP_SET_SERVO_CONF       =212,
  MSP_SET_MOTOR            =214,
  
  
  MSP_BIND                 =241,

  MSP_EEPROM_WRITE         =250,
  
  MSP_DEBUGMSG             =253,
  MSP_DEBUG                =254,
  
  TELEMERY_BAT_VOLT        =5,
  TELEMERY_TEMPERATURE     =6,

  TELEMERY_HEADING         =10,

  TELEMERY_BARO_EST        =21,
  TELEMERY_PID_RP_P        =22,
  TELEMERY_PID_RP_I        =23,
  TELEMERY_PID_RP_D        =24,
  TELEMERY_PID_Y_P         =25,
  TELEMERY_PID_Y_I         =26,
  TELEMERY_PID_Y_D         =27,
  TELEMERY_PID_SAVE        =56
  ;

public static final int
  IDLE = 0,
  HEADER_START = 1,
  HEADER_M = 2,
  HEADER_ARROW = 3,
  HEADER_SIZE = 4,
  HEADER_CMD = 5,
  HEADER_ERR = 6
;

int c_state = IDLE;
boolean err_rcvd = false;

byte checksum=0;
byte cmd;
int offset=0, dataSize=0;
byte[] inBuf = new byte[256];


int p;
int read32() {return (inBuf[p++]&0xff) + ((inBuf[p++]&0xff)<<8) + ((inBuf[p++]&0xff)<<16) + ((inBuf[p++]&0xff)<<24); }
int read16() {return (inBuf[p++]&0xff) + ((inBuf[p++])<<8); }
int read8()  {return  inBuf[p++]&0xff;}

int mode;
boolean toggleRead = true, toggleReset = false,toggleWrite = false, toggleSave=false, toggleCalibAcc = false,toggleCalibMag = false, toggleMotor = false;

//send msp without payload
private List<Byte> requestMSP(int msp) {
  return  requestMSP( msp, null);
}

//send multiple msp without payload
private List<Byte> requestMSP (int[] msps) {
  List<Byte> s = new LinkedList<Byte>();
  for (int m : msps) {
    s.addAll(requestMSP(m, null));
  }
  return s;
}

//send msp with payload
private List<Byte> requestMSP (int msp, Character[] payload) {
  if(msp < 0) {
   return null;
  }
  List<Byte> bf = new LinkedList<Byte>();
  for (byte c : MSP_HEADER.getBytes()) {
    bf.add( c );
  }
  
  byte checksum=0;
  byte pl_size = (byte)((payload != null ? int(payload.length) : 0)&0xFF);
  bf.add(pl_size);
  checksum ^= (pl_size&0xFF);
  
  bf.add((byte)(msp & 0xFF));
  checksum ^= (msp&0xFF);
  
  if (payload != null) {
    for (char c :payload){
      bf.add((byte)(c&0xFF));
      checksum ^= (c&0xFF);
    }
  }
  bf.add(checksum);
  return (bf);
}

void sendRequestMSP(List<Byte> msp) {
  byte[] arr = new byte[msp.size()];
  int i = 0;
  for (byte b: msp) {
    arr[i++] = b;
  }
  port.write(arr); // send the complete byte sequence in one go
}

public void evaluateCommand(byte cmd, int dataSize) {
  int i;
  int icmd = (int)(cmd&0xFF);
  switch(icmd) {
     case MSP_STATUS:
           armedTime = (int)read32();
           minutes = (int)(armedTime/60000000);
           seconds = (int)(armedTime-minutes*60000000)/1000000;
           
           cycleTime = read32();
           
           error = read8();
           
           mode  = read8();
           if((mode&(1<<0))>0){
             ARMED = 1;
           }else{
             ARMED = 0;
           }
           
           if((mode&(1<<1))>0){
             HEADFREE_MODE = 1;
           }else{
             HEADFREE_MODE = 0;
           } 
     break;
     
     case MSP_ATTITUDE:
        roll = read16()/10;pitch = read16()/10;
        yaw = read16();
        heading = read16();
        //head = heading-180;
     break;
                   
     case MSP_ALTITUDE:
     alt = read16();
     break;
              
     case MSP_RAW_IMU:
        ax = read16();ay = read16();az = read16();
        gx = read16();gy = read16();gz = read16();
        magx = read16();magy = read16();magz = read16(); 
        break;
        
      case MSP_RAW_GPS:
        GPS_fix = read8();
        satellites = read8();
        latitudeDegrees = read32();
        longitudeDegrees = read32();
        
        GPS_altitude = read16();
        //GPS_speed = read16();
        break;
        
       case MSP_MOTOR:
        for(i=0;i<4;i++){ mot[i] = read16();} 
        motSlider[0].setValue(mot[0]).show();
        motSlider[1].setValue(mot[1]).show();
        motSlider[2].setValue(mot[2]).show();
        motSlider[3].setValue(mot[3]).show();
        break; 
        
     case MSP_RC:
        for(i=0;i<6;i++) {
          RCChan[i]=read16();
          TX_StickSlider[i].setValue(RCChan[i]);
        }
        break;    
             
     case MSP_PID:
          for(i=0;i<PIDITEMS;i++) {
          byteP[i] = read16();byteI[i] = read16();byteD[i] = read16();          
          byteP_outer[i] = read16();byteI_outer[i] = read16();byteP_inner[i] = read16();byteI_inner[i] = read16();byteD_inner[i] = read16();
          byteP_rate[i] = read16();byteI_rate[i] = read16();byteD_rate[i] = read16();
          switch (i) {
           case 0:confP[i].setValue(byteP[i]/10.0);confI[i].setValue(byteI[i]/10.0);confD[i].setValue(byteD[i]/100.0);
                  confP_dual_outer[i].setValue(byteP_outer[i]/10.0);confI_dual_outer[i].setValue(byteI_outer[i]/10.0);
                  confP_dual_inner[i].setValue(byteP_inner[i]/10.0);confI_dual_inner[i].setValue(byteI_inner[i]/10.0);confD_dual_inner[i].setValue(byteD_inner[i]/100.0);
                  confP_rate[i].setValue(byteP_rate[i]/10.0);confI_rate[i].setValue(byteI_rate[i]/10.0);confD_rate[i].setValue(byteD_rate[i]/100.0);
           break;
           
           case 1:confP[i].setValue(byteP[i]/10.0);confI[i].setValue(byteI[i]/10.0);confD[i].setValue(byteD[i]/100.0);
                  confP_dual_outer[i].setValue(byteP_outer[i]/10.0);confI_dual_outer[i].setValue(byteI_outer[i]/10.0);
                  confP_dual_inner[i].setValue(byteP_inner[i]/10.0);confI_dual_inner[i].setValue(byteI_inner[i]/10.0);confD_dual_inner[i].setValue(byteD_inner[i]/100.0);           
                  confP_rate[i].setValue(byteP_rate[i]/10.0);confI_rate[i].setValue(byteI_rate[i]/10.0);confD_rate[i].setValue(byteD_rate[i]/100.0);
           break;
           
           case 2:confP[i].setValue(byteP[i]/10.0);confI[i].setValue(byteI[i]/10.0);confD[i].setValue(byteD[i]/100.0);
                  confP_dual_outer[i].setValue(byteP_outer[i]/10.0);confI_dual_outer[i].setValue(byteI_outer[i]/10.0);
                  confP_dual_inner[i].setValue(byteP_inner[i]/10.0);confI_dual_inner[i].setValue(byteI_inner[i]/10.0);confD_dual_inner[i].setValue(byteD_inner[i]/100.0);          
                  confP_rate[i].setValue(byteP_rate[i]/10.0);confI_rate[i].setValue(byteI_rate[i]/10.0);confD_rate[i].setValue(byteD_rate[i]/100.0);
           break;
          }
          confP[i].setColorBackground(green_);confI[i].setColorBackground(green_);confD[i].setColorBackground(green_);
          confP_dual_outer[i].setColorBackground(green_);confI_dual_outer[i].setColorBackground(green_);
          confP_dual_inner[i].setColorBackground(green_);confI_dual_inner[i].setColorBackground(green_);confD_dual_inner[i].setColorBackground(green_);
          confP_rate[i].setColorBackground(green_);confI_rate[i].setColorBackground(green_);confD_rate[i].setColorBackground(green_);
        }     
        break;
        
    case MSP_ANALOG:
        VBAT = read16();
        Temperature = read16()/10;
        break; 
        
    case MSP_MISC:
        for(i=0;i<6;i++) {
          RCChan[i]=read16();
          TX_StickSlider[i].setValue(RCChan[i]);
        }
          armedTime = (int)read32();
          minutes = (int)(armedTime/60000000);
          seconds = (int)(armedTime-minutes*60000000)/1000000;
          cycleTime = read32();
          error = read8();
          mode  = read8();
          if((mode&(1<<0))>0){
            ARMED = 1;
          }else{
            ARMED = 0;
          }
          if((mode&(1<<1))>0){
            HEADFREE_MODE = 1;
          }else{
            HEADFREE_MODE = 0;
          }
          if((mode&(1<<2))>0){
             ANGLE_MODE = 1;
           }else{
             ANGLE_MODE = 0;
           } 
          if((mode&(1<<3))>0){
             HORIZON_MODE = 1;
           }else{
             HORIZON_MODE = 0;
           } 
           if((mode&(1<<4))>0){
             ACRO_MODE = 1;
           }else{
             ACRO_MODE = 0;
           }
           if((mode&(1<<5))>0){
             buttonCALIBRATE_ACC.setColorBackground(green_);
           }else{
             buttonCALIBRATE_ACC.setColorBackground(red_);
           } 
           if((mode&(1<<6))>0){
             buttonCALIBRATE_MAG.setColorBackground(green_);
           }else{
             buttonCALIBRATE_MAG.setColorBackground(red_);
           } 
        roll = read16()/10;
        pitch = read16()/10;
        yaw = read16();
        heading = read16();
        alt = read16();
        VBAT = read16();
        Temperature = read16()/10;
        ax = read16();ay = read16();az = read16();
        gx = read16();gy = read16();gz = read16();
        magx = read16();magy = read16();magz = read16(); 
        GPS_fix = read8();
        satellites = read8();
        latitudeDegrees = read32();
        longitudeDegrees = read32();
        GPS_altitude = read16();
        GPS_speed = read16();
        for(i=0;i<4;i++){ mot[i] = read16();}
        if(toggleMotor == false){
        motSlider[0].setValue(mot[0]).show();
        motSlider[1].setValue(mot[1]).show();
        motSlider[2].setValue(mot[2]).show();
        motSlider[3].setValue(mot[3]).show();
        }
        //println("Mot[0] : " + mot[0]+", Mot[1] : " + mot[1]+", Mot[2] : " + mot[2]+", Mot[3] : " + mot[3]);
        println("com_time : "+ (millis()-com_time));
        com_time= millis();
        break;
   
     default:

     break;
  }
}

int time,time2,time3,time4,time5,time6;

void draw(){
int c;
float a,b,h;
background(100);
if(graph_on==1) {
time=millis();

if((time-time6)>20){
  time6=time;
  // Graph Left Side
  accROLL.addVal(ax);accPITCH.addVal(ay);accYAW.addVal(az);gyroROLL.addVal(gx);gyroPITCH.addVal(gy);gyroYAW.addVal(gz);
  magxData.addVal(magx);magyData.addVal(magy);magzData.addVal(magz);
  imuROLL.addVal(roll);imuPITCH.addVal(pitch);imuYAW.addVal(yaw);
  altData.addVal(alt);headData.addVal(heading);
  //debug1Data.addVal(debug1);debug2Data.addVal(debug2);debug3Data.addVal(debug3);debug4Data.addVal(debug4);

}

//if((time-time2)>40){
//  time2=time;
//  int[] requests = {MSP_STATUS, MSP_RAW_IMU};
//  sendRequestMSP(requestMSP(requests)); 
//}

//if((time-time3)>25){
//  time3=time;
//  int[] requests = {MSP_ATTITUDE};
//  sendRequestMSP(requestMSP(requests)); 
//}

//if((time-time4)>100){
//  time4=time;
//  int[] requests = {MSP_RC, MSP_ALTITUDE};
//  sendRequestMSP(requestMSP(requests)); 
//}

//if((time-time5)>200){
//  time5=time;
//    int[] requests = {MSP_MOTOR, MSP_RAW_GPS, MSP_ANALOG};
//  sendRequestMSP(requestMSP(requests)); 
//}
if((time-time5)>50){//50
  time5=time;
    int[] requests = {MSP_MISC};
  sendRequestMSP(requestMSP(requests)); 
}

if (toggleRead) {
  toggleRead = false;
  int[] requests = {MSP_PID};
  sendRequestMSP(requestMSP(requests));
}

if (toggleReset) {
  toggleReset = false;
  int[] requests = {MSP_RESET};
  sendRequestMSP(requestMSP(requests));
  error_count = 0;
}

if (toggleCalibAcc) {
  toggleCalibAcc = false;
  int[] requests = {MSP_ACC_CALIBRATION};
  sendRequestMSP(requestMSP(requests));
}

if (toggleCalibMag) {
  toggleCalibMag = false;
  int[] requests = {MSP_MAG_CALIBRATION};
  sendRequestMSP(requestMSP(requests));
}

if (toggleMotor & (time-time5)>20) {
  if(motSlider[4].getValue() != 2250){
    for(i=0;i<4;i++){
      motSlider[i].setValue(motSlider[4].getValue());
    }
  }
  payload = new ArrayList<Character>();
  for(i=0;i<5;i++){
     payload.add(char (int(motSlider[i].getValue()) % 256) ); payload.add(char (int(motSlider[i].getValue()) / 256)  );
     println("Motor["+i+"] : " + motSlider[i].getValue());
  }
  sendRequestMSP(requestMSP(MSP_SET_MOTOR,payload.toArray( new Character[payload.size()]) ));
}

  while (port.available()>0) {

    c = (port.read());
    //print((char)c);
   try{
    if (c_state == IDLE) {
      c_state = (c=='$') ? HEADER_START : IDLE;
    } else if (c_state == HEADER_START) {
      c_state = (c=='M') ? HEADER_M : IDLE;
    } else if (c_state == HEADER_M) {
      if (c == '>') {
        c_state = HEADER_ARROW;
      } else if (c == '!') {
        c_state = HEADER_ERR;
      } else {
        c_state = IDLE;
      }
    } else if (c_state == HEADER_ARROW || c_state == HEADER_ERR) {
      /* is this an error message? */
      err_rcvd = (c_state == HEADER_ERR);        /* now we are expecting the payload size */
      dataSize = (c&0xFF);
      /* reset index variables */
      p = 0;
      offset = 0;
      checksum = 0;
      checksum ^= (c&0xFF);
      /* the command is to follow */
      c_state = HEADER_SIZE;
    } else if (c_state == HEADER_SIZE) {
      cmd = (byte)(c&0xFF);
      checksum ^= (c&0xFF);
      c_state = HEADER_CMD;
    } else if (c_state == HEADER_CMD && offset < dataSize) {
        checksum ^= (c&0xFF);
        inBuf[offset++] = (byte)(c&0xFF);
    } else if (c_state == HEADER_CMD && offset >= dataSize) {
      /* compare calculated and transferred checksum */
      if ((checksum&0xFF) == (c&0xFF)) {
        if (err_rcvd) {
          //System.err.println("Copter did not understand request type "+c);
        } else {
          /* we got a valid response packet, evaluate it */
          evaluateCommand(cmd, (int)dataSize);
        }
      } else {
        error_count++;
        System.out.println("invalid checksum for command "+((int)(cmd&0xFF))+": "+(checksum&0xFF)+" expected, got "+(int)(c&0xFF));
        System.out.print("<"+(cmd&0xFF)+" "+(dataSize&0xFF)+"> {");
        for (i=0; i<dataSize; i++) {
          if (i!=0) { System.err.print(' '); }
          System.out.print((inBuf[i] & 0xFF));
        }
        System.out.println("} ["+c+"]");
        System.out.println(new String(inBuf, 0, dataSize));
      }
      c_state = IDLE;
    }
  }catch(Exception e){
    //println("Caught Exception");
  }
 }
}

  background(80);
  // version Background
  fill(40, 40, 40);
  strokeWeight(3);stroke(0);
  rectMode(CORNERS);
  rect(5,5+tabHeight,130,40+tabHeight);
  textFont(font15);
  // version
  fill(255, 255, 255);
  text("Can_Rotor.com",12,19+tabHeight);
  text("V",12,35+tabHeight);
  text(1, 27, 35+tabHeight);
  text(error,xGraph+410,yGraph-10);
  text(cycleTime,xGraph+290,yGraph-10);
  text(error_count,xGraph+600,yGraph-10);
  
  // ---------------------------------------------------------------------------------------------
  // GPS DATA
  // ---------------------------------------------------------------------------------------------
  // GPS Background
  fill(0, 0, 0);
  strokeWeight(3);stroke(0);
  rectMode(CORNERS);
  rect(xGPS-5,yGPS-15,xGPS+140,yGPS+95);
  // GPS DATA
  fill(255, 255, 255);
  text("GPS",xGPS,yGPS);
  text(GPS_altitude,xGPS+50,yGPS+15);
  text(latitudeDegrees,xGPS+50,yGPS+30);
  text(longitudeDegrees,xGPS+50,yGPS+45);
  text(GPS_speed,xGPS+50,yGPS+60);
  text(satellites,xGPS+50,yGPS+75);
  text(0,xGPS+65,yGPS+90);
  textFont(font12);
  text("alt   :",xGPS,yGPS+15);
  text("lat   :",xGPS,yGPS+30);
  text("lon   :",xGPS,yGPS+45);
  text("speed :",xGPS,yGPS+60);
  text("sat   :",xGPS,yGPS+75);
  text("dist home : ",xGPS,yGPS+90);
  // ---------------------------------------------------------------------------------------------

  text("Error:",xGraph+370,yGraph-10);
  text("Cycle Time:",xGraph+220,yGraph-10);
  text("Telemetry_Error:",xGraph+500,yGraph-10);
  
  text("Atmosphere", xGraph -5, yGraph -110);
  
  text("Power", xGraph -5, yGraph -65);
  textFont(font9);
  
  text("   Temprature:", xGraph -5, yGraph -100);
  text(Temperature + " deg", xGraph +70, yGraph -100);
  
  text("   Voltage:", xGraph -5, yGraph -55);
  text(VBAT/10.0 + " V", xGraph +50, yGraph -55);
  
  text("   Current:", xGraph -5, yGraph -45);
  text(0/10.0 + " A", xGraph +50, yGraph -45);
  
  text("   Total:", xGraph -5, yGraph -35);
  text(0 * 1.0 + " mAh", xGraph +50, yGraph -35);

  fill(255,255,255);
  
  textFont(font12);
  text("HEADFREE :", 10, 130);
  text("ARMED :", 10, 150);
  text("MODE :", 10, 170);
  text("ARMED TIME :", 10, 190);
  textFont(font9);
  if(ARMED == 1){
      text("ARMED", 60, 150);
  }else{
      text("DISARMED", 60, 150);
  }
  if(HEADFREE_MODE == 1){
    text("YES", 80, 130);
  }else{
    text("NO", 80, 130);
  }
  if(ANGLE_MODE == 1){
      text("ANGLE", 60, 170);
  }else if(HORIZON_MODE == 1){
      text("HORIZON", 60, 170);
  }else if(ACRO_MODE == 1){
      text("ACRO", 60, 170);
  }
  
  text(minutes+":"+seconds, 95, 190);


  
confPowerTrigger.setValue(255);

headingSlider.setValue(heading);

rollSlider.setValue(roll);
pitchSlider.setValue(pitch);
yawSlider.setValue(yaw);

// Graph
axSlider.setValue(ax);aySlider.setValue(ay);azSlider.setValue(az);gxSlider.setValue(gx);gySlider.setValue(gy);gzSlider.setValue(gz);
altSlider.setValue(alt);headSlider.setValue(heading);magxSlider.setValue(magx);magySlider.setValue(magy);magzSlider.setValue(magz);

  stroke(255);
  a=radians(roll);
  if (pitch<-90) {b=radians(-180 - pitch);}
  else if (pitch>90) {b=radians(+180 - pitch);}
  else{ b=radians(pitch);}
  h=radians(heading);

  // ---------------------------------------------------------------------------------------------
  // DRAW MULTICOPTER TYPE
  // ---------------------------------------------------------------------------------------------
  float size = 38.0;
  // object
  fill(255,255,255);
  pushMatrix();
  camera(xObj,yObj,300/tan(PI*60.0/360.0),xObj/2+30,yObj/2-40,0,0,1,0);
  translate(xObj,yObj);
  directionalLight(200,200,200, 0, 0, -1);
  rotateZ(h);rotateX(b);rotateY(a);
  stroke(150,255,150);
  strokeWeight(0);sphere(size/3);strokeWeight(3);
  line(0,0, 10,0,-size-5,10);line(0,-size-5,10,+size/4,-size/2,10); line(0,-size-5,10,-size/4,-size/2,10);
  stroke(255);
  textFont(font12);
  drawMotor(-size, +size, 1, 'R');
  drawMotor(+size, -size, 2, 'R');
  drawMotor(-size, -size, 3, 'L');
  drawMotor(+size, +size, 4, 'L');
  line(-size,-size, 0,0);line(+size,-size, 0,0);line(-size,+size, 0,0);line(+size,+size, 0,0);
  noLights();text("QUADRICOPTER X", -40,-50);camera();popMatrix();
  
  motSlider[0].setPosition(xMot+10,yMot+75).setHeight(60).setCaptionLabel("REAR_L") .show();
  motSlider[1].setPosition(xMot+90,yMot-15).setHeight(60).setCaptionLabel("FRONT_R").show();
  motSlider[2].setPosition(xMot+10,yMot-15).setHeight(60).setCaptionLabel("FRONT_L").show();
  motSlider[3].setPosition(xMot+90,yMot+75).setHeight(60).setCaptionLabel("REAR_R") .show();
   
  // ------------------------------------------------------------------------
  // Draw background control boxes
  // ------------------------------------------------------------------------
  fill(0, 0, 0);
  strokeWeight(3);stroke(0);
  rectMode(CORNERS);
  // motor background
  rect(xMot-5,yMot-20, xMot+145, yMot+150);
    // rc background
  rect(xRC-5,yRC-5, xRC+145, yRC+120);
  // param background
  rect(xParam,yParam, xParam+555, yParam+280);
  
  fill(255);
  textFont(font15);    
  text("P",xParam+45,yParam+15);text("I",xParam+90,yParam+15);text("D",xParam+130,yParam+15);
  textFont(font12);
  text("Angle_PID",xParam+170,yParam+49);
  text("ROLL",xParam+3,yParam+32);
  text("PITCH",xParam+3,yParam+32+1*17);
  text("YAW",xParam+3,yParam+32+2*17);
  
  text("Dual_PID",xParam+170,yParam+133);
  text("R_OUT",xParam+3,yParam+90);
  text("P_OUT",xParam+3,yParam+90+1*17);
  text("Y_OUT",xParam+3,yParam+90+2*17);
  
  text("R_IN",xParam+3,yParam+140);
  text("P_IN",xParam+3,yParam+140+1*17);
  text("Y_IN",xParam+3,yParam+140+2*17);
  
  text("Rate_PID",xParam+170,yParam+217);
  text("ROLL",xParam+3,yParam+200);
  text("PITCH",xParam+3,yParam+200+1*17);
  text("YAW",xParam+3,yParam+200+2*17);

  // ---------------------------------------------------------------------------------------------
  // Compass Section
  // ---------------------------------------------------------------------------------------------
  pushMatrix();
  translate(xCompass,yCompass);
  // Compass Background
  fill(0, 0, 0);
  strokeWeight(3);stroke(0);
  rectMode(CORNERS);
  size=29;
  rect(-size*2.5,-size*2.5,size*2.5,size*2.5);
  // GPS quadrant
  strokeWeight(1.5);
  fill(160);stroke(160);
  ellipse(0,  0,   4*size+7, 4*size+7);
  //  // GPS rotating pointer
  //rotate(360*PI/180);
  //strokeWeight(4);stroke(255,255,100);line(0,0, 0,-2.4*size);line(0,-2.4*size, -5 ,-2.4*size+10); line(0,-2.4*size, +5 ,-2.4*size+10);  
  //rotate(-360*PI/180);
  // compass quadrant
  strokeWeight(1.5);fill(0);stroke(0);
  ellipse(0,  0,   2.6*size+7, 2.6*size+7);
  // Compass rotating pointer
  stroke(255);
  rotate(heading*PI/180);
  line(0,size*0.2, 0,-size*1.3); line(0,-size*1.3, -5 ,-size*1.3+10); line(0,-size*1.3, +5 ,-size*1.3+10);
  popMatrix();
  
   // angles 
  for (i=0;i<=12;i++) {
    angCalc=i*PI/6;
    if (i%3!=0) {
      stroke(75);
      line(xCompass+cos(angCalc)*size*2,yCompass+sin(angCalc)*size*2,xCompass+cos(angCalc)*size*1.6,yCompass+sin(angCalc)*size*1.6);
    } else {
      stroke(255);
      line(xCompass+cos(angCalc)*size*2.2,yCompass+sin(angCalc)*size*2.2,xCompass+cos(angCalc)*size*1.9,yCompass+sin(angCalc)*size*1.9);
    }
  }
  
  textFont(font15);
  text("N", xCompass-5, yCompass-22-size*0.9);text("S", xCompass-5, yCompass+32+size*0.9);
  text("W", xCompass-33-size*0.9, yCompass+6);text("E", xCompass+21+size*0.9, yCompass+6);
  
  // head indicator
  textFont(font12);
  noStroke();
  fill(80,80,80,130);
  rect(xCompass-22,yCompass-8,xCompass+22,yCompass+9);
  fill(255,255,127);
  text(heading + "°",xCompass-11-(heading>=10.0 ? (heading>=100.0 ? 6 : 3) : 0),yCompass+6);
  
  // GPS direction indicator
  fill(255,255,0);
  text(GPS_directionToHome + "°",xCompass-6-size*2.1,yCompass+7+size*2);
  // GPS fix
  if (GPS_fix==0) {
     fill(127,0,0);
  } else {
     fill(0,255,0);
  }
  //println("GPS_fix : "+ GPS_fix);
    //ellipse(xCompass+3+size*2.1,yCompass+3+size*2,12,12);
  rect(xCompass-28+size*2.1,yCompass+1+size*2,xCompass+9+size*2.1,yCompass+13+size*2);
  textFont(font9);
  if (GPS_fix==0) {
    fill(255,255,0);
  } else {
    fill(0,50,0);
  }
  text("GPS_fix",xCompass-27+size*2.1,yCompass+10+size*2);
  
  // ---------------------------------------------------------------------------------------------
  // Fly Level Control Instruments
  // ---------------------------------------------------------------------------------------------
  // info angles
  fill(255,255,127);
  textFont(font12);
  text((int)pitch + "°", xLevelObj+38, yLevelObj+78); //pitch
  text((int)roll + "°", xLevelObj-62, yLevelObj+78); //roll

  pushMatrix();
  translate(xLevelObj-34,yLevelObj+112);
  fill(50,50,50);
  noStroke();
  ellipse(0,0,66,66);
  rotate(a);
  fill(255,255,127);
  textFont(font12);
  text("ROLL", -13, 15);
  strokeWeight(1.5);
  stroke(127,127,127);
  line(-30,1,30,1);
  stroke(255,255,255);
  line(-30,0,+30,0);line(0,0,0,-10);
  popMatrix();
  
  pushMatrix();
  translate(xLevelObj+34,yLevelObj+112);
  fill(50,50,50);
  noStroke();
  ellipse(0,0,66,66);
  rotate(b);
  fill(255,255,127);
  textFont(font12);
  text("PITCH", -18, 15);
  strokeWeight(1.5);
  stroke(127,127,127);
  line(-30,1,30,1);
  stroke(255,255,255);
  line(-30,0,30,0);line(30,0,30-size/6 ,size/6);line(+30,0,30-size/6 ,-size/6);  
  popMatrix();
  
  // ---------------------------------------------------------------------------------------------
  // Magnetron Combi Fly Level Control
  // ---------------------------------------------------------------------------------------------
  horizonInstrSize=68;
  angyLevelControl=((pitch<-horizonInstrSize) ? -horizonInstrSize : (pitch>horizonInstrSize) ? horizonInstrSize : pitch);
  pushMatrix();
  translate(xLevelObj,yLevelObj);
  noStroke();
  // instrument background
  fill(50,50,50);
  ellipse(0,0,150,150);
  // full instrument
  rotate(a);
  rectMode(CORNER);
  // outer border
  strokeWeight(1);
  stroke(90,90,90);
  //border ext
  arc(0,0,140,140,0,TWO_PI);
  stroke(190,190,190);
  //border int
  arc(0,0,138,138,0,TWO_PI);
  // inner quadrant
  strokeWeight(1);
  stroke(255,255,255);
  fill(124,73,31);
  //earth
  float angle = acos(angyLevelControl/horizonInstrSize);
  arc(0,0,136,136,0,TWO_PI);
  fill(38,139,224); 
  //sky 
  arc(0,0,136,136,HALF_PI-angle+PI,HALF_PI+angle+PI);
  float x = sin(angle)*horizonInstrSize;
  if (pitch>0) 
    fill(124,73,31);
  noStroke();   
  triangle(0,0,x,-angyLevelControl,-x,-angyLevelControl);
  // inner lines
  strokeWeight(1);
  for(i=0;i<8;i++) {
    j=i*15;
    if (pitch<=(35-j) && pitch>=(-65-j)) {
      stroke(255,255,255); line(-30,-15-j-pitch,30,-15-j-pitch); // up line
      fill(255,255,255);
      textFont(font9);
      text("+" + (i+1) + "0", 34, -12-j-pitch); //  up value
      text("+" + (i+1) + "0", -48, -12-j-pitch); //  up value
    }
    if (pitch<=(42-j) && pitch>=(-58-j)) {
      stroke(167,167,167); line(-20,-7-j-pitch,20,-7-j-pitch); // up semi-line
    }
    if (pitch<=(65+j) && pitch>=(-35+j)) {
      stroke(255,255,255); line(-30,15+j-pitch,30,15+j-pitch); // down line
      fill(255,255,255);
      textFont(font9);
      text("-" + (i+1) + "0", 34, 17+j-pitch); //  down value
      text("-" + (i+1) + "0", -48, 17+j-pitch); //  down value
    }
    if (pitch<=(58+j) && pitch>=(-42+j)) {
      stroke(127,127,127); line(-20,7+j-pitch,20,7+j-pitch); // down semi-line
    }
  }
  strokeWeight(2);
  stroke(255,255,255);
  if (pitch<=50 && pitch>=-50) {
    line(-40,-pitch,40,-pitch); //center line
    fill(255,255,255);
    textFont(font9);
    text("0", 34, 4-pitch); // center
    text("0", -39, 4-pitch); // center
  }

  // lateral arrows
  strokeWeight(1);
  // down fixed triangle
  stroke(60,60,60);
  fill(180,180,180,255);

  triangle(-horizonInstrSize,-8,-horizonInstrSize,8,-55,0);
  triangle(horizonInstrSize,-8,horizonInstrSize,8,55,0);

  // center
  strokeWeight(1);
  stroke(255,0,0);
  line(-20,0,-5,0); line(-5,0,-5,5);
  line(5,0,20,0); line(5,0,5,5);
  line(0,-5,0,5);
  popMatrix();
  
  // ---------------------------------------------------------------------------------------------
  // GRAPH
  // ---------------------------------------------------------------------------------------------
  strokeWeight(1);
  fill(255, 255, 255);
  g_graph.drawGraphBox();
  
  strokeWeight(1.5);
  stroke(255, 0, 0); if (axGraph) g_graph.drawLine(accROLL, -1000, +1000);
  stroke(0, 255, 0); if (ayGraph) g_graph.drawLine(accPITCH, -1000, +1000);
  stroke(0, 0, 255); if (azGraph) g_graph.drawLine(accYAW, -1000, +1000);
  
  float altMin = (altData.getMinVal() + altData.getRange() / 2) - 100;
  float altMax = (altData.getMaxVal() + altData.getRange() / 2) + 100;

  stroke(200, 200, 0);  if (gxGraph)   g_graph.drawLine(gyroROLL, -300, +300);
  stroke(0, 255, 255);  if (gyGraph)   g_graph.drawLine(gyroPITCH, -300, +300);
  stroke(255, 0, 255);  if (gzGraph)   g_graph.drawLine(gyroYAW, -300, +300);
  stroke(125, 125, 125);if (altGraph)  g_graph.drawLine(altData, altMin, altMax);
  stroke(225, 225, 125);if (headGraph) g_graph.drawLine(headData, -370, +370); 
  stroke(50, 100, 150); if (magxGraph) g_graph.drawLine(magxData, -1000, +1000);
  stroke(100, 50, 150); if (magyGraph) g_graph.drawLine(magyData, -1000, +1000);
  stroke(150, 100, 50); if (magzGraph) g_graph.drawLine(magzData, -1000, +1000);

  //stroke(200, 50, 0); if (debug1Graph)  g_graph.drawLine(debug1Data, -5000, +5000);
  //stroke(0, 200, 50); if (debug2Graph)  g_graph.drawLine(debug2Data, -5000, +5000);
  //stroke(50, 0, 200); if (debug3Graph)  g_graph.drawLine(debug3Data, -5000, +5000);
  //stroke(0, 0, 0);    if (debug4Graph)  g_graph.drawLine(debug4Data, -5000, +5000);
  
}

void drawMotor(float x1, float y1, int mot_num, char dir) {   //Code by Danal
  float size = 30.0;
  pushStyle();
  float d = 0;
  if (dir == 'L') {d = +5; fill(254, 221, 44);} 
  if (dir == 'R') {d = -5; fill(256, 152, 12);}
  ellipse(x1, y1, size, size);
  //textFont(font15);
  textAlign(CENTER);
  fill(0,0,0);
  text(mot_num,x1,y1+5,3);
  float y2 = y1-(size/2);
  stroke(255,0,0);
  line(x1, y2, 3, x1+d, y2-5, 3);
  line(x1, y2, 3, x1+d, y2+5, 3);
  line(x1, y2, 3, x1+d*2, y2, 3); 
  popStyle();
}

void ACC_ROLL(boolean theFlag) {axGraph = theFlag;}
void ACC_PITCH(boolean theFlag) {ayGraph = theFlag;}
void ACC_Z(boolean theFlag) {azGraph = theFlag;}
void GYRO_ROLL(boolean theFlag) {gxGraph = theFlag;}
void GYRO_PITCH(boolean theFlag) {gyGraph = theFlag;}
void GYRO_YAW(boolean theFlag) {gzGraph = theFlag;}
void BARO(boolean theFlag) {altGraph = theFlag;}
void HEAD(boolean theFlag) {headGraph = theFlag;}
void MAGX(boolean theFlag) {magxGraph = theFlag;}
void MAGY(boolean theFlag) {magyGraph = theFlag;}
void MAGZ(boolean theFlag) {magzGraph = theFlag;}
void DEBUG1(boolean theFlag) {debug1Graph = theFlag;}
void DEBUG2(boolean theFlag) {debug2Graph = theFlag;}
void DEBUG3(boolean theFlag) {debug3Graph = theFlag;}
void DEBUG4(boolean theFlag) {debug4Graph = theFlag;}

public void READ() {
//  toggleLive = false ; 
  toggleRead = true;
}

public void RESET() {
  toggleReset = true;
}

public void CALIB_ACC() {toggleCalibAcc = true;}

public void CALIB_MAG() {toggleCalibMag = true;}

public void MANUAL_MOTOR(){
  toggleMotor = !toggleMotor;
  if(toggleMotor == true){
    for(i=0;i<5;i++){
      motSlider[i].setValue(2250);
    }
    buttonMANUAL_MOTOR.setColorBackground(green_);
    motSlider[4].show();
  }else{
    for(i=0;i<5;i++){
      motSlider[i].setValue(2250);
    }
    buttonMANUAL_MOTOR.setColorBackground(red_);
    motSlider[4].hide();
  }
}

public void WRITE() {
  int tmp_P[] = new int[PIDITEMS], tmp_I[] = new int[PIDITEMS], tmp_D[] = new int[PIDITEMS],
  tmp_P_dual_outer[] = new int[PIDITEMS], tmp_I_dual_outer[] = new int[PIDITEMS], tmp_P_dual_inner[] = new int[PIDITEMS], tmp_I_dual_inner[] = new int[PIDITEMS], tmp_D_dual_inner[] = new int[PIDITEMS],
  tmp_P_rate[] = new int[PIDITEMS], tmp_I_rate[] = new int[PIDITEMS], tmp_D_rate[] = new int[PIDITEMS];
  toggleWrite = true;
  // MSP_SET_PID
  payload = new ArrayList<Character>();
  //for(i=0;i<PIDITEMS;i++) {
  //   tmp_P[i] = (int)(confP[i].getValue()*10);
  //   tmp_I[i] = (int)(confI[i].getValue()*10);
  //   tmp_D[i] = (int)(confD[i].getValue()*10);
  // }
     tmp_P[ROLL]   = (int)(confP[ROLL].getValue()*10);
     tmp_P[PITCH]  = (int)(confP[ROLL].getValue()*10);
     tmp_P[YAW]    = (int)(confP[YAW].getValue()*10);
     tmp_I[ROLL]   = (int)(confI[ROLL].getValue()*10);
     tmp_I[PITCH]  = (int)(confI[ROLL].getValue()*10);
     tmp_I[YAW]    = (int)(confI[YAW].getValue()*10);
     tmp_D[ROLL]   = (int)(confD[ROLL].getValue()*100);
     tmp_D[PITCH]  = (int)(confD[ROLL].getValue()*100);
     tmp_D[YAW]    = (int)(confD[YAW].getValue()*100);
     
     tmp_P_dual_outer[ROLL] = (int)(confP_dual_outer[ROLL].getValue()*10);
     tmp_P_dual_outer[PITCH] = (int)(confP_dual_outer[ROLL].getValue()*10);
     tmp_P_dual_outer[YAW] = (int)(confP_dual_outer[YAW].getValue()*10);
     tmp_I_dual_outer[ROLL] = (int)(confI_dual_outer[ROLL].getValue()*10);
     tmp_I_dual_outer[PITCH] = (int)(confI_dual_outer[ROLL].getValue()*10);
     tmp_I_dual_outer[YAW] = (int)(confI_dual_outer[YAW].getValue()*10);
     
     tmp_P_dual_inner[ROLL] = (int)(confP_dual_inner[ROLL].getValue()*10);
     tmp_P_dual_inner[PITCH] = (int)(confP_dual_inner[ROLL].getValue()*10);
     tmp_P_dual_inner[YAW] = (int)(confP_dual_inner[YAW].getValue()*10);
     tmp_I_dual_inner[ROLL] = (int)(confI_dual_inner[ROLL].getValue()*10);
     tmp_I_dual_inner[PITCH] = (int)(confI_dual_inner[ROLL].getValue()*10);
     tmp_I_dual_inner[YAW] = (int)(confI_dual_inner[YAW].getValue()*10);
     tmp_D_dual_inner[ROLL] = (int)(confD_dual_inner[ROLL].getValue()*100);
     tmp_D_dual_inner[PITCH] = (int)(confD_dual_inner[ROLL].getValue()*100);
     tmp_D_dual_inner[YAW] = (int)(confD_dual_inner[YAW].getValue()*100);
     
     tmp_P_rate[ROLL] = (int)(confP_rate[ROLL].getValue()*10);
     tmp_P_rate[PITCH] = (int)(confP_rate[ROLL].getValue()*10);
     tmp_P_rate[YAW] = (int)(confP_rate[YAW].getValue()*10);
     tmp_I_rate[ROLL] = (int)(confI_rate[ROLL].getValue()*10);
     tmp_I_rate[PITCH] = (int)(confI_rate[ROLL].getValue()*10);
     tmp_I_rate[YAW] = (int)(confI_rate[YAW].getValue()*10);
     tmp_D_rate[ROLL] = (int)(confD_rate[ROLL].getValue()*100);
     tmp_D_rate[PITCH] = (int)(confD_rate[ROLL].getValue()*100);
     tmp_D_rate[YAW] = (int)(confD_rate[YAW].getValue()*100);
     
  for(i=0;i<PIDITEMS;i++) {
    int q;
    q = tmp_P[i]; payload.add(char (q % 256) ); payload.add(char (q / 256) );
    q = tmp_I[i]; payload.add(char (q % 256) ); payload.add(char (q / 256) );
    q = tmp_D[i]; payload.add(char (q % 256) ); payload.add(char (q / 256) );
    q = tmp_P_dual_outer[i]; payload.add(char (q % 256) ); payload.add(char (q / 256) );
    q = tmp_I_dual_outer[i]; payload.add(char (q % 256) ); payload.add(char (q / 256) );
    q = tmp_P_dual_inner[i]; payload.add(char (q % 256) ); payload.add(char (q / 256) );
    q = tmp_I_dual_inner[i]; payload.add(char (q % 256) ); payload.add(char (q / 256) );
    q = tmp_D_dual_inner[i]; payload.add(char (q % 256) ); payload.add(char (q / 256) );
    q = tmp_P_rate[i]; payload.add(char (q % 256) ); payload.add(char (q / 256) );
    q = tmp_I_rate[i]; payload.add(char (q % 256) ); payload.add(char (q / 256) );
    q = tmp_D_rate[i]; payload.add(char (q % 256) ); payload.add(char (q / 256) );
  }
  sendRequestMSP(requestMSP(MSP_SET_PID,payload.toArray(new Character[payload.size()])));
    toggleWrite = false;
}

public void SAVE() {
  toggleSave = true;
  sendRequestMSP(requestMSP(TELEMERY_PID_SAVE));
}

public void bSTART() {
  //if(graphEnabled == false) {return;}
  graph_on=1;
  toggleRead=true;
  toggleMotor = false;
  buttonMANUAL_MOTOR.setColorBackground(red_);
  motSlider[4].hide();
  //g_serial.clear();
}

public void bSTOP() {
  graph_on=0;
  toggleMotor = false;
  buttonMANUAL_MOTOR.setColorBackground(red_);
  motSlider[4].hide();
  for(i=0;i<4;i++){
    motSlider[i].setValue(2250);
  }
  for(i=0;i<PIDITEMS;i++) {
  confP[i].setColorBackground(red_);confI[i].setColorBackground(red_);confD[i].setColorBackground(red_);
  confP_dual_outer[i].setColorBackground(red_);confI_dual_outer[i].setColorBackground(red_);
  confP_dual_inner[i].setColorBackground(red_);confI_dual_inner[i].setColorBackground(red_);confD_dual_inner[i].setColorBackground(red_);
  confP_rate[i].setColorBackground(red_);confI_rate[i].setColorBackground(red_);confD_rate[i].setColorBackground(red_);
  }
}

//update the model with the value view 
public void updateModel(){
        //updateModelMSP_SET_RC_TUNING();
        updateModelMSP_SET_PID();
        //updateModelMSP_SET_MISC();
}

//public void updateModelMSP_SET_RC_TUNING(){
//        MWI.setProperty("rc.rate",String.valueOf(confRC_RATE.value()));
//        MWI.setProperty("rc.expo",String.valueOf(confRC_EXPO.value()));
//        MWI.setProperty("rc.rollpitch.rate",String.valueOf(rollPitchRate.value()));
//        MWI.setProperty("rc.yaw.rate",String.valueOf(yawRate.value()));
//        MWI.setProperty("rc.throttle.rate",String.valueOf(dynamic_THR_PID.value()));
//        MWI.setProperty("rc.throttle.mid",String.valueOf(throttle_MID.value()));
//        MWI.setProperty("rc.throttle.expo",String.valueOf(throttle_EXPO.value()));
//}

public void updateModelMSP_SET_PID(){
for( i=0;i<PIDITEMS;i++) {
    MWI.setProperty("pid."+i+".p",String.valueOf(confP[i].getValue()));
    MWI.setProperty("pid."+i+".i",String.valueOf(confI[i].getValue()));
    MWI.setProperty("pid."+i+".d",String.valueOf(confD[i].getValue()));
  }
}

//public void updateModelMSP_SET_MISC(){
//   MWI.setProperty("power.trigger",String.valueOf(round(confPowerTrigger.value())));
//}


// use the model to update the value of the view
public void updateView(){
  // MSP_SET_RC_TUNING
  //confRC_RATE.setValue(Float.valueOf(MWI.conf.getProperty("rc.rate")));
  //confRC_EXPO.setValue(Float.valueOf(MWI.conf.getProperty("rc.expo")));
  //rollPitchRate.setValue(Float.valueOf(MWI.conf.getProperty("rc.rollpitch.rate")));
  //yawRate.setValue(Float.valueOf(MWI.conf.getProperty("rc.yaw.rate")));
  //dynamic_THR_PID.setValue(Float.valueOf(MWI.conf.getProperty("rc.throttle.rate")));
  //throttle_MID.setValue(Float.valueOf(MWI.conf.getProperty("rc.throttle.mid")));
  //throttle_EXPO.setValue(Float.valueOf(MWI.conf.getProperty("rc.throttle.expo")));
  
  // MSP_SET_PID
  for( i=0;i<PIDITEMS;i++) {
     confP[i].setValue(Float.valueOf(MWI.conf.getProperty("pid."+i+".p"))) ;
     confI[i].setValue(Float.valueOf(MWI.conf.getProperty("pid."+i+".i"))) ;
     confD[i].setValue(Float.valueOf(MWI.conf.getProperty("pid."+i+".d"))) ;
  }

  // MSP_SET_MISC
  //confPowerTrigger.setValue(Float.valueOf(MWI.conf.getProperty("power.trigger")));
}
      

//  our model
static class MWI{
private static Properties conf = new Properties();

  public static void setProperty(String key ,String value ){
    conf.setProperty( key,value );
  }

  public static String getProperty(String key ){
    return conf.getProperty( key,"0");
  }

  public static void clear(){
    conf= null; // help gc
    conf = new Properties();
  }
}

//********************************************************
//********************************************************
//********************************************************

class cDataArray {
  float[] m_data;
  int m_maxSize, m_startIndex = 0, m_endIndex = 0, m_curSize;
  
  cDataArray(int maxSize){
    m_maxSize = maxSize;
    m_data = new float[maxSize];
  }
  void addVal(float val) {
    m_data[m_endIndex] = val;
    m_endIndex = (m_endIndex+1)%m_maxSize;
    if (m_curSize == m_maxSize) {
      m_startIndex = (m_startIndex+1)%m_maxSize;
    } else {
      m_curSize++;
    }
  }
  float getVal(int index) {return m_data[(m_startIndex+index)%m_maxSize];}
  int getCurSize(){return m_curSize;}
  int getMaxSize() {return m_maxSize;}
  float getMaxVal() {
    float res = 0.0;
    for( i=0; i<m_curSize-1; i++) if ((m_data[i] > res) || (i==0)) res = m_data[i];
    return res;
  }
  float getMinVal() {
    float res = 0.0;
    for( i=0; i<m_curSize-1; i++) if ((m_data[i] < res) || (i==0)) res = m_data[i];
    return res;
  }
  float getRange() {return getMaxVal() - getMinVal();}
}

// This class takes the data and helps graph it
class cGraph {
  float m_gWidth, m_gHeight, m_gLeft, m_gBottom, m_gRight, m_gTop;
  
  cGraph(float x, float y, float w, float h) {
    m_gWidth     = w; m_gHeight    = h;
    m_gLeft      = x; m_gBottom    = y;
    m_gRight     = x + w;
    m_gTop       = y + h;
  }
  
  void drawGraphBox() {
    stroke(0, 0, 0);
    rectMode(CORNERS);
    rect(m_gLeft, m_gBottom, m_gRight, m_gTop);
  }
  
  void drawLine(cDataArray data, float minRange, float maxRange) {
    float graphMultX = m_gWidth/data.getMaxSize();
    float graphMultY = m_gHeight/(maxRange-minRange);
    
    for( i=0; i<data.getCurSize()-1; ++i) {
      float x0 = i*graphMultX+m_gLeft;
      float y0 = m_gTop-(((data.getVal(i)-(maxRange+minRange)/2)*scaleSlider.getValue()+(maxRange-minRange)/2)*graphMultY);
      float x1 = (i+1)*graphMultX+m_gLeft;
      float y1 = m_gTop-(((data.getVal(i+1)-(maxRange+minRange)/2 )*scaleSlider.getValue()+(maxRange-minRange)/2)*graphMultY);
      line(x0, y0, x1, y1);
    }
  }
}
