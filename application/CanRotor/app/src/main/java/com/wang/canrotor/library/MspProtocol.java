package com.wang.canrotor.library;

import android.util.Log;

import java.util.LinkedList;
import java.util.List;

public class MspProtocol {

    public static final int
            TELEMERY_PID_SAVE  = 56,
            MSP_IDENT = 100,
            MSP_STATUS = 101,
            MSP_RAW_IMU = 102,
            MSP_SERVO = 103,
            MSP_MOTOR = 104,
            MSP_RC = 105,
            MSP_RAW_GPS = 106,
            MSP_COMP_GPS = 107,
            MSP_ATTITUDE = 108,
            MSP_ALTITUDE = 109,
            MSP_ANALOG = 110,
            MSP_RC_TUNING = 111,
            MSP_PID = 112,
            MSP_BOX = 113,
            MSP_MISC = 114,
            MSP_MOTOR_PINS = 115,
            MSP_BOXNAMES = 116,
            MSP_PIDNAMES = 117,
            MSP_WP = 118,
            MSP_SERVO_CONF = 120,
            MSP_RESET      =121,
            MSP_MOBILE = 122,

    MSP_RC_RAW = 150,   //out message         radio channel Flexbot
            MSP_ARM = 151,
            MSP_DISARM = 152,
            MSP_TRIM_UP = 153,
            MSP_TRIM_DOWN = 154,
            MSP_TRIM_LEFT = 155,
            MSP_TRIM_RIGHT = 156,

    MSP_TRIM_UP_FAST = 157,
            MSP_TRIM_DOWN_FAST = 158,
            MSP_TRIM_LEFT_FAST = 159,
            MSP_TRIM_RIGHT_FAST = 160,

    MSP_READ_TEST_PARAM = 189,
            MSP_SET_TEST_PARAM = 190,
            MSP_HEX_NANO = 199,

    MSP_SET_RAW_RC = 200,
            MSP_SET_RAW_GPS = 201,
            MSP_SET_PID = 202,
            MSP_SET_BOX = 203,
            MSP_SET_RC_TUNING = 204,
            MSP_ACC_CALIBRATION = 205,
            MSP_MAG_CALIBRATION = 206,
            MSP_SET_MISC = 207,
            MSP_RESET_CONF = 208,
            MSP_SELECT_SETTING = 210,
            MSP_SET_HEAD = 211, // Not used
            MSP_SET_SERVO_CONF = 212,
            MSP_SET_MOTOR = 214,

    MSP_BIND = 241,

    MSP_EEPROM_WRITE = 250,

    MSP_DEBUGMSG = 253,
            MSP_DEBUG = 254;

    public static final int
            IDLE = 0,
            HEADER_START = 1,
            HEADER_M = 2,
            HEADER_ARROW = 3,
            HEADER_SIZE = 4,
            HEADER_CMD = 5,
            HEADER_ERR = 6;

    int c_state = IDLE;

    final static int INBUF_SIZE = 256;
    byte[] inBuf = new byte[256];
    byte checksum = 0;
    byte indRX;
    byte cmdMSP;
    int offset = 0, dataSize = 0;

    private static final String MSP_HEADER = "$M<";

    public List<Character> payload;

    public int PIDITEMS=3;

    public int error_count;
    public int[] arry = {0};


    public int armedTime, hours, minutes, seconds, mode, ARMED, HEADFREE_MODE, ANGLE_MODE, HORIZON_MODE, ACRO_MODE, cycleTime, error;
    public float VBAT, Temperature, alt;
    public float byteP[] = new float[PIDITEMS], byteI[] = new float[PIDITEMS], byteD[] = new float[PIDITEMS],
            byteP_outer[] = new float[PIDITEMS], byteI_outer[] = new float[PIDITEMS], byteP_inner[] = new float[PIDITEMS], byteI_inner[] = new float[PIDITEMS], byteD_inner[] = new float[PIDITEMS],
            byteP_rate[] = new float[PIDITEMS], byteI_rate[] = new float[PIDITEMS], byteD_rate[] = new float[PIDITEMS];

    public float ROLL, PITCH, YAW;

    public int RCchan[] = new int[16],
               mot[] = new int[4];
    public int RCThro = 3, RCRoll = 0, RCPitch = 1, RCYaw = 2, RCAUX1 = 4, RCAUX2 = 5, RCAUX3 = 6, RCAUX4 = 7;

    public MspProtocol() {}

    public List<Byte> requestMSP(int msp) {
        return requestMSP(msp, null);
    }

    //send multiple msp without payload
    public List<Byte> requestMSP(int[] msps) {
        List<Byte> s = new LinkedList<Byte>();
        for (int m : msps) {
            s.addAll(requestMSP(m, null));
        }
        return s;
    }

    public List<Byte> requestMSP(int msp, Character[] payload) {
        if (msp < 0) {
            return null;
        }
        List<Byte> bf = new LinkedList<Byte>();
        for (byte c : MSP_HEADER.getBytes()) {
            bf.add(c);
        }
        byte checksum = 0;
        byte pl_size = (byte) (payload != null ? payload.length : 0);
        bf.add(pl_size);
        checksum ^= (pl_size & 0xFF);

        bf.add((byte) (msp & 0xFF));
        checksum ^= (msp & 0xFF);

        if (payload != null) {
            for (char c : payload) {
                bf.add((byte) (c & 0xFF));
                checksum ^= (c & 0xFF);
            }
        }
        bf.add(checksum);
        return (bf);
    }

    public byte[] sendRequestMSP(List<Byte> msp) {
        byte[] arr = new byte[msp.size()];
        int i = 0;
        for (byte b : msp) {
            arr[i++] = b;
        }
        return arr;
    }

    public void SerialCom(byte[] i) {

        for (byte c : i) {
            if (c_state == IDLE) {
                c_state = (c == '$') ? HEADER_START : IDLE;
                //Log.d("msp", "c_state : " + c_state);
            } else if (c_state == HEADER_START) {
                c_state = (c == 'M') ? HEADER_M : IDLE;
                //Log.d("msp", "c_state : " + c_state);
            } else if (c_state == HEADER_M) {
                c_state = (c == '>') ? HEADER_ARROW : IDLE;
                //Log.d("msp", "c_state : " + c_state);
            } else if (c_state == HEADER_ARROW) {
                if (c > INBUF_SIZE) {  // now we are expecting the payload size
                    c_state = IDLE;
                    //Log.d("msp", "c_state : " + c_state);
                    continue;
                }
                dataSize = c;
                offset = 0;
                indRX = 0;
                checksum = 0;
                checksum ^= c;
                c_state = HEADER_SIZE;
                //Log.d("msp", "c_state : " + c_state);
            } else if (c_state == HEADER_SIZE) {
                cmdMSP = c;
                checksum ^= c;
                c_state = HEADER_CMD;
                //Log.d("msp", "cmdMSP : " + cmdMSP);
                //Log.d("msp", "c_state : " + c_state);
            } else if (c_state == HEADER_CMD && offset < dataSize) {
                checksum ^= c;
                inBuf[offset++] = c;
                //Log.d("msp", "offset : " + offset);
                //Log.d("msp", "dataSize : " + dataSize);
                //Log.d("msp", "inBuf : " + inBuf[offset-1]);
                //Log.d("msp", "checksum : " + checksum);
            } else if (c_state == HEADER_CMD && offset >= dataSize) {
                //Log.d("msp", "-----------------------------");
                //Log.d("msp", "checksum : " + checksum);
                //Log.d("msp", "c : " + c);
                if (checksum == c) {
                    //Log.d("msp1", "checksum OK!");
                    evaluateCommand();
                }else{
                    error_count++;
                    //Log.d("error", "Test"+error_count);
                }
                c_state = IDLE;
            }
        }
    }

    private void evaluateCommand() {
        switch (cmdMSP) {
            case MSP_MOBILE:
                for(int i=0; i<6;i++){
                    RCchan[i] = read16();
                }
                //armedTime = read32();
                //minutes = (armedTime/60000000);
                //seconds = (armedTime-minutes*60000000)/1000000;
                cycleTime = read32();
                error = read16();
                mode  = read16();
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
                    //buttonCALIBRATE_ACC.setColorBackground(green_);
                }else{
                    //buttonCALIBRATE_ACC.setColorBackground(red_);
                }
                if((mode&(1<<6))>0){
                    //buttonCALIBRATE_MAG.setColorBackground(green_);
                }else{
                    //buttonCALIBRATE_MAG.setColorBackground(red_);
                }

                alt = read16();
                VBAT = read16();
                Temperature = (float)read16()/10;

                ROLL = (float)read16()/10;
                PITCH = (float)read16()/10;
                YAW = (float)read16()/10;

                for(int i=0;i<4;i++){ mot[i] = read16();}
                break;

            case MSP_PID:
                //Log.d("msp_pid", "OK");
                //Log.d("msp_pid", "OK"+byteP[0]);
                for(int i=0;i<PIDITEMS;i++) {
                    byteP[i] = (float)read16()/10;byteI[i] = (float)read16()/10;byteD[i] = (float)read16()/10;
                    byteP_outer[i] = (float)read16()/10;byteI_outer[i] = (float)read16()/10;byteP_inner[i] = (float)read16()/10;byteI_inner[i] = (float)read16()/10;byteD_inner[i] = (float)read16()/10;
                    byteP_rate[i] = (float)read16()/10;byteI_rate[i] = (float)read16()/10;byteD_rate[i] = (float)read16()/10;
                }
                break;
        }
    }

    int read32() {
        return (inBuf[indRX++] & 0xff) + ((inBuf[indRX++] & 0xff) << 8) + ((inBuf[indRX++] & 0xff) << 16) + ((inBuf[indRX++] & 0xff) << 24);
    }

    int read16() {
        return (inBuf[indRX++] & 0xff) + ((inBuf[indRX++]) << 8);
    }

    int read8() {
        return (inBuf[indRX++] & 0xff);
    }
}