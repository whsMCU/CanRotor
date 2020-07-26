package com.wang.canrotor.activity;

import android.app.AlertDialog;
import android.app.Dialog;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.support.v4.app.FragmentManager;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import com.wang.canrotor.R;
import com.wang.canrotor.library.MspProtocol;
import com.wang.canrotor.library.UsbService;

import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.Set;

import static android.widget.Toast.makeText;
import static com.wang.canrotor.library.MspProtocol.MSP_ACC_CALIBRATION;
import static com.wang.canrotor.library.MspProtocol.MSP_MAG_CALIBRATION;
import static com.wang.canrotor.library.MspProtocol.MSP_MOBILE;
import static com.wang.canrotor.library.MspProtocol.MSP_PID;
import static com.wang.canrotor.library.MspProtocol.MSP_RESET;
import static com.wang.canrotor.library.MspProtocol.MSP_SET_MOTOR;
import static com.wang.canrotor.library.MspProtocol.MSP_SET_PID;
import static com.wang.canrotor.library.MspProtocol.TELEMERY_PID_SAVE;

public class MainActivity extends AppCompatActivity implements PidFragment.PidFragmentListener, SetupFragment.SetupFragmentListener {
    MyThread mThread;
    long start = 0;
    long end = 0;
    public boolean msp_request = false;
    /*
     * Notifications from UsbService will be received here.
     */
    private final BroadcastReceiver mUsbReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            switch (intent.getAction()) {
                case UsbService.ACTION_USB_PERMISSION_GRANTED: // USB PERMISSION GRANTED
                    makeText(context, "USB Ready", Toast.LENGTH_SHORT).show();
                    break;
                case UsbService.ACTION_USB_PERMISSION_NOT_GRANTED: // USB PERMISSION NOT GRANTED
                    makeText(context, "USB Permission not granted", Toast.LENGTH_SHORT).show();
                    break;
                case UsbService.ACTION_NO_USB: // NO USB CONNECTED
                    makeText(context, "No USB connected", Toast.LENGTH_SHORT).show();
                    break;
                case UsbService.ACTION_USB_DISCONNECTED: // USB DISCONNECTED
                    makeText(context, "USB disconnected", Toast.LENGTH_SHORT).show();
                    break;
                case UsbService.ACTION_USB_NOT_SUPPORTED: // USB NOT SUPPORTED
                    makeText(context, "USB device not supported", Toast.LENGTH_SHORT).show();
                    break;
            }
        }
    };
    private UsbService usbService;

    public MyHandler mHandler;

    private boolean read_flag, write_flag, radio_flag, reset_flag, save_flag, accCalib_flag, magCalib_flag, mot_flag;

    private TextView radio_THR, radio_ROLL, radio_PITCH, radio_YAW, radio_GEAR, radio_AUX1;
    private TextView drone_Error, tlemetry_Error;

    private TextView state_Armed, state_Mode, state_HeadFree, state_FlyTime, state_Temp, state_Volt, state_CycleTime, state_Alt;
    private TextView att_ROLL, att_PITCH, att_YAW;

    private int[] mot_data = new int[4];


    private final ServiceConnection usbConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName arg0, IBinder arg1) {
            usbService = ((UsbService.UsbBinder) arg1).getService();
            usbService.setHandler(mHandler);
        }

        @Override
        public void onServiceDisconnected(ComponentName arg0) {
            usbService = null;
        }
    };

    SetupFragment setupFragment;
    AttitudeFragment attitudeFragment;
    ChartFragment chartFragment;
    PidFragment pidFragment;
    MapFragment mapFragment;

    MspProtocol mspProtocol = new MspProtocol();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Log.d("Wang", "MainActivity");
        setupFragment = new SetupFragment();
        getSupportFragmentManager().beginTransaction().add(R.id.container, setupFragment).commit();
        pidFragment = new PidFragment();
        attitudeFragment = new AttitudeFragment();
        mapFragment = new MapFragment();
        chartFragment = new ChartFragment();
        mHandler = new MyHandler(this);

        ToggleButton toggleButton = findViewById(R.id.toggleButton);
        toggleButton.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
                if (b & msp_request == false) {
                    msp_request = true;
                } else {
                    msp_request = false;
                }
            }
        });

        Button reset_Button = findViewById(R.id.button_Reset);
        reset_Button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (msp_request == false) {
                    Toast.makeText(getApplicationContext(), "우선 기체와 통신을 해주세요.", Toast.LENGTH_SHORT).show();
                } else {
                    showMessage_Reset();
                }
            }
        });

        Button accCaib_Button = findViewById(R.id.button_ACCcalibration);
        accCaib_Button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (msp_request == false) {
                    Toast.makeText(getApplicationContext(), "우선 기체와 통신을 해주세요.", Toast.LENGTH_SHORT).show();
                } else {
                    showMessage_AccCailb();
                }
            }
        });

        Button magCaib_Button = findViewById(R.id.button_MAGcalibration);
        magCaib_Button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (msp_request == false) {
                    Toast.makeText(getApplicationContext(), "우선 기체와 통신을 해주세요.", Toast.LENGTH_SHORT).show();
                } else {
                    showMessage_MagCailb();
                }
            }
        });

        drone_Error = findViewById(R.id.drone_Error);
        tlemetry_Error = findViewById(R.id.telemetry_Error);

        att_ROLL = findViewById(R.id.ROLL_View);
        att_PITCH = findViewById(R.id.PITCH_View);
        att_YAW = findViewById(R.id.YAW_View);

        radio_THR = findViewById(R.id.Radio_THROT);
        radio_ROLL = findViewById(R.id.Radio_ROLL);
        radio_PITCH = findViewById(R.id.Radio_PITCH);
        radio_YAW = findViewById(R.id.Radio_YAW);
        radio_GEAR = findViewById(R.id.Radio_GEAR);
        radio_AUX1 = findViewById(R.id.Radio_AUX1);

        state_Armed = findViewById(R.id.textARMED);
        state_HeadFree = findViewById(R.id.textHEADFREE);
        state_CycleTime = findViewById(R.id.textCYCLETIME);
        state_Mode = findViewById(R.id.textMODE);
        state_Alt = findViewById(R.id.textALT);
        state_Volt = findViewById(R.id.textVOLT);
        state_Temp = findViewById(R.id.textTEMP);

        mThread = new MyThread();
        mThread.setDaemon(true);
        mThread.start();

        sendThread msendThread = new sendThread();
        Thread t = new Thread(msendThread);
        t.start();
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.activity_menu, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        Toast toast = Toast.makeText(getApplicationContext(), "", Toast.LENGTH_LONG);

        switch (item.getItemId()) {
            case R.id.SETUP:
                toast.setText("Setup Menu");
                getSupportFragmentManager().beginTransaction().replace(R.id.container, setupFragment).commit();
                break;

            case R.id.Attitude_menu:
                toast.setText("Attitude Menu");
                getSupportFragmentManager().beginTransaction().replace(R.id.container, attitudeFragment).commit();
                break;

            case R.id.Chart_menu:
                toast.setText("CHART Menu");
                getSupportFragmentManager().beginTransaction().replace(R.id.container, chartFragment).commit();
                break;

            case R.id.PID_Setting:
                read_flag = true;
                toast.setText("Select PID 세팅");
                getSupportFragmentManager().beginTransaction().replace(R.id.container, pidFragment).commit();
                break;

            case R.id.Google_Map:
                toast.setText("Select 지도");
                getSupportFragmentManager().beginTransaction().replace(R.id.container, mapFragment).commit();
                break;
        }
        toast.show();
        return super.onOptionsItemSelected(item);
    }

    private void showMessage_Reset() {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle("경고!");
        builder.setMessage("정말 리셋 하시겠습니까?");
        builder.setIcon(android.R.drawable.ic_dialog_alert);
        builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
                reset_flag = true;
                mspProtocol.error_count = 0;
                Toast.makeText(MainActivity.this, "기체 리셋을 요청하였습니다.", Toast.LENGTH_SHORT).show();
            }
        });

        builder.setNeutralButton("취소", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {

            }
        });

        builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
                Toast.makeText(MainActivity.this, "기체 리셋을 요청을 취소하였습니다.", Toast.LENGTH_SHORT).show();
            }
        });

        AlertDialog dialog = builder.create();
        dialog.show();
    }

    private void showMessage_AccCailb() {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle("경고!");
        builder.setMessage("정말 가속도 센서 캘리브레이션을 하시겠습니까?");
        builder.setIcon(android.R.drawable.ic_dialog_alert);
        builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
                accCalib_flag = true;
                Toast.makeText(MainActivity.this, "기체 가속도 센서 캘리브레이션을 요청하였습니다.", Toast.LENGTH_SHORT).show();
            }
        });

        builder.setNeutralButton("취소", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {

            }
        });

        builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
                Toast.makeText(MainActivity.this, "기체 가속도 센서 캘리브레이션을 취소하였습니다.", Toast.LENGTH_SHORT).show();
            }
        });

        AlertDialog dialog = builder.create();
        dialog.show();
    }

    private void showMessage_MagCailb() {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle("경고!");
        builder.setMessage("정말 지자기 센서 캘리브레이션을 하시겠습니까?");
        builder.setIcon(android.R.drawable.ic_dialog_alert);
        builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
                magCalib_flag = true;
                Toast.makeText(MainActivity.this, "기체 지자기 센서 캘리브레이션을 요청하였습니다.", Toast.LENGTH_SHORT).show();
            }
        });

        builder.setNeutralButton("취소", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {

            }
        });

        builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
                Toast.makeText(MainActivity.this, "기체 지자기 센서 캘리브레이션을 취소하였습니다.", Toast.LENGTH_SHORT).show();
            }
        });

        AlertDialog dialog = builder.create();
        dialog.show();
    }

    @Override
    public void onResume() {
        super.onResume();
        setFilters();  // Start listening notifications from UsbService
        startService(UsbService.class, usbConnection, null); // Start UsbService(if it was not started before) and Bind it
    }

    @Override
    public void onPause() {
        super.onPause();
        unregisterReceiver(mUsbReceiver);
        unbindService(usbConnection);
    }

    private void startService(Class<?> service, ServiceConnection serviceConnection, Bundle extras) {
        if (!UsbService.SERVICE_CONNECTED) {
            Intent startService = new Intent(this, service);
            if (extras != null && !extras.isEmpty()) {
                Set<String> keys = extras.keySet();
                for (String key : keys) {
                    String extra = extras.getString(key);
                    startService.putExtra(key, extra);
                }
            }
            startService(startService);
        }
        Intent bindingIntent = new Intent(this, service);
        bindService(bindingIntent, serviceConnection, Context.BIND_AUTO_CREATE);
    }

    private void setFilters() {
        IntentFilter filter = new IntentFilter();
        filter.addAction(UsbService.ACTION_USB_PERMISSION_GRANTED);
        filter.addAction(UsbService.ACTION_NO_USB);
        filter.addAction(UsbService.ACTION_USB_DISCONNECTED);
        filter.addAction(UsbService.ACTION_USB_NOT_SUPPORTED);
        filter.addAction(UsbService.ACTION_USB_PERMISSION_NOT_GRANTED);
        registerReceiver(mUsbReceiver, filter);
    }

    @Override
    public void saveClicked(boolean flag) {
        save_flag = flag;
    }

    @Override
    public void readClicked() {
        Bundle pid_bundle = new Bundle();
        pid_bundle.putString("pid_angle_roll_p", Float.toString(mspProtocol.byteP[0]));
        pid_bundle.putString("pid_angle_roll_i", Float.toString(mspProtocol.byteI[0]));
        pid_bundle.putString("pid_angle_roll_d", Float.toString(mspProtocol.byteD[0]));
        pid_bundle.putString("pid_angle_pitch_p", Float.toString(mspProtocol.byteP[1]));
        pid_bundle.putString("pid_angle_pitch_i", Float.toString(mspProtocol.byteI[1]));
        pid_bundle.putString("pid_angle_pitch_d", Float.toString(mspProtocol.byteD[1]));
        pid_bundle.putString("pid_angle_yaw_p", Float.toString(mspProtocol.byteP[2]));
        pid_bundle.putString("pid_angle_yaw_i", Float.toString(mspProtocol.byteI[2]));
        pid_bundle.putString("pid_angle_yaw_d", Float.toString(mspProtocol.byteD[2]));

        pid_bundle.putString("pid_dual_out_roll_p", Float.toString(mspProtocol.byteP_outer[0]));
        pid_bundle.putString("pid_dual_out_roll_i", Float.toString(mspProtocol.byteI_outer[0]));
        pid_bundle.putString("pid_dual_out_pitch_p", Float.toString(mspProtocol.byteP_outer[1]));
        pid_bundle.putString("pid_dual_out_pitch_i", Float.toString(mspProtocol.byteI_outer[1]));
        pid_bundle.putString("pid_dual_out_yaw_p", Float.toString(mspProtocol.byteP_outer[2]));
        pid_bundle.putString("pid_dual_out_yaw_i", Float.toString(mspProtocol.byteI_outer[2]));

        pid_bundle.putString("pid_dual_in_roll_p", Float.toString(mspProtocol.byteP_inner[0]));
        pid_bundle.putString("pid_dual_in_roll_i", Float.toString(mspProtocol.byteI_inner[0]));
        pid_bundle.putString("pid_dual_in_roll_d", Float.toString(mspProtocol.byteD_inner[0]));
        pid_bundle.putString("pid_dual_in_pitch_p", Float.toString(mspProtocol.byteP_inner[1]));
        pid_bundle.putString("pid_dual_in_pitch_i", Float.toString(mspProtocol.byteI_inner[1]));
        pid_bundle.putString("pid_dual_in_pitch_d", Float.toString(mspProtocol.byteD_inner[1]));
        pid_bundle.putString("pid_dual_in_yaw_p", Float.toString(mspProtocol.byteP_inner[2]));
        pid_bundle.putString("pid_dual_in_yaw_i", Float.toString(mspProtocol.byteI_inner[2]));
        pid_bundle.putString("pid_dual_in_yaw_d", Float.toString(mspProtocol.byteD_inner[2]));

        pid_bundle.putString("pid_rate_roll_p", Float.toString(mspProtocol.byteP_rate[0]));
        pid_bundle.putString("pid_rate_roll_i", Float.toString(mspProtocol.byteI_rate[0]));
        pid_bundle.putString("pid_rate_roll_d", Float.toString(mspProtocol.byteD_rate[0]));

        pid_bundle.putString("pid_rate_pitch_p", Float.toString(mspProtocol.byteP_rate[1]));
        pid_bundle.putString("pid_rate_pitch_i", Float.toString(mspProtocol.byteI_rate[1]));
        pid_bundle.putString("pid_rate_pitch_d", Float.toString(mspProtocol.byteD_rate[1]));

        pid_bundle.putString("pid_rate_yaw_p", Float.toString(mspProtocol.byteP_rate[2]));
        pid_bundle.putString("pid_rate_yaw_i", Float.toString(mspProtocol.byteI_rate[2]));
        pid_bundle.putString("pid_rate_yaw_d", Float.toString(mspProtocol.byteD_rate[2]));
        pidFragment.setArguments(pid_bundle);
    }

    @Override
    public void writeClicked(float[] parameter) {
        WRITE(parameter);
    }

    @Override
    public void radioClicked(boolean flag) {
        radio_flag = flag;
        Log.d("radioClicked", "뭐냐너?" + radio_flag);
    }

    @Override
    public void motor_data(int[] data) {
        mot_data = data;
        mot_flag = true;
    }

    /*
     * This handler will be passed to UsbService. Data received from serial port is displayed through this handler
     */
    private class MyHandler extends Handler {
        private final WeakReference<MainActivity> mActivity;

        public MyHandler(MainActivity activity) {
            mActivity = new WeakReference<>(activity);
        }

        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
                case UsbService.MESSAGE_FROM_SERIAL_PORT:
                    //String data = (String) msg.obj;
                    byte[] data = (byte[]) msg.obj;
                    //byte[] mspBuf = data.getBytes();
//                    for(int i=0; i<data.length;i++){
//                        int Test = data[i] & 0xff;
//                        Log.d("Main", i+" : "+ Test);
//                    }

                    mspProtocol.SerialCom(data);
                    start = System.currentTimeMillis();
                    Log.w("Period_Time", (start - end)+"ms");
                    end = System.currentTimeMillis();

                    state_CycleTime.setText(Integer.toString(mspProtocol.cycleTime));
                    Log.d("WANG_D", "값이 왜 멈춰는거야? "+Integer.toString(mspProtocol.cycleTime));
                    tlemetry_Error.setText(Integer.toString(mspProtocol.error_count));

                    if (mspProtocol.ARMED == 1) {
                        state_Armed.setText("ARMED");
                    } else {
                        state_Armed.setText("DISARMED");
                    }

                    if (mspProtocol.ANGLE_MODE == 1) {
                        state_Mode.setText("ANGLE");
                    } else if (mspProtocol.HORIZON_MODE == 1) {
                        state_Mode.setText("HORI");
                    } else if (mspProtocol.ACRO_MODE == 1) {
                        state_Mode.setText("ACRO");
                    }

                    if (mspProtocol.HEADFREE_MODE == 1) {
                        state_HeadFree.setText("YES");
                    } else {
                        state_HeadFree.setText("NO");
                    }

                    if (mspProtocol.error == 0) {
                        drone_Error.setText("에러 없음 (code : 0)");
                    } else if (mspProtocol.error == 1) {
                        drone_Error.setText("초기화 실패 (code : 1)");
                    } else if (mspProtocol.error == 2) {
                        drone_Error.setText("내부 통신 에러 (code : 2)");
                    } else if (mspProtocol.error == 3) {
                        drone_Error.setText("조정기 초기화 실패 (code : 3)");
                    } else if (mspProtocol.error == 4) {
                        drone_Error.setText("루프시간 초과 (code : 4)");
                    }
                    state_Alt.setText(Float.toString(mspProtocol.alt));
                    state_Volt.setText(Float.toString(mspProtocol.VBAT));
                    state_Temp.setText(Float.toString(mspProtocol.Temperature));

                    att_ROLL.setText(Float.toString(mspProtocol.ROLL));
                    att_PITCH.setText(Float.toString(mspProtocol.PITCH));
                    att_YAW.setText(Float.toString(mspProtocol.YAW));

                    Bundle attitude_bundle = new Bundle();
                    attitude_bundle.putFloat("attitude_roll", mspProtocol.ROLL);
                    attitude_bundle.putFloat("attitude_pitch", mspProtocol.PITCH);
                    attitudeFragment.setArguments(attitude_bundle);

                    radio_THR.setText(Integer.toString(mspProtocol.RCchan[mspProtocol.RCThro]));
                    radio_ROLL.setText(Integer.toString(mspProtocol.RCchan[mspProtocol.RCRoll]));
                    radio_PITCH.setText(Integer.toString(mspProtocol.RCchan[mspProtocol.RCPitch]));
                    radio_YAW.setText(Integer.toString(mspProtocol.RCchan[mspProtocol.RCYaw]));
                    radio_GEAR.setText(Integer.toString(mspProtocol.RCchan[mspProtocol.RCAUX1]));
                    radio_AUX1.setText(Integer.toString(mspProtocol.RCchan[mspProtocol.RCAUX2]));
                    break;
                case UsbService.CTS_CHANGE:
                    Toast.makeText(mActivity.get(), "CTS_CHANGE", Toast.LENGTH_LONG).show();
                    break;
                case UsbService.DSR_CHANGE:
                    Toast.makeText(mActivity.get(), "DSR_CHANGE", Toast.LENGTH_LONG).show();
                    break;
            }
        }
    }

    public class MyThread extends Thread {
        public void run() {
            while (true) {
                try {
                    while (msp_request) {
                        //Log.d("Thread_wang", "mainThread");
                        if (read_flag != true) {
                            int[] requests = {MSP_MOBILE};
                            byte[] arr = mspProtocol.sendRequestMSP(mspProtocol.requestMSP(requests));
                            if (usbService != null) { // if UsbService was correctly binded, Send data
                                usbService.write(arr);
                            }
                        }
                        if (reset_flag == true) {
                            reset_flag = false;
                            int[] requests1 = {MSP_RESET};
                            byte[] arr1 = mspProtocol.sendRequestMSP(mspProtocol.requestMSP(requests1));
                            if (usbService != null) { // if UsbService was correctly binded, Send data
                                usbService.write(arr1);
                            }
                        }
                        if (save_flag == true) {
                            save_flag = false;
                            int[] requests2 = {TELEMERY_PID_SAVE};
                            byte[] arr2 = mspProtocol.sendRequestMSP(mspProtocol.requestMSP(requests2));
                            if (usbService != null) { // if UsbService was correctly binded, Send data
                                usbService.write(arr2);
                            }
                        }
                        if (accCalib_flag == true) {
                            accCalib_flag = false;
                            int[] requests3 = {MSP_ACC_CALIBRATION};
                            byte[] arr3 = mspProtocol.sendRequestMSP(mspProtocol.requestMSP(requests3));
                            if (usbService != null) { // if UsbService was correctly binded, Send data
                                usbService.write(arr3);
                            }
                        }
                        if (magCalib_flag == true) {
                            magCalib_flag = false;
                            int[] requests4 = {MSP_MAG_CALIBRATION};
                            byte[] arr4 = mspProtocol.sendRequestMSP(mspProtocol.requestMSP(requests4));
                            if (usbService != null) { // if UsbService was correctly binded, Send data
                                usbService.write(arr4);
                            }
                        }
                        if (read_flag == true) {
                            read_flag = false;
                            int[] requests5 = {MSP_PID};
                            byte[] arr5 = mspProtocol.sendRequestMSP(mspProtocol.requestMSP(requests5));
                            if (usbService != null) { // if UsbService was correctly binded, Send data
                                usbService.write(arr5);
                            }
                        }
                        if (mot_flag == true) {
                            mot_flag = false;
                            mspProtocol.payload = new ArrayList<Character>();
                            for (int i = 0; i < 4; i++) {
                                int q;
                                q = mot_data[i];
                                mspProtocol.payload.add((char) (q % 256));
                                mspProtocol.payload.add((char) (q / 256));
                            }
                            byte[] data = mspProtocol.sendRequestMSP(mspProtocol.requestMSP(MSP_SET_MOTOR, mspProtocol.payload.toArray(new Character[mspProtocol.payload.size()])));
                            if (usbService != null) { // if UsbService was correctly binded, Send data
                                usbService.write(data);
                            }
                        }
                        Thread.sleep(1000);
                    }
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

            }
        }
    }

    public class sendThread implements Runnable {
        @Override
        public void run() {
            while (true) {
                try {
                    Log.d("Thread_wang", "Main SendThread"+mThread.getName());

                    Bundle setup_bundle = new Bundle();
                    setup_bundle.putString("mot_1", Integer.toString(mspProtocol.mot[0]));
                    setup_bundle.putString("mot_2", Integer.toString(mspProtocol.mot[1]));
                    setup_bundle.putString("mot_3", Integer.toString(mspProtocol.mot[2]));
                    setup_bundle.putString("mot_4", Integer.toString(mspProtocol.mot[3]));

                    setupFragment.setArguments(setup_bundle);
                    Thread.sleep(500);

                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    public void WRITE(float[] parameter) {
        int tmp_P[] = new int[mspProtocol.PIDITEMS], tmp_I[] = new int[mspProtocol.PIDITEMS], tmp_D[] = new int[mspProtocol.PIDITEMS],
                tmp_P_dual_outer[] = new int[mspProtocol.PIDITEMS], tmp_I_dual_outer[] = new int[mspProtocol.PIDITEMS], tmp_P_dual_inner[] = new int[mspProtocol.PIDITEMS], tmp_I_dual_inner[] = new int[mspProtocol.PIDITEMS], tmp_D_dual_inner[] = new int[mspProtocol.PIDITEMS],
                tmp_P_rate[] = new int[mspProtocol.PIDITEMS], tmp_I_rate[] = new int[mspProtocol.PIDITEMS], tmp_D_rate[] = new int[mspProtocol.PIDITEMS];
        write_flag = true;
        // MSP_SET_PID
        mspProtocol.payload = new ArrayList<Character>();

        tmp_P[0] = (int) (parameter[0] * 10);
        tmp_P[1] = (int) (parameter[3] * 10);
        tmp_P[2] = (int) (parameter[6] * 10);
        tmp_I[0] = (int) (parameter[1] * 10);
        tmp_I[1] = (int) (parameter[4] * 10);
        tmp_I[2] = (int) (parameter[7] * 10);
        tmp_D[0] = (int) (parameter[2] * 10);
        tmp_D[1] = (int) (parameter[5] * 10);
        tmp_D[2] = (int) (parameter[8] * 10);

        tmp_P_dual_outer[0] = (int) (parameter[9] * 10);
        tmp_P_dual_outer[1] = (int) (parameter[11] * 10);
        tmp_P_dual_outer[2] = (int) (parameter[13] * 10);
        tmp_I_dual_outer[0] = (int) (parameter[10] * 10);
        tmp_I_dual_outer[1] = (int) (parameter[12] * 10);
        tmp_I_dual_outer[2] = (int) (parameter[14] * 10);

        tmp_P_dual_inner[0] = (int) (parameter[15] * 10);
        tmp_P_dual_inner[1] = (int) (parameter[18] * 10);
        tmp_P_dual_inner[2] = (int) (parameter[21] * 10);
        tmp_I_dual_inner[0] = (int) (parameter[16] * 10);
        tmp_I_dual_inner[1] = (int) (parameter[19] * 10);
        tmp_I_dual_inner[2] = (int) (parameter[22] * 10);
        tmp_D_dual_inner[0] = (int) (parameter[17] * 10);
        tmp_D_dual_inner[1] = (int) (parameter[20] * 10);
        tmp_D_dual_inner[2] = (int) (parameter[23] * 10);

        tmp_P_rate[0] = (int) (parameter[24] * 10);
        tmp_P_rate[1] = (int) (parameter[27] * 10);
        tmp_P_rate[2] = (int) (parameter[30] * 10);
        tmp_I_rate[0] = (int) (parameter[25] * 10);
        tmp_I_rate[1] = (int) (parameter[28] * 10);
        tmp_I_rate[2] = (int) (parameter[31] * 10);
        tmp_D_rate[0] = (int) (parameter[26] * 10);
        tmp_D_rate[1] = (int) (parameter[29] * 10);
        tmp_D_rate[2] = (int) (parameter[32] * 10);

        for (int i = 0; i < mspProtocol.PIDITEMS; i++) {
            int q;
            q = tmp_P[i];
            mspProtocol.payload.add((char) (q % 256));
            mspProtocol.payload.add((char) (q / 256));
            q = tmp_I[i];
            mspProtocol.payload.add((char) (q % 256));
            mspProtocol.payload.add((char) (q / 256));
            q = tmp_D[i];
            mspProtocol.payload.add((char) (q % 256));
            mspProtocol.payload.add((char) (q / 256));
            q = tmp_P_dual_outer[i];
            mspProtocol.payload.add((char) (q % 256));
            mspProtocol.payload.add((char) (q / 256));
            q = tmp_I_dual_outer[i];
            mspProtocol.payload.add((char) (q % 256));
            mspProtocol.payload.add((char) (q / 256));
            q = tmp_P_dual_inner[i];
            mspProtocol.payload.add((char) (q % 256));
            mspProtocol.payload.add((char) (q / 256));
            q = tmp_I_dual_inner[i];
            mspProtocol.payload.add((char) (q % 256));
            mspProtocol.payload.add((char) (q / 256));
            q = tmp_D_dual_inner[i];
            mspProtocol.payload.add((char) (q % 256));
            mspProtocol.payload.add((char) (q / 256));
            q = tmp_P_rate[i];
            mspProtocol.payload.add((char) (q % 256));
            mspProtocol.payload.add((char) (q / 256));
            q = tmp_I_rate[i];
            mspProtocol.payload.add((char) (q % 256));
            mspProtocol.payload.add((char) (q / 256));
            q = tmp_D_rate[i];
            mspProtocol.payload.add((char) (q % 256));
            mspProtocol.payload.add((char) (q / 256));
        }

        byte[] data = mspProtocol.sendRequestMSP(mspProtocol.requestMSP(MSP_SET_PID, mspProtocol.payload.toArray(new Character[mspProtocol.payload.size()])));
        if (usbService != null) { // if UsbService was correctly binded, Send data
            usbService.write(data);
        }
        write_flag = false;
    }

}
//    long start = System.currentTimeMillis();
//    long end = System.currentTimeMillis();
//    Log.w("Period_Time", (end - start)+"ms");
