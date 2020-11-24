package com.wang.canrotor.activity;

import android.content.Context;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.support.annotation.Nullable;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.LinearLayout;
import android.widget.RadioGroup;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import com.wang.canrotor.R;
import com.wang.canrotor.opengl.MyGLSurfaceView;

public class SetupFragment extends Fragment {
    setupshowThread showThread;
    Thread t;
    private TextView mot_1, mot_2, mot_3, mot_4;
    private SeekBar sb_mot_1, sb_mot_2, sb_mot_3, sb_mot_4;
    private RadioGroup selectMode_MotorControl;
    private boolean running, mot_thread_running;
    final int max = 4500;
    final int min = 2250;
    final int step = 1;
    private int[] Mot_data = new int[4];

    static interface SetupFragmentListener {
        void radioClicked(boolean flag);

        void motor_data(int[] data);
    }

    private SetupFragmentListener setup_fragment_listener;
    private GLSurfaceView mGLView;

    @Nullable
    @Override
    public View onCreateView(LayoutInflater inflater, @Nullable ViewGroup container, Bundle savedInstanceState) {
        ViewGroup rootView = (ViewGroup) inflater.inflate(R.layout.fragment_setup, container, false);
        Log.d("Wang", "Setup_Fragment(onCreateView)");

        mGLView = new MyGLSurfaceView(getContext());
        LinearLayout rootLayout = rootView.findViewById(R.id.rootLayout);
        rootLayout.addView(mGLView);

        // Create a GLSurfaceView instance and set it
        // as the ContentView for this Activity.
        SeekBar myBar = rootView.findViewById(R.id.seekBar);
        myBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {

                Log.i("SeekBar", Integer.toString(progress));

                MyGLSurfaceView obj = new MyGLSurfaceView(getContext());
                obj.setRenderer(progress);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        mot_1 = rootView.findViewById(R.id.MOTOR1);
        mot_2 = rootView.findViewById(R.id.MOTOR2);
        mot_3 = rootView.findViewById(R.id.MOTOR3);
        mot_4 = rootView.findViewById(R.id.MOTOR4);

        sb_mot_1 = rootView.findViewById(R.id.sb_motor1);
        sb_mot_1.setMax((max - min) / step);
        sb_mot_2 = rootView.findViewById(R.id.sb_motor2);
        sb_mot_2.setMax((max - min) / step);
        sb_mot_3 = rootView.findViewById(R.id.sb_motor3);
        sb_mot_3.setMax((max - min) / step);
        sb_mot_4 = rootView.findViewById(R.id.sb_motor4);
        sb_mot_4.setMax((max - min) / step);

        selectMode_MotorControl = rootView.findViewById(R.id.radio_Group);
        selectMode_MotorControl.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(RadioGroup radioGroup, int i) {
                switch (i) {
                    case R.id.radioButton_Control:
                        if (setup_fragment_listener != null) {
                            setup_fragment_listener.radioClicked(false);
                            mot_thread_running = false;
                            mot_1.setText(Integer.toString(min));
                            mot_2.setText(Integer.toString(min));
                            mot_3.setText(Integer.toString(min));
                            mot_4.setText(Integer.toString(min));
                            running = true;
                            Toast.makeText(getContext(), "조정기로 조작 할 수 있습니다.", Toast.LENGTH_SHORT).show();
                        }
                        break;
                    case R.id.radioButton_Manual:
                        if (setup_fragment_listener != null) {
                            setup_fragment_listener.radioClicked(true);
                            mot_thread_running = true;
                            running = false;
                            sb_mot_1.setProgress(0);
                            sb_mot_2.setProgress(0);
                            sb_mot_3.setProgress(0);
                            sb_mot_4.setProgress(0);
                            Toast.makeText(getContext(), "수동으로 조작 할 수 있습니다.", Toast.LENGTH_SHORT).show();
                        }
                        break;
                }
            }
        });
        moterListener();
        motThread mot_Thread = new motThread();
        t = new Thread(mot_Thread);
        t.start();

        running = true;
        showThread = new setupshowThread();
        showThread.setDaemon(true);
        showThread.start();
        return rootView;
    }

    public class setupshowThread extends Thread {
        @Override
        public void run() {
            while (running) {
                try {
                    Log.d("Thread_wang", "setupThread"+showThread.getName());
                    Bundle bundle = getArguments();
                    if (bundle != null) {
                        mot_1.setText(bundle.getString("mot_1"));
                        mot_2.setText(bundle.getString("mot_2"));
                        mot_3.setText(bundle.getString("mot_3"));
                        mot_4.setText(bundle.getString("mot_4"));

                        sb_mot_1.setProgress(bundle.getInt("mot_1_int"));
                        sb_mot_2.setProgress(bundle.getInt("mot_2_int"));
                        sb_mot_3.setProgress(bundle.getInt("mot_3_int"));
                        sb_mot_4.setProgress(bundle.getInt("mot_4_int"));
 //                       Log.d("Motor_data", Mot_data[0] + "show값" + Mot_data[1] + "값" + Mot_data[2] + "값" + Mot_data[3] + "값");

//                        sb_mot_1.setProgress(bundle.getInt("sb_mot_1"));
//                        sb_mot_2.setProgress(bundle.getInt("sb_mot_2"));
//                        sb_mot_3.setProgress(bundle.getInt("sb_mot_3"));
//                        sb_mot_4.setProgress(bundle.getInt("sb_mot_4"));
                    }
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

            }
        }
    }

    public class motThread implements Runnable {
        @Override
        public void run() {
            while (mot_thread_running) {
                try {
                    if (setup_fragment_listener != null) {
                        setup_fragment_listener.motor_data(Mot_data);
                        Log.d("Motor_data", Mot_data[0] + "mot값" + Mot_data[1] + "값" + Mot_data[2] + "값" + Mot_data[3] + "값");
                    }
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    @Override
    public void onAttach(Context context) {
        super.onAttach(context);
        try {
            if (context instanceof SetupFragmentListener)
                this.setup_fragment_listener = (SetupFragmentListener) context;
        } catch (ClassCastException e) {
            throw new ClassCastException(context.toString() + " must implement PidFragmentListener");
        }
        Log.d("Fragment", "Setup_Fragment(onAttach)");
    }

    @Override
    public void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.d("Fragment", "Setup_Fragment(onCreate)");
    }

    @Override
    public void onStart() {
        super.onStart();
        Log.d("Fragment", "Setup_Fragment(onStart)");
    }

    @Override
    public void onResume() {
        super.onResume();
        Log.d("Fragment", "Setup_Fragment(onResume)");
    }

    @Override
    public void onPause() {
        super.onPause();
        Log.d("Fragment", "Setup_Fragment(onPause)");
    }

    @Override
    public void onStop() {
        super.onStop();
        Log.d("Fragment", "Setup_Fragment(onStop)");
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        Log.d("Fragment", "Setup_Fragment(onDestroyView)");
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        Log.d("Fragment", "Setup_Fragment(onDestroy)");
    }

    @Override
    public void onDetach() {
        super.onDetach();
        setup_fragment_listener = null;
        running = false;
        Log.d("Fragment", "Setup_Fragment(onDetach)");
    }

    private void moterListener() {
        sb_mot_1.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
                Mot_data[0] = min + (i * step);
                mot_1.setText(Integer.toString(Mot_data[0]));
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        sb_mot_2.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
                Mot_data[1] = min + (i * step);
                mot_2.setText(Integer.toString(Mot_data[1]));
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        sb_mot_3.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
                Mot_data[2] = min + (i * step);
                mot_3.setText(Integer.toString(Mot_data[2]));
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        sb_mot_4.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
                Mot_data[3] = min + (i * step);
                mot_4.setText(Integer.toString(Mot_data[3]));
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
    }

}
