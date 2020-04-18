package com.wang.canrotor.activity;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.os.Bundle;
import android.support.annotation.Nullable;
import android.support.constraint.ConstraintLayout;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import com.wang.canrotor.R;


public class PidFragment extends Fragment {
    Activity mActivity;
    showThread mThread;
    private TextView pid_angle_roll_p, pid_angle_roll_i, pid_angle_roll_d, pid_angle_pitch_p, pid_angle_pitch_i, pid_angle_pitch_d, pid_angle_yaw_p, pid_angle_yaw_i, pid_angle_yaw_d,
            pid_dual_out_roll_p, pid_dual_out_roll_i, pid_dual_out_pitch_p, pid_dual_out_pitch_i, pid_dual_out_yaw_p, pid_dual_out_yaw_i,
            pid_dual_in_roll_p, pid_dual_in_roll_i, pid_dual_in_roll_d, pid_dual_in_pitch_p, pid_dual_in_pitch_i, pid_dual_in_pitch_d, pid_dual_in_yaw_p, pid_dual_in_yaw_i, pid_dual_in_yaw_d,
            pid_rate_roll_p, pid_rate_roll_i, pid_rate_roll_d, pid_rate_pitch_p, pid_rate_pitch_i, pid_rate_pitch_d, pid_rate_yaw_p, pid_rate_yaw_i, pid_rate_yaw_d;

    private Button angle_roll_sub, angle_roll_add;
    private EditText edit_angle_roll;
    private ConstraintLayout angle_roll_layout;
    private boolean running;

    static interface PidFragmentListener {
        void readClicked();

        void writeClicked(float[] parameter);

        void saveClicked(boolean flag);
    }

    private PidFragmentListener pid_fragment_listener;

    @Nullable
    @Override
    public View onCreateView(LayoutInflater inflater, @Nullable ViewGroup container, Bundle savedInstanceState) {
        ViewGroup rootView = (ViewGroup) inflater.inflate(R.layout.fragment_pid, container, false);
        Log.d("Fragment", "PID_Fragment(onCreateView)");

        setup_view(rootView);

        setup_Thread();

        setup_Button(rootView);

        setup_EditText(rootView);

        return rootView;
    }

    @Override
    public void onAttach(Context context) {
        super.onAttach(context);
        try {
            if (context instanceof PidFragmentListener)
                this.pid_fragment_listener = (PidFragmentListener) context;
        } catch (ClassCastException e) {
            throw new ClassCastException(context.toString() + " must implement PidFragmentListener");
        }
        if(context instanceof Activity){
            mActivity = (Activity) context;
        }
        Log.d("Fragment", "PID_Fragment(onAttach)");
    }

    @Override
    public void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.d("Fragment", "PID_Fragment(onCreate)");
    }

    @Override
    public void onStart() {
        super.onStart();
        Log.d("Fragment", "PID_Fragment(onStart)");
    }

    @Override
    public void onResume() {
        super.onResume();
        Log.d("Fragment", "PID_Fragment(onResume)");
    }

    @Override
    public void onPause() {
        super.onPause();
        running = false;
        Log.d("Fragment", "PID_Fragment(onPause)");
    }

    @Override
    public void onStop() {
        super.onStop();
        Log.d("Fragment", "PID_Fragment(onStop)");
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        Log.d("Fragment", "PID_Fragment(onDestroyView)");
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        Log.d("Fragment", "PID_Fragment(onDestroy)");
    }

    @Override
    public void onDetach() {
        super.onDetach();
        pid_fragment_listener = null;
        running = false;
        Log.d("Fragment", "PID_Fragment(onDetach)");
    }

    private void showMessage_pidWrite() {
        AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
        builder.setTitle("경고!");
        builder.setMessage("정말 PID 파라메터값을 적용 오시겠습니까?");
        builder.setIcon(android.R.drawable.ic_dialog_alert);
        builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
                float[] parameter = new float[33];

                parameter[0] = Float.parseFloat(pid_angle_roll_p.getText().toString());
                parameter[1] = Float.parseFloat(pid_angle_roll_i.getText().toString());
                parameter[2] = Float.parseFloat(pid_angle_roll_d.getText().toString());
                parameter[3] = Float.parseFloat(pid_angle_pitch_p.getText().toString());
                parameter[4] = Float.parseFloat(pid_angle_pitch_i.getText().toString());
                parameter[5] = Float.parseFloat(pid_angle_pitch_d.getText().toString());
                parameter[6] = Float.parseFloat(pid_angle_yaw_p.getText().toString());
                parameter[7] = Float.parseFloat(pid_angle_yaw_i.getText().toString());
                parameter[8] = Float.parseFloat(pid_angle_yaw_d.getText().toString());

                parameter[9] = Float.parseFloat(pid_dual_out_roll_p.getText().toString());
                parameter[10] = Float.parseFloat(pid_dual_out_roll_i.getText().toString());
                parameter[11] = Float.parseFloat(pid_dual_out_pitch_p.getText().toString());
                parameter[12] = Float.parseFloat(pid_dual_out_pitch_i.getText().toString());
                parameter[13] = Float.parseFloat(pid_dual_out_yaw_p.getText().toString());
                parameter[14] = Float.parseFloat(pid_dual_out_yaw_i.getText().toString());
                parameter[15] = Float.parseFloat(pid_dual_in_roll_p.getText().toString());
                parameter[16] = Float.parseFloat(pid_dual_in_roll_i.getText().toString());
                parameter[17] = Float.parseFloat(pid_dual_in_roll_d.getText().toString());
                parameter[18] = Float.parseFloat(pid_dual_in_pitch_p.getText().toString());
                parameter[19] = Float.parseFloat(pid_dual_in_pitch_i.getText().toString());
                parameter[20] = Float.parseFloat(pid_dual_in_pitch_d.getText().toString());
                parameter[21] = Float.parseFloat(pid_dual_in_yaw_p.getText().toString());
                parameter[22] = Float.parseFloat(pid_dual_in_yaw_i.getText().toString());
                parameter[23] = Float.parseFloat(pid_dual_in_yaw_d.getText().toString());

                parameter[24] = Float.parseFloat(pid_rate_roll_p.getText().toString());
                parameter[25] = Float.parseFloat(pid_rate_roll_i.getText().toString());
                parameter[26] = Float.parseFloat(pid_rate_roll_d.getText().toString());
                parameter[27] = Float.parseFloat(pid_rate_pitch_p.getText().toString());
                parameter[28] = Float.parseFloat(pid_rate_pitch_i.getText().toString());
                parameter[29] = Float.parseFloat(pid_rate_pitch_d.getText().toString());
                parameter[30] = Float.parseFloat(pid_rate_yaw_p.getText().toString());
                parameter[31] = Float.parseFloat(pid_rate_yaw_i.getText().toString());
                parameter[32] = Float.parseFloat(pid_rate_yaw_d.getText().toString());

                if (pid_fragment_listener != null) {
                    pid_fragment_listener.writeClicked(parameter);
                    Toast.makeText(getContext(), "기체 PID 파라메터값 적용을 요청 하였습니다.", Toast.LENGTH_SHORT).show();
                }
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
                Toast.makeText(getContext(), "기체 PID 파라메터값 적용 요청을 취소하였습니다.", Toast.LENGTH_SHORT).show();
            }
        });

        AlertDialog dialog = builder.create();
        dialog.show();
    }

    private void showMessage_Save() {
        AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
        builder.setTitle("경고!");
        builder.setMessage("정말 기체의 PID 파라메터를 EEPROM에 저장 하시겠습니까?");
        builder.setIcon(android.R.drawable.ic_dialog_alert);
        builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
                if (pid_fragment_listener != null) {
                    pid_fragment_listener.saveClicked(true);
                    Toast.makeText(getContext(), "PID 파라메터를 저장을 요청하였습니다.", Toast.LENGTH_SHORT).show();
                }
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
                Toast.makeText(getContext(), "PID 파라메터 저장 요청을 취소하였습니다.", Toast.LENGTH_SHORT).show();
            }
        });

        AlertDialog dialog = builder.create();
        dialog.show();
    }

    public class showThread extends Thread {
        @Override
        public void run() {
            while (running) {
                try {
                    Log.d("Thread_wang", "pidThread"+mThread.getName());
                    sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                mActivity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {

                    }
                });

            }
        }
    }

    private void setup_view(ViewGroup rootView) {
        pid_angle_roll_p = rootView.findViewById(R.id.angle_roll_p);
        pid_angle_roll_i = rootView.findViewById(R.id.angle_roll_i);
        pid_angle_roll_d = rootView.findViewById(R.id.angle_roll_d);

        pid_angle_pitch_p = rootView.findViewById(R.id.angle_pitch_p);
        pid_angle_pitch_i = rootView.findViewById(R.id.angle_pitch_i);
        pid_angle_pitch_d = rootView.findViewById(R.id.angle_pitch_d);
        pid_angle_yaw_p = rootView.findViewById(R.id.angle_yaw_p);
        pid_angle_yaw_i = rootView.findViewById(R.id.angle_yaw_i);
        pid_angle_yaw_d = rootView.findViewById(R.id.angle_yaw_d);

        pid_dual_out_roll_p = rootView.findViewById(R.id.dual_out_roll_p);
        pid_dual_out_roll_i = rootView.findViewById(R.id.dual_out_roll_i);
        pid_dual_out_pitch_p = rootView.findViewById(R.id.dual_out_pitch_p);
        pid_dual_out_pitch_i = rootView.findViewById(R.id.dual_out_pitch_i);
        pid_dual_out_yaw_p = rootView.findViewById(R.id.dual_out_yaw_p);
        pid_dual_out_yaw_i = rootView.findViewById(R.id.dual_out_yaw_i);
        pid_dual_in_roll_p = rootView.findViewById(R.id.dual_in_roll_p);
        pid_dual_in_roll_i = rootView.findViewById(R.id.dual_in_roll_i);
        pid_dual_in_roll_d = rootView.findViewById(R.id.dual_in_roll_d);
        pid_dual_in_pitch_p = rootView.findViewById(R.id.dual_in_pitch_p);
        pid_dual_in_pitch_i = rootView.findViewById(R.id.dual_in_pitch_i);
        pid_dual_in_pitch_d = rootView.findViewById(R.id.dual_in_pitch_d);
        pid_dual_in_yaw_p = rootView.findViewById(R.id.dual_in_yaw_p);
        pid_dual_in_yaw_i = rootView.findViewById(R.id.dual_in_yaw_i);
        pid_dual_in_yaw_d = rootView.findViewById(R.id.dual_in_yaw_d);

        pid_rate_roll_p = rootView.findViewById(R.id.rate_roll_p);
        pid_rate_roll_i = rootView.findViewById(R.id.rate_roll_i);
        pid_rate_roll_d = rootView.findViewById(R.id.rate_roll_d);
        pid_rate_pitch_p = rootView.findViewById(R.id.rate_pitch_p);
        pid_rate_pitch_i = rootView.findViewById(R.id.rate_pitch_i);
        pid_rate_pitch_d = rootView.findViewById(R.id.rate_pitch_d);
        pid_rate_yaw_p = rootView.findViewById(R.id.rate_yaw_p);
        pid_rate_yaw_i = rootView.findViewById(R.id.rate_yaw_i);
        pid_rate_yaw_d = rootView.findViewById(R.id.rate_yaw_d);

    }

    private void setup_Thread() {
        Log.d("Thread_wang", "setup_Thread 생성");
        running = true;
        mThread = new showThread();
        mThread.setDaemon(true);
        mThread.start();
    }

    private void setup_Button(ViewGroup rootView) {

        Button pidRead_Button = rootView.findViewById(R.id.button_Read);
        pidRead_Button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (((MainActivity) getActivity()).msp_request == false) {
                    Toast.makeText(getContext(), "우선 기체와 통신을 해주세요.", Toast.LENGTH_SHORT).show();
                } else {
                    if (pid_fragment_listener != null) {
                        pid_fragment_listener.readClicked();
                        Toast.makeText(getContext(), "기체에 데이터를 요청했습니다.", Toast.LENGTH_SHORT).show();
                        Bundle bundle = getArguments();
                        if (bundle != null) {
                            pid_angle_roll_p.setText(bundle.getString("pid_angle_roll_p"));
                            pid_angle_roll_i.setText(bundle.getString("pid_angle_roll_i"));
                            pid_angle_roll_d.setText(bundle.getString("pid_angle_roll_d"));
                            pid_angle_pitch_p.setText(bundle.getString("pid_angle_pitch_p"));
                            pid_angle_pitch_i.setText(bundle.getString("pid_angle_pitch_i"));
                            pid_angle_pitch_d.setText(bundle.getString("pid_angle_pitch_d"));
                            pid_angle_yaw_p.setText(bundle.getString("pid_angle_yaw_p"));
                            pid_angle_yaw_i.setText(bundle.getString("pid_angle_yaw_i"));
                            pid_angle_yaw_d.setText(bundle.getString("pid_angle_yaw_d"));

                            pid_dual_out_roll_p.setText(bundle.getString("pid_dual_out_roll_p"));
                            pid_dual_out_roll_i.setText(bundle.getString("pid_dual_out_roll_i"));
                            pid_dual_out_pitch_p.setText(bundle.getString("pid_dual_out_pitch_p"));
                            pid_dual_out_pitch_i.setText(bundle.getString("pid_dual_out_pitch_i"));
                            pid_dual_out_yaw_p.setText(bundle.getString("pid_dual_out_yaw_p"));
                            pid_dual_out_yaw_i.setText(bundle.getString("pid_dual_out_yaw_i"));
                            pid_dual_in_roll_p.setText(bundle.getString("pid_dual_in_roll_p"));
                            pid_dual_in_roll_i.setText(bundle.getString("pid_dual_in_roll_i"));
                            pid_dual_in_roll_d.setText(bundle.getString("pid_dual_in_roll_d"));
                            pid_dual_in_pitch_p.setText(bundle.getString("pid_dual_in_pitch_p"));
                            pid_dual_in_pitch_i.setText(bundle.getString("pid_dual_in_pitch_i"));
                            pid_dual_in_pitch_d.setText(bundle.getString("pid_dual_in_pitch_d"));
                            pid_dual_in_yaw_p.setText(bundle.getString("pid_dual_in_yaw_p"));
                            pid_dual_in_yaw_i.setText(bundle.getString("pid_dual_in_yaw_i"));
                            pid_dual_in_yaw_d.setText(bundle.getString("pid_dual_in_yaw_d"));

                            pid_rate_roll_p.setText(bundle.getString("pid_rate_roll_p"));
                            pid_rate_roll_i.setText(bundle.getString("pid_rate_roll_i"));
                            pid_rate_roll_d.setText(bundle.getString("pid_rate_roll_d"));
                            pid_rate_pitch_p.setText(bundle.getString("pid_rate_pitch_p"));
                            pid_rate_pitch_i.setText(bundle.getString("pid_rate_pitch_i"));
                            pid_rate_pitch_d.setText(bundle.getString("pid_rate_pitch_d"));
                            pid_rate_yaw_p.setText(bundle.getString("pid_rate_yaw_p"));
                            pid_rate_yaw_i.setText(bundle.getString("pid_rate_yaw_i"));
                            pid_rate_yaw_d.setText(bundle.getString("pid_rate_yaw_d"));
                        }
                    }
                }
            }
        });

        Button pidWrite_Button = rootView.findViewById(R.id.button_Write);
        pidWrite_Button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Toast.makeText(getContext(), "적용 버튼", Toast.LENGTH_SHORT).show();
                if (((MainActivity) getActivity()).msp_request == false) {
                    Toast.makeText(getContext(), "우선 기체와 통신을 해주세요.", Toast.LENGTH_SHORT).show();
                } else {
                    showMessage_pidWrite();
                }
            }
        });


        Button save_Button = rootView.findViewById(R.id.button_Save);
        save_Button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Toast.makeText(getContext(), "저장 버튼", Toast.LENGTH_SHORT).show();
                if (((MainActivity) getActivity()).msp_request == false) {
                    Toast.makeText(getContext(), "우선 기체와 통신을 해주세요.", Toast.LENGTH_SHORT).show();
                } else {
                    showMessage_Save();
                }
            }
        });
    }

    private void setup_EditText(ViewGroup rootView) {
        pid_angle_roll_p.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("앵글모드 ROLL, PITCH의 P값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText angle_roll_p = new EditText(mActivity);
                angle_roll_p.setText(pid_angle_roll_p.getText());
                builder.setView(angle_roll_p);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_angle_roll_p.setText(angle_roll_p.getText());
                        pid_angle_pitch_p.setText(angle_roll_p.getText());
                    }
                });


                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_angle_roll_i.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("앵글모드 ROLL, PITCH의 I값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText angle_roll_i = new EditText(mActivity);
                angle_roll_i.setText(pid_angle_roll_i.getText());
                builder.setView(angle_roll_i);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_angle_roll_i.setText(angle_roll_i.getText());
                        pid_angle_pitch_i.setText(angle_roll_i.getText());
                    }
                });


                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_angle_roll_d.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("앵글모드 ROLL, PITCH의 D값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText angle_roll_d = new EditText(mActivity);
                angle_roll_d.setText(pid_angle_roll_d.getText());
                builder.setView(angle_roll_d);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_angle_roll_d.setText(angle_roll_d.getText());
                        pid_angle_pitch_d.setText(angle_roll_d.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_angle_yaw_p.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("앵글모드 YAW의 P값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText angle_yaw_p = new EditText(mActivity);
                angle_yaw_p.setText(pid_angle_yaw_p.getText());
                builder.setView(angle_yaw_p);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_angle_yaw_p.setText(angle_yaw_p.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_angle_yaw_i.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("앵글모드 YAW의 I값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText angle_yaw_i = new EditText(mActivity);
                angle_yaw_i.setText(pid_angle_yaw_i.getText());
                builder.setView(angle_yaw_i);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_angle_yaw_i.setText(angle_yaw_i.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_angle_yaw_d.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("앵글모드 YAW의 D값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText angle_yaw_d = new EditText(mActivity);
                angle_yaw_d.setText(pid_angle_yaw_d.getText());
                builder.setView(angle_yaw_d);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_angle_yaw_d.setText(angle_yaw_d.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_dual_out_roll_p.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("듀얼루프 ROLL, PITCH의 외부 P값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText dual_out_roll_d = new EditText(mActivity);
                dual_out_roll_d.setText(pid_dual_out_roll_p.getText());
                builder.setView(dual_out_roll_d);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_dual_out_roll_p.setText(dual_out_roll_d.getText());
                        pid_dual_out_pitch_p.setText(dual_out_roll_d.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_dual_out_roll_i.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("듀얼루프 ROLL, PITCH의 외부 I값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText dual_out_roll_i = new EditText(mActivity);
                dual_out_roll_i.setText(pid_dual_out_roll_i.getText());
                builder.setView(dual_out_roll_i);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_dual_out_roll_i.setText(dual_out_roll_i.getText());
                        pid_dual_out_pitch_i.setText(dual_out_roll_i.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_dual_out_yaw_p.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("듀얼루프 YAW의 외부 P값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText dual_out_yaw_p = new EditText(mActivity);
                dual_out_yaw_p.setText(pid_dual_out_yaw_p.getText());
                builder.setView(dual_out_yaw_p);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_dual_out_yaw_p.setText(dual_out_yaw_p.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_dual_out_yaw_i.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("듀얼루프 YAW의 외부 I값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText dual_out_yaw_i = new EditText(mActivity);
                dual_out_yaw_i.setText(pid_dual_out_yaw_i.getText());
                builder.setView(dual_out_yaw_i);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_dual_out_yaw_i.setText(dual_out_yaw_i.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_dual_in_roll_p.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("듀얼루프 ROLL, PITCH의 내부 P값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText dual_in_roll_p = new EditText(mActivity);
                dual_in_roll_p.setText(pid_dual_in_roll_p.getText());
                builder.setView(dual_in_roll_p);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_dual_in_roll_p.setText(dual_in_roll_p.getText());
                        pid_dual_in_pitch_p.setText(dual_in_roll_p.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_dual_in_roll_i.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("듀얼루프 ROLL, PITCH의 내부 I값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText dual_in_roll_i = new EditText(mActivity);
                dual_in_roll_i.setText(pid_dual_in_roll_i.getText());
                builder.setView(dual_in_roll_i);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_dual_in_roll_i.setText(dual_in_roll_i.getText());
                        pid_dual_in_pitch_i.setText(dual_in_roll_i.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_dual_in_roll_d.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("듀얼루프 ROLL, PITCH의 내부 D값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText dual_in_roll_d = new EditText(mActivity);
                dual_in_roll_d.setText(pid_dual_in_roll_d.getText());
                builder.setView(dual_in_roll_d);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_dual_in_roll_d.setText(dual_in_roll_d.getText());
                        pid_dual_in_pitch_d.setText(dual_in_roll_d.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_dual_in_yaw_p.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("듀얼루프 YAW의 내부 P값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText dual_in_yaw_p = new EditText(mActivity);
                dual_in_yaw_p.setText(pid_dual_in_yaw_p.getText());
                builder.setView(dual_in_yaw_p);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_dual_in_yaw_p.setText(dual_in_yaw_p.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_dual_in_yaw_i.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("듀얼루프 YAW의 내부 I값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText dual_in_yaw_i = new EditText(mActivity);
                dual_in_yaw_i.setText(pid_dual_in_yaw_i.getText());
                builder.setView(dual_in_yaw_i);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_dual_in_yaw_i.setText(dual_in_yaw_i.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_dual_in_yaw_d.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("듀얼루프 YAW의 내부 D값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText dual_in_yaw_d = new EditText(mActivity);
                dual_in_yaw_d.setText(pid_dual_in_yaw_d.getText());
                builder.setView(dual_in_yaw_d);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_dual_in_yaw_d.setText(dual_in_yaw_d.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_rate_roll_p.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("레이트모드 ROLL, PITCH의 P값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText rate_roll_p = new EditText(mActivity);
                rate_roll_p.setText(pid_rate_roll_p.getText());
                builder.setView(rate_roll_p);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_rate_roll_p.setText(rate_roll_p.getText());
                        pid_rate_pitch_p.setText(rate_roll_p.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_rate_roll_i.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("레이트모드 ROLL, PITCH의 I값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText rate_roll_i = new EditText(mActivity);
                rate_roll_i.setText(pid_rate_roll_i.getText());
                builder.setView(rate_roll_i);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_rate_roll_i.setText(rate_roll_i.getText());
                        pid_rate_pitch_i.setText(rate_roll_i.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_rate_roll_d.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("레이트모드 ROLL, PITCH의 D값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText rate_roll_d = new EditText(mActivity);
                rate_roll_d.setText(pid_rate_roll_d.getText());
                builder.setView(rate_roll_d);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_rate_roll_d.setText(rate_roll_d.getText());
                        pid_rate_pitch_d.setText(rate_roll_d.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_rate_yaw_p.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("레이트모드 YAW의 P값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText rate_yaw_p = new EditText(mActivity);
                rate_yaw_p.setText(pid_rate_yaw_p.getText());
                builder.setView(rate_yaw_p);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_rate_yaw_p.setText(rate_yaw_p.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_rate_yaw_i.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("레이트모드 YAW의 I값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText rate_yaw_i = new EditText(mActivity);
                rate_yaw_i.setText(pid_rate_yaw_i.getText());
                builder.setView(rate_yaw_i);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_rate_yaw_i.setText(rate_yaw_i.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

        pid_rate_yaw_d.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
                builder.setTitle("PID 파라메터 변경");
                builder.setMessage("레이트모드 YAW의 D값을 입력해주세요");
                builder.setIcon(android.R.drawable.ic_dialog_info);
                final EditText rate_yaw_d = new EditText(mActivity);
                rate_yaw_d.setText(pid_rate_yaw_d.getText());
                builder.setView(rate_yaw_d);
                builder.setPositiveButton("예", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        pid_rate_yaw_d.setText(rate_yaw_d.getText());
                    }
                });

                builder.setNegativeButton("아니요", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                    }
                });

                AlertDialog dialog = builder.create();
                dialog.show();
            }
        });

    }

}


