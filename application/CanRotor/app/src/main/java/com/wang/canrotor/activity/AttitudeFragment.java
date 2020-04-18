package com.wang.canrotor.activity;

import android.app.Activity;
import android.content.Context;
import android.os.Bundle;
import android.support.annotation.Nullable;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import com.wang.canrotor.R;
import com.wang.canrotor.attitudeView.AttitudeView;

public class AttitudeFragment extends Fragment {
    Activity mActivity;
    AttitudeView attitudeView;
    Thread thread;
    @Nullable
    @Override
    public View onCreateView(LayoutInflater inflater, @Nullable ViewGroup container, Bundle savedInstanceState) {
        ViewGroup rootView = (ViewGroup) inflater.inflate(R.layout.fragment_attitude, container, false);

        attitudeView= (AttitudeView) rootView.findViewById(R.id.attitudeView);
        update();

        return rootView;
    }

    private void update() {
        if(thread != null){
            thread.interrupt();
        }

        final Runnable runnable = new Runnable() {
            @Override
            public void run() {
                Bundle bundle = getArguments();
                if (bundle != null) {
                    // Specify the property like layout params
                    attitudeView.setPitch(bundle.getFloat("attitude_pitch")); // Update the Pitch value in degrees.
                    float pitch = attitudeView.getPitch(); // Update get the actual pitch value degrees.
                    attitudeView.setRoll(bundle.getFloat("attitude_roll")); // Update the Roll value in degrees.
                    float roll = attitudeView.getRoll(); // Update get the actual roll value degrees.
                }
            }
        };

        thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(true){
                    mActivity.runOnUiThread(runnable);
                    try{
                        Thread.sleep(20);
                    }catch (InterruptedException ie){
                        ie.printStackTrace();
                    }
                }
            }
        });
        thread.start();
    }


    @Override
    public void onAttach(Context context) {
        super.onAttach(context);

        if(context instanceof Activity){
            mActivity = (Activity) context;
        }
    }

    @Override
    public void onResume() {
        super.onResume();
     }

    @Override
    public void onPause() {
        super.onPause();
        if(thread != null){
            thread.interrupt();
        }
    }

    @Override
    public void onDetach() {
        super.onDetach();
    }

}


