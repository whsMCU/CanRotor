package com.wang.canrotor.activity;

import android.app.Activity;
import android.content.Context;
import android.graphics.Color;
import android.os.Bundle;
import android.support.annotation.Nullable;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import com.github.mikephil.charting.charts.LineChart;
import com.github.mikephil.charting.components.Description;
import com.github.mikephil.charting.components.Legend;
import com.github.mikephil.charting.components.XAxis;
import com.github.mikephil.charting.components.YAxis;
import com.github.mikephil.charting.data.Entry;
import com.github.mikephil.charting.data.LineData;
import com.github.mikephil.charting.data.LineDataSet;
import com.github.mikephil.charting.interfaces.datasets.ILineDataSet;
import com.github.mikephil.charting.utils.ColorTemplate;
import com.wang.canrotor.R;

import java.util.ArrayList;


public class ChartFragment extends Fragment {
    Activity mActivity;
    LineChart mpLineChart;
    private Thread thread;
    @Nullable
    @Override
    public View onCreateView(LayoutInflater inflater, @Nullable ViewGroup container, Bundle savedInstanceState) {
        ViewGroup rootView = (ViewGroup) inflater.inflate(R.layout.fragment_chart, container, false);

        mpLineChart = (LineChart) rootView.findViewById(R.id.line_chart);
        Description des = mpLineChart.getDescription();

//        LineDataSet lineDataSet1 = new LineDataSet(dataValues1(), "Data Set 1");
//        ArrayList<ILineDataSet> dataSets = new ArrayList<>();
//
//        dataSets.add(lineDataSet1);
//
//        LineData data = new LineData(dataSets);
//        mpLineChart.setData(data);
//        mpLineChart.invalidate();
        des.setEnabled(true);
        des.setText("Real_Time Data");
        des.setTextSize(15f);
        des.setTextColor(Color.WHITE);
        mpLineChart.setTouchEnabled(true);
        mpLineChart.setDragEnabled(true);
        mpLineChart.setScaleEnabled(true);
        mpLineChart.setDrawGridBackground(true);

        // if disabled, scaling can be done on x- and y-axis separately
        mpLineChart.setPinchZoom(true);

        // set an alternative background color
        mpLineChart.setBackgroundColor(Color.LTGRAY);

        XAxis xAxis = mpLineChart.getXAxis();
        xAxis.setPosition(XAxis.XAxisPosition.BOTTOM);
        xAxis.setTextSize(10f);
        xAxis.setDrawGridLines(false);
        xAxis.setAvoidFirstLastClipping(true);
        xAxis.setEnabled(true);

        YAxis leftAxis = mpLineChart.getAxisLeft();
        leftAxis.setDrawGridLines(false);
        
        YAxis rightAxis = mpLineChart.getAxisRight();
        rightAxis.setEnabled(false);
        
        LineData data = new LineData();
        data.setValueTextColor(Color.CYAN);
        mpLineChart.setData(data);
        
        feedMultiple();

        return rootView;
    }

    private void addEntry() {
        LineData data = mpLineChart.getData();
        if(data != null){
            ILineDataSet set = data.getDataSetByIndex(0);
            if(set == null){
                set = createSet();
                data.addDataSet(set);
            }
            Bundle bundle = getArguments();
            if (bundle != null) {
            //data.addEntry(new Entry(set.getEntryCount(), (float)(Math.random() * 40) + 30f), 0);
            data.addEntry(new Entry(set.getEntryCount(), (float)bundle.getFloat("attitude_roll"),0), 0);
            //data.addEntry(new Entry(set.getEntryCount(), (float)bundle.getFloat("attitude_pitch"),0), 1);
            data.notifyDataChanged();
            }

            mpLineChart.notifyDataSetChanged();
            mpLineChart.setVisibleXRangeMaximum(100);
            mpLineChart.moveViewToX(data.getEntryCount());

        }
    }

    private ILineDataSet createSet() {
        LineDataSet set = new LineDataSet(null, "Dynamic Data");
        set.setAxisDependency(YAxis.AxisDependency.LEFT);
        set.setColor(ColorTemplate.getHoloBlue());
        set.setCircleColor(Color.WHITE);
        set.setLineWidth(2f);
        set.setCircleRadius(1f);
        set.setFillAlpha(65);
        set.setFillColor(ColorTemplate.getHoloBlue());
        set.setHighLightColor(Color.rgb(244,177,177));
        set.setValueTextColor(Color.WHITE);
        set.setValueTextSize(9f);
        set.setDrawValues(false);
        return set;
    }

    private void feedMultiple() {
        if(thread != null){
            thread.interrupt();
        }

        final Runnable runnable = new Runnable() {
            @Override
            public void run() {
                addEntry();
            }
        };

        thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(true){
                    mActivity.runOnUiThread(runnable);
                    try{
                        Thread.sleep(100);
                    }catch (InterruptedException ie){
                        ie.printStackTrace();
                    }
                }
            }
        });
        thread.start();
    }

    private ArrayList<Entry> dataValues1(){
        ArrayList<Entry> dataVals = new ArrayList<Entry>();
        dataVals.add(new Entry(0,20));
        dataVals.add(new Entry(1,24));
        dataVals.add(new Entry(2,2));
        dataVals.add(new Entry(3,10));
        dataVals.add(new Entry(4,28));

        return dataVals;
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


