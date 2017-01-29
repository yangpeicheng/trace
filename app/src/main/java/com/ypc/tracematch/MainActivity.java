package com.ypc.tracematch;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import java.util.ArrayList;

public class MainActivity extends AppCompatActivity implements SensorEventListener {
    protected static final float NS2S = 1.0f / 1000000000.0f;
    protected static final float EPSILON = 0.000000001f;
    protected boolean hasInitialOrientation = false;
    protected boolean stateInitializedCalibrated=false;
    private long timestampold=0;
    protected Button startButton,endButton,analysisButton;
    protected CanvasContour canvasContour;
    protected SensorManager sensorManager;
    protected float[] vMagnetic = new float[3];
    protected float[] vAcceleration = new float[3];
    protected float[] vGravity=new float[3];
    protected float[] vGyroscope=new float[3];
    protected float[] rmAccelMag = new float[9];
    protected float[] deltaRotationVector = new float[4];
    protected float[] deltaRotationMatrix=new float[9];
    protected float[] currentRotationMatrix=new float[9];
    protected float[] RotationMatrixFromVector=new float[9];
    protected float[] gyroscopeOrientation=new float[3];
    protected float[] fusedOrientation = new float[3];
    protected float[] rmAccelMagOrientation=new float[3];
    private MeanFilterSmoothing meanFilterAcceleration=new MeanFilterSmoothing();
    private MeanFilterSmoothing meanFilterMagnetic=new MeanFilterSmoothing();
    private MeanFilterSmoothing meanFilterGyroscope=new MeanFilterSmoothing();
    protected Trough trough=new Trough();
    protected LpfFilter accLpf=new LpfFilter();
    protected GetInteralAcc getInteralAcc=new GetInteralAcc();
    protected float[] distance=new float[3];
    protected float[] lastDistance=new float[3];
    protected Handler handler=new Handler(),writeData=new Handler();
    protected WriteCsv writeOrientation=new WriteCsv("a.csv");
    protected DataSegmentation dataSegmentation=new DataSegmentation();
    protected DataSegmentation template1=new DataSegmentation();
    protected DataSegmentation template2=new DataSegmentation();
    protected int count=0;
    protected int troughCount=0;
    protected Turn turn=new Turn();
    protected int turnFlag=0;
    protected int templateNum,templateLen;
    protected DataSegmentation[] templates;
    protected boolean templateInitFlag=false;
    protected Point[][] templatePoints;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        initGui();
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if(event.sensor.getType()==Sensor.TYPE_GRAVITY){
            System.arraycopy(event.values, 0, this.vGravity, 0,
                    this.vGravity.length);
            calculateInitialOrientation();
        }
        if(event.sensor.getType()==Sensor.TYPE_LINEAR_ACCELERATION){
            System.arraycopy(event.values,0,this.vAcceleration,0,this.vAcceleration.length);
            vAcceleration=meanFilterAcceleration.addSamples(vAcceleration);
            vAcceleration=accLpf.HighPassfliter(vAcceleration);
            if(hasInitialOrientation==false)
                return;
            count++;
            if(trough.judge()>0){
                getInteralAcc.Reset();
                if(count-troughCount>1){
                    troughCount=count;
                    if(turn.addSample(fusedOrientation[0]))
                        turnFlag=1;
                }
                if(trough.judge()==2){
                    if(different(distance,lastDistance)){
                        dataSegmentation.addSample(distance[0],distance[1],turnFlag);
                        writeOrientation.writeData(new float[]{distance[0],distance[1],turnFlag});
                        System.arraycopy(distance,0,lastDistance,0,distance.length);
                   }

                    turnFlag=0;
                    //Log.d("dis",String.valueOf(distance[0]));
                }

            }
            else {
                distance=getInteralAcc.calculateDistance(matrixMultiVector(currentRotationMatrix,vAcceleration),event.timestamp);
                handler.post(updateDistanceDisplayTask);
            }

        }
        if(event.sensor.getType()==Sensor.TYPE_MAGNETIC_FIELD){
            System.arraycopy(event.values,0,this.vMagnetic,0,this.vMagnetic.length);
            vMagnetic=meanFilterMagnetic.addSamples(vMagnetic);
        }
        if(event.sensor.getType()==Sensor.TYPE_GYROSCOPE) {
            System.arraycopy(event.values, 0, this.vGyroscope, 0,
                    this.vGyroscope.length);
            this.vGyroscope = meanFilterGyroscope.addSamples(this.vGyroscope);
            onGyroscopeChange(this.vGyroscope, event.timestamp);
        }
        if(event.sensor.getType()==Sensor.TYPE_ROTATION_VECTOR){
            SensorManager.getRotationMatrixFromVector(RotationMatrixFromVector,event.values);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    private void initGui(){
        sensorManager=(SensorManager)getSystemService(SENSOR_SERVICE);
        startButton=(Button)findViewById(R.id.startButton);
        endButton=(Button)findViewById(R.id.endButton);
        canvasContour=(CanvasContour)findViewById(R.id.contour);
        analysisButton=(Button)findViewById(R.id.analysisButton);
        startButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                sensorManager.registerListener(MainActivity.this,sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY),SensorManager.SENSOR_DELAY_GAME);
                sensorManager.registerListener(MainActivity.this,sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION),SensorManager.SENSOR_DELAY_GAME);
                sensorManager.registerListener(MainActivity.this,sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),SensorManager.SENSOR_DELAY_GAME);
                sensorManager.registerListener(MainActivity.this,sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),SensorManager.SENSOR_DELAY_GAME);
            }
        });
        endButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                sensorManager.unregisterListener(MainActivity.this);
                writeOrientation.closeFile();
            }
        });
        analysisButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                dataSegmentation.DataEnd();
                //Point[] points1=new Point[]{new Point(0,0),new Point(3,0),new Point(3,3),new Point(3,0)};
               // Point[] points2=new Point[]{new Point(0,0),new Point(3,0),new Point(3,-3),new Point(6,3)};
                initTemplate();
                if(!templateInitFlag)
                    return;
                if(dataSegmentation.dataLists.size()!=templateLen){
                    Toast.makeText(MainActivity.this,String.format("template error %d vs %d " ,dataSegmentation.dataLists.size(),templateLen),Toast.LENGTH_SHORT).show();
                    return;
                }
                Toast.makeText(MainActivity.this,String.format("%d vs %d",templateNum,templateLen),Toast.LENGTH_SHORT).show();
                for(int j=0;j<dataSegmentation.dataLists.size()-1;j++){
                    //getTemplate(points1[j],points1[j+1],dataSegmentation.dataLists.get(j).size(),template1);
                    //getTemplate(points2[j],points2[j+1],dataSegmentation.dataLists.get(j).size(),template2);
                    int size=dataSegmentation.dataLists.get(j).size();
                    for (int k=0;k<templates.length;k++){
                        getTemplate(templatePoints[k][j],templatePoints[k][j+1],size,templates[k]);
                    }
                }
                for (int k=0;k<templates.length;k++){
                    templates[k].DataEnd();
                }
                //template1.DataEnd();
                //template2.DataEnd();
                int i=0;
               // float dtwDistance1=0.0f,dtwDistance2=0.0f;
                float[] dtwDistance=new float[templates.length];
                Toast.makeText(MainActivity.this,String.valueOf(dataSegmentation.dataLists.size()),Toast.LENGTH_SHORT).show();
                while (true){
                    ArrayList<Point> result=dataSegmentation.CoordinateChange(i);
                    if(result==null)
                        break;
                    //ArrayList<Point> resultTemplate1=template1.CoordinateChange(i);
                    //ArrayList<Point> resultTemplate2=template2.CoordinateChange(i);
                    ArrayList<Point> TemplateResult;
                    for(int k=0;k<templates.length;k++){
                        TemplateResult=templates[k].CoordinateChange(i);
                        dtwDistance[k]+=DTW.DTWDistance(result,TemplateResult);
                    }
                    //dtwDistance1+=DTW.DTWDistance(result,resultTemplate1);
                    //dtwDistance2+=DTW.DTWDistance(result,resultTemplate2);
                    writeCoordinate(result,i);
                   // writeCoordinate(resultTemplate1,i+10);
                    //writeCoordinate(resultTemplate2,i+20);
                    i++;
                }
                int best=0;
                float bestDistance=dtwDistance[0];
                for(int k=1;k<templates.length;k++){
                    if(dtwDistance[k]<bestDistance){
                        best=k;
                        bestDistance=dtwDistance[k];
                    }
                }
                ArrayList<Point> bestPoints=new ArrayList<Point>();
                for(int k=0;k<templates[best].dataLists.size();k++){
                    for(int j=0;j<templates[best].dataLists.get(k).size();j++)
                        bestPoints.add(templates[best].dataLists.get(k).get(j));
                }
                canvasContour.drawMap(bestPoints);
                Toast.makeText(MainActivity.this,String.format("%f vs %f",dtwDistance[0],dtwDistance[1]),Toast.LENGTH_SHORT).show();
            }
        });
    }
    private void onGyroscopeChange(float [] gyroscope,long timestamp){
        if(!hasInitialOrientation)
            return;
        if (!stateInitializedCalibrated)
        {
            System.arraycopy(rmAccelMag,0,currentRotationMatrix,0,rmAccelMag.length);
            stateInitializedCalibrated = true;
        }
        if(timestampold!=0){
            final float dT=(timestamp-timestampold)*NS2S;
            float axisX=gyroscope[0];
            float axisY=gyroscope[1];
            float axisZ=gyroscope[2];
            float magnitude=(float)Math.sqrt(axisX*axisX+axisY*axisY+axisZ*axisZ);
            if(magnitude>EPSILON){
                axisX/=magnitude;
                axisY/=magnitude;
                axisZ/=magnitude;
            }
            float theta=magnitude*dT/2.0f;
            float sinTheta=(float)Math.sin(theta);
            float cosTheta=(float)Math.cos(theta);
            deltaRotationVector[0]=sinTheta*axisX;
            deltaRotationVector[1]=sinTheta*axisY;
            deltaRotationVector[2]=sinTheta*axisZ;
            deltaRotationVector[3]=cosTheta;
            SensorManager.getRotationMatrixFromVector(deltaRotationMatrix,deltaRotationVector);
            currentRotationMatrix=matrixMultiplication(currentRotationMatrix,deltaRotationMatrix);
            SensorManager.getOrientation(currentRotationMatrix,gyroscopeOrientation);
            fusedOrientation=FuseOrientation.fuseOrientation(rmAccelMagOrientation,gyroscopeOrientation);
            currentRotationMatrix = getRotationMatrixFromOrientation(fusedOrientation);
            trough.insert(fusedOrientation[1]);
            writeData.post(updateCsvDataTask);
        }
        timestampold=timestamp;
    }
    private void calculateInitialOrientation()
    {
        hasInitialOrientation =SensorManager.getRotationMatrix(
                rmAccelMag, null, vGravity, vMagnetic);
        SensorManager.getOrientation(rmAccelMag,rmAccelMagOrientation );
    }
    private float[] matrixMultiplication(float[] a, float[] b)
    {
        float[] result = new float[9];

        result[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
        result[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
        result[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];

        result[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
        result[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
        result[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];

        result[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
        result[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
        result[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];

        return result;
    }
    private float[] matrixMultiVector(float[] a, float[] b)
    {
        float[] result = new float[3];
        result[0] = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        result[1] = a[3] * b[0] + a[4] * b[1] + a[5] * b[2];
        result[2] = a[6] * b[0] + a[7] * b[1] + a[8] * b[2];
        return result;
    }
    private float[] getRotationMatrixFromOrientation(float[] o) {
        float[] xM = new float[9];
        float[] yM = new float[9];
        float[] zM = new float[9];

        float sinX = (float)Math.sin(o[1]);
        float cosX = (float)Math.cos(o[1]);
        float sinY = (float)Math.sin(o[2]);
        float cosY = (float)Math.cos(o[2]);
        float sinZ = (float)Math.sin(o[0]);
        float cosZ = (float)Math.cos(o[0]);

        // rotation about x-axis (pitch)
        xM[0] = 1.0f; xM[1] = 0.0f; xM[2] = 0.0f;
        xM[3] = 0.0f; xM[4] = cosX; xM[5] = sinX;
        xM[6] = 0.0f; xM[7] = -sinX; xM[8] = cosX;

        // rotation about y-axis (roll)
        yM[0] = cosY; yM[1] = 0.0f; yM[2] = sinY;
        yM[3] = 0.0f; yM[4] = 1.0f; yM[5] = 0.0f;
        yM[6] = -sinY; yM[7] = 0.0f; yM[8] = cosY;

        // rotation about z-axis (azimuth)
        zM[0] = cosZ; zM[1] = sinZ; zM[2] = 0.0f;
        zM[3] = -sinZ; zM[4] = cosZ; zM[5] = 0.0f;
        zM[6] = 0.0f; zM[7] = 0.0f; zM[8] = 1.0f;

        // rotation order is y, x, z (roll, pitch, azimuth)
        float[] resultMatrix = matrixMultiplication(xM, yM);
        resultMatrix = matrixMultiplication(zM, resultMatrix);
        return resultMatrix;
    }

    public void updateDistanceDisplay() {
       // writeOrientation.writeData(new float[]{1,1});
        //turnFlag=0;
        canvasContour.move(distance[0], -distance[1],0);
    }
    private Runnable updateDistanceDisplayTask = new Runnable() {
        public void run() {
            updateDistanceDisplay();
        }
    };
    public void updateCsvData() {
        //writeOrientation.writeData(new float[]{(float)Math.toDegrees(fusedOrientation[0]),turnFlag,trough.judge()});

    }
    private Runnable updateCsvDataTask = new Runnable() {
        public void run() {
            updateCsvData();
        }
    };
    private void writeCoordinate(ArrayList<Point> s,int index){
        String name=String.format("%d.csv",index);
        WriteCsv writer=new WriteCsv(name);
        for(int i=0;i<s.size();i++)
            writer.writeData(new float[]{s.get(i).x,s.get(i).y});
        writer.closeFile();
    }
    protected boolean different(float [] a,float [] b){
        if(Math.abs(a[0]-b[0])<0.0001f&&Math.abs(a[1]-b[1])<0.0001f){
            return false;
        }
        return true;
    }
    protected void getTemplate(Point start,Point end,int length,DataSegmentation template){
        float avgX=(end.x-start.x)/length;
        float avgY=(end.y-start.y)/length;
        template.addSample(start.x,start.y,1);
        for(int i=1;i<length;i++){
            template.addSample(start.x+i*avgX,start.y+i*avgY,0);
        }
    }
    protected void initTemplate(){
        ReadCsv configFile=new ReadCsv("config.csv");
        if(configFile==null)
            return;
        String[] first=configFile.readNext();
        if(first==null||first.length!=2)
            return;
        templateNum=Integer.parseInt(first[0]);
        templateLen=Integer.parseInt(first[1]);
        if(templateNum==0||templateLen==0)
            return;
        templatePoints=new Point[templateNum][];
        templates=new DataSegmentation[templateNum];
        for(int i=0;i<templateNum;i++){
            templates[i]=new DataSegmentation();
            templatePoints[i]=new Point[templateLen];
            for(int j=0;j<templateLen;j++) {
                String[] data = configFile.readNext();
                if (data.length != 2)
                    return;
                templatePoints[i][j]=new Point(Integer.parseInt(data[0]),Integer.parseInt(data[1]));
            }
        }
        templateInitFlag=true;
    }
}
