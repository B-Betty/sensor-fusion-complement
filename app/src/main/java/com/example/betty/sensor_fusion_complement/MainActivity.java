package com.example.betty.sensor_fusion_complement;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.widget.RadioGroup;
import android.widget.TextView;

import org.w3c.dom.Text;

import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.util.Timer;
import java.util.TimerTask;

public class MainActivity extends AppCompatActivity implements SensorEventListener, RadioGroup.OnCheckedChangeListener{

    private SensorManager mSensorManager = null;
    private float[] gyro = new float[3];
    //陀螺仪的旋转矩阵
    private float[] gyroMatrix = new float[9];
    //陀螺仪的方向角
    private float[] gyroOrientation = new float[3];
    private float[] magnet = new float[3];
    private float[] accel = new float[3];
    private float[] gravity = new float[3];
    private float[] accMagOrientation = new float[3];
    private float[] fusedOrientation = new float[3];
    //加速度计和磁力计的旋转矩阵
    private float[] rotationMatrix = new float[9];
    float[] initMatrix = new float[9];
    public static final float EPSILON = 0.000000001f;
    //纳秒转换为秒
    private static final float NS2S = 1.0f / 1000000000.0f;
    private long timestamp;
    private long timestamp_vel;
    private boolean initState = true;
    public static final int TIME_CONSTANT = 30;
    //滤波系数
    public static final float FILTER_COEFFICIENT = 0.98f;
    private Timer fuseTimer = new Timer();
    public Handler mHandler;
    private RadioGroup mRadioGroup;
    private TextView mAzimuthView;
    private TextView mPitchView;
    private TextView mRollView;
    private int radioSelection;
    private TextView tv;
    DecimalFormat d = new DecimalFormat("#.##");//对数值的格式化方法
    float[] velocity = new float[3];
    float[] position = new float[3];

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        gyroOrientation[0] = 0.0f;
        gyroOrientation[1] = 0.0f;
        gyroOrientation[2] = 0.0f;

        gyroMatrix[0] = 1.0f; gyroMatrix[1] = 0.0f; gyroMatrix[2] = 0.0f;
        gyroMatrix[3] = 0.0f; gyroMatrix[4] = 1.0f; gyroMatrix[5] = 0.0f;
        gyroMatrix[6] = 0.0f; gyroMatrix[7] = 0.0f; gyroMatrix[8] = 1.0f;

        mSensorManager = (SensorManager) this.getSystemService(SENSOR_SERVICE);
        //注册传感器监听事件
        initListeners();
        //延时一秒计算合成的方向角
        fuseTimer.scheduleAtFixedRate(new calculateFusedOrientationTask(),1000, TIME_CONSTANT);

        mHandler = new Handler();
        radioSelection = 0;
        //向最接近数字方向舍入的舍入模式
        d.setRoundingMode(RoundingMode.HALF_UP);
        //整数部分的最大位数3
        d.setMaximumFractionDigits(3);
        //整数部分最小的位数为3
        d.setMinimumFractionDigits(3);
        mRadioGroup = (RadioGroup)findViewById(R.id.radioGroup1);
        mAzimuthView = (TextView)findViewById(R.id.textView4);
        mPitchView = (TextView)findViewById(R.id.textView5);
        mRollView = (TextView)findViewById(R.id.textView6);
        mRadioGroup.setOnCheckedChangeListener(this);
        tv = (TextView) findViewById(R.id.tv);
    }
    @Override
    public void onStop() {
        super.onStop();
        mSensorManager.unregisterListener(this);
    }

    @Override
    protected void onPause() {
        super.onPause();
        mSensorManager.unregisterListener(this);
    }

    @Override
    public void onResume() {
        super.onResume();
        initListeners();
    }

    public void initListeners(){
        mSensorManager.registerListener(this,mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION),20000000);
        mSensorManager.registerListener(this,mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),20000000);
        mSensorManager.registerListener(this,mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),20000000);
        mSensorManager.registerListener(this,mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY),20000000);
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }
    @Override
    public void onSensorChanged(SensorEvent event) {
        switch(event.sensor.getType()) {

            case Sensor.TYPE_LINEAR_ACCELERATION:
                System.arraycopy(event.values, 0, accel, 0, 3);
                calculateAccMagOrientation();
                 getvelocityandposition(event);
                break;
            case Sensor.TYPE_GYROSCOPE:
                gyroFunction(event);
                break;
            case Sensor.TYPE_MAGNETIC_FIELD:
                System.arraycopy(event.values, 0, magnet, 0, 3);
                break;
            case Sensor.TYPE_GRAVITY:
                System.arraycopy(event.values, 0, gravity, 0, 3);
                break;
        }
    }

    //计算加速度计和磁力计得到的方向角---accMagOrientation
    public void calculateAccMagOrientation() {
        if(SensorManager.getRotationMatrix(rotationMatrix, null, accel, magnet)) {
            SensorManager.getOrientation(rotationMatrix, accMagOrientation);
        }
    }

    /**
     *  从角速度获得旋转向量（四元素）
     * @param gyroValues 陀螺仪值
     * @param deltaRotationVector 四元素
     * @param timeFactor 时间差
     */
    private void getRotationVectorFromGyro(float[] gyroValues, float[] deltaRotationVector, float timeFactor)
    {
        float[] normValues = new float[3];

        //归一化
        float omegaMagnitude = (float)Math.sqrt(gyroValues[0] * gyroValues[0] + gyroValues[1] * gyroValues[1] + gyroValues[2] * gyroValues[2]);
        if(omegaMagnitude > EPSILON) {
            normValues[0] = gyroValues[0] / omegaMagnitude;  //x,y,z 轴的单位矢量
            normValues[1] = gyroValues[1] / omegaMagnitude;
            normValues[2] = gyroValues[2] / omegaMagnitude;
        }
        //由角速度对时间积分得到角度
        float thetaOverTwo = omegaMagnitude * timeFactor;
        float sinThetaOverTwo = (float)Math.sin(thetaOverTwo);
        float cosThetaOverTwo = (float)Math.cos(thetaOverTwo);
        //获取四元素
        deltaRotationVector[0] = sinThetaOverTwo * normValues[0];  //x
        deltaRotationVector[1] = sinThetaOverTwo * normValues[1];  //y
        deltaRotationVector[2] = sinThetaOverTwo * normValues[2];  //z
        deltaRotationVector[3] = cosThetaOverTwo;                  //w
    }

    // This function performs the integration of the gyroscope data.
    // It writes the gyroscope based orientation into gyroOrientation.
    public void gyroFunction(SensorEvent event) {
        if (accMagOrientation == null)
            return;
        if(initState) {
            //从加速度计和磁力计的方向矢量((欧拉角)得到旋转矩阵
            initMatrix = getRotationMatrixFromOrientation(accMagOrientation);
            float[] test = new float[3];
            //test为陀螺仪得到的方向矢量 欧拉角
            SensorManager.getOrientation(initMatrix, test);
            gyroMatrix = matrixMultiplication(gyroMatrix, initMatrix);
            initState = false;
        }

        float[] deltaVector = new float[4];
        if(timestamp != 0) {
            //以秒为单位的时间差
            final float dT = (float) ((event.timestamp - timestamp) * NS2S);
            System.arraycopy(event.values, 0, gyro, 0, 3);
            //从角速度获得旋转向量 deltaVector[4]
            getRotationVectorFromGyro(gyro, deltaVector, dT / 2.0f);
        }
        timestamp = (int) event.timestamp;
        //从旋转向量（四元素）转换为旋转矩阵
        float[] deltaMatrix = new float[9];
        SensorManager.getRotationMatrixFromVector(deltaMatrix, deltaVector);

        gyroMatrix = matrixMultiplication(gyroMatrix, deltaMatrix);

        //从陀螺仪的旋转矩阵获得陀螺仪的方向矢量 欧拉角
        SensorManager.getOrientation(gyroMatrix, gyroOrientation);
    }

    /**
     * 由方向角得到旋转矩阵
     * @param o
     * @return
     */
    private float[] getRotationMatrixFromOrientation(float[] o) {  // o[0]表示 Z 方向角，o[1]表示 X 方向角，o[2]表示 Y 方向角
        float[] xM = new float[9];
        float[] yM = new float[9];
        float[] zM = new float[9];

        float sinX = (float)Math.sin(o[1]);
        float cosX = (float)Math.cos(o[1]);
        float sinY = (float)Math.sin(o[2]);
        float cosY = (float)Math.cos(o[2]);
        float sinZ = (float)Math.sin(o[0]);
        float cosZ = (float)Math.cos(o[0]);

        // x-axis (pitch)  绕 X 轴的旋转矩阵
        // 1    0    0
        // 0   cos   sin
        // 0 - sin   cos
        xM[0] = 1.0f; xM[1] = 0.0f;  xM[2] = 0.0f;
        xM[3] = 0.0f; xM[4] = cosX;  xM[5] = sinX;
        xM[6] = 0.0f; xM[7] = -sinX; xM[8] = cosX;

        // y-axis (roll)  绕 Y 轴的旋转矩阵
        yM[0] = cosY;  yM[1] = 0.0f; yM[2] = sinY;
        yM[3] = 0.0f;  yM[4] = 1.0f; yM[5] = 0.0f;
        yM[6] = -sinY; yM[7] = 0.0f; yM[8] = cosY;

        //  z-axis (azimuth)  绕 Z 轴的旋转矩阵
        zM[0] = cosZ;  zM[1] = sinZ; zM[2] = 0.0f;
        zM[3] = -sinZ; zM[4] = cosZ; zM[5] = 0.0f;
        zM[6] = 0.0f;  zM[7] = 0.0f; zM[8] = 1.0f;

        // 旋转顺序： y, x, z (roll, pitch, azimuth)
        float[] resultMatrix = matrixMultiplication(xM, yM);
        resultMatrix = matrixMultiplication(zM, resultMatrix);
        return resultMatrix;
    }

    /**
     *
     * @param A
     * @param B
     * @return  矩阵 A * B
     */
    private float[] matrixMultiplication(float[] A, float[] B) {
        float[] result = new float[9];

        result[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
        result[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
        result[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

        result[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
        result[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
        result[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

        result[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
        result[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
        result[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];

        return result;
    }

    class calculateFusedOrientationTask extends TimerTask {
        public void run() {
            float oneMinusCoeff = 1.0f - FILTER_COEFFICIENT;

            /*
             * Fix for 179? <--> -179? transition problem:
             * Check whether one of the two orientation angles (gyro or accMag) is negative while the other one is positive.
             * If so, add 360? (2 * math.PI) to the negative value, perform the sensor fusion, and remove the 360? from the result
             * if it is greater than 180?. This stabilizes the output in positive-to-negative-transition cases.
             */

            // azimuth 方位角
            if (gyroOrientation[0] < -0.5 * Math.PI && accMagOrientation[0] > 0.0) {
                fusedOrientation[0] = (float) (FILTER_COEFFICIENT * (gyroOrientation[0] + 2.0 * Math.PI) + oneMinusCoeff * accMagOrientation[0]);
                fusedOrientation[0] -= (fusedOrientation[0] > Math.PI) ? 2.0 * Math.PI : 0;
            }
            else if (accMagOrientation[0] < -0.5 * Math.PI && gyroOrientation[0] > 0.0) {
                fusedOrientation[0] = (float) (FILTER_COEFFICIENT * gyroOrientation[0] + oneMinusCoeff * (accMagOrientation[0] + 2.0 * Math.PI));
                fusedOrientation[0] -= (fusedOrientation[0] > Math.PI)? 2.0 * Math.PI : 0;
            }
            else {
                fusedOrientation[0] = FILTER_COEFFICIENT * gyroOrientation[0] + oneMinusCoeff * accMagOrientation[0];
            }

            // pitch
            if (gyroOrientation[1] < -0.5 * Math.PI && accMagOrientation[1] > 0.0) {
                fusedOrientation[1] = (float) (FILTER_COEFFICIENT * (gyroOrientation[1] + 2.0 * Math.PI) + oneMinusCoeff * accMagOrientation[1]);
                fusedOrientation[1] -= (fusedOrientation[1] > Math.PI) ? 2.0 * Math.PI : 0;
            }
            else if (accMagOrientation[1] < -0.5 * Math.PI && gyroOrientation[1] > 0.0) {
                fusedOrientation[1] = (float) (FILTER_COEFFICIENT * gyroOrientation[1] + oneMinusCoeff * (accMagOrientation[1] + 2.0 * Math.PI));
                fusedOrientation[1] -= (fusedOrientation[1] > Math.PI)? 2.0 * Math.PI : 0;
            }
            else {
                fusedOrientation[1] = FILTER_COEFFICIENT * gyroOrientation[1] + oneMinusCoeff * accMagOrientation[1];
            }

            // roll
            if (gyroOrientation[2] < -0.5 * Math.PI && accMagOrientation[2] > 0.0) {
                fusedOrientation[2] = (float) (FILTER_COEFFICIENT * (gyroOrientation[2] + 2.0 * Math.PI) + oneMinusCoeff * accMagOrientation[2]);
                fusedOrientation[2] -= (fusedOrientation[2] > Math.PI) ? 2.0 * Math.PI : 0;
            }
            else if (accMagOrientation[2] < -0.5 * Math.PI && gyroOrientation[2] > 0.0) {
                fusedOrientation[2] = (float) (FILTER_COEFFICIENT * gyroOrientation[2] + oneMinusCoeff * (accMagOrientation[2] + 2.0 * Math.PI));
                fusedOrientation[2] -= (fusedOrientation[2] > Math.PI)? 2.0 * Math.PI : 0;
            }
            else {
                fusedOrientation[2] = FILTER_COEFFICIENT * gyroOrientation[2] + oneMinusCoeff * accMagOrientation[2];
            }

            // 补偿陀螺仪漂移，
            gyroMatrix = getRotationMatrixFromOrientation(fusedOrientation);
            System.arraycopy(fusedOrientation, 0, gyroOrientation, 0, 3);

            // update sensor output in GUI
            mHandler.post(updateOrentationDisplayTask);
        }
    }


    // **************************** GUI FUNCTIONS *********************************

    @Override
    public void onCheckedChanged(RadioGroup group, int checkedId) {
        switch(checkedId) {
            case R.id.radio0:
                radioSelection = 0;
                break;
            case R.id.radio1:
                radioSelection = 1;
                break;
            case R.id.radio2:
                radioSelection = 2;
                break;
        }
    }

    public void updateOreintationDisplay() {
        switch(radioSelection) {
            case 0:
                mAzimuthView.setText(d.format(accMagOrientation[0] ));//转换为度单位  * 180/Math.PI
                mPitchView.setText(d.format(accMagOrientation[1]));
                mRollView.setText(d.format(accMagOrientation[2]));
                break;
            case 1:
                mAzimuthView.setText(d.format(gyroOrientation[0]));
                mPitchView.setText(d.format(gyroOrientation[1] ));
                mRollView.setText(d.format(gyroOrientation[2] ));
                break;
            case 2:
                mAzimuthView.setText(d.format(fusedOrientation[0] ));
                mPitchView.setText(d.format(fusedOrientation[1] ));
                mRollView.setText(d.format(fusedOrientation[2] ));
                break;
        }
    }

    /**
     * 从融合后的欧拉角得到四元素
     * @param Z
     * @param X
     * @param Y
     * @return
     */
    public float[]  eulerAnglesToQuaternion(float Z,float X,float Y){
        float[] q = new float[4];
        float sinX = (float)Math.sin(X * 0.5f);
        float cosX = (float)Math.cos(X * 0.5f);
        float sinY = (float)Math.sin(Y * 0.5f);
        float cosY = (float)Math.cos(Y * 0.5f);
        float sinZ = (float)Math.sin(Z * 0.5f);
        float cosZ = (float)Math.cos(Z * 0.5f);
        q[0] = cosZ * cosX * cosY + sinZ * sinX * sinY;
        q[1] = sinZ * cosX * cosY - cosZ * sinX * sinY;
        q[2] = cosZ * sinX * cosY + sinZ * cosX * sinY;
        q[3] = cosZ * cosX * sinY - sinZ * sinX * cosY;
        return q;
    }

    private Runnable updateOrentationDisplayTask = new Runnable() {
        public void run() {
            updateOreintationDisplay();
            float hdg_Z =   (float) (fusedOrientation[0] * 180/Math.PI);
            float pitch_X = (float) (fusedOrientation[1] * 180/Math.PI);
            float roll_Y =  (float) (fusedOrientation[2] * 180/Math.PI);
            float[] q1 = eulerAnglesToQuaternion(hdg_Z,pitch_X,roll_Y);
            tv.setText("IMU融合的四元素为 :  " + d.format(q1[0]) + "+  "+ d.format(q1[1]) +" * i +  "+ d.format(q1[2]) +" * j +  "+ d.format(q1[3]) +" * k ");
        }
    };

    /**
     * 获取当前的速度和位置
     * @param event
     */
    private void getvelocityandposition(SensorEvent event) {
        float[] linear_acceleration = new float[3];
        float[] fusedRotationMatrix = new float[9];
            if (timestamp_vel != 0) {
            final float dT =  ((event.timestamp - timestamp_vel) * NS2S); //以秒为单位的时间差
            Log.i("dT","时间: " + dT);
            //TYPE_ACCELEROMETER加速度计获取的值
//            linear_acceleration[0] = initMatrix[0] * accel[0] + initMatrix[1] * accel[1] + initMatrix[2] * accel[2] - gravity[0];
//            linear_acceleration[1] = initMatrix[3] * accel[0] + initMatrix[4] * accel[1] + initMatrix[5] * accel[2] - gravity[1];
//            linear_acceleration[2] = initMatrix[6] * accel[0] + initMatrix[7] * accel[1] + initMatrix[8] * accel[2] - gravity[2];
            //线性加速度获取的值
            linear_acceleration[0] = initMatrix[0] * accel[0] + initMatrix[1] * accel[1] + initMatrix[2] * accel[2];
            linear_acceleration[1] = initMatrix[3] * accel[0] + initMatrix[4] * accel[1] + initMatrix[5] * accel[2];
            linear_acceleration[2] = initMatrix[6] * accel[0] + initMatrix[7] * accel[1] + initMatrix[8] * accel[2];
            velocity[0] = velocity[0] + linear_acceleration[0] * dT;
            velocity[1] = velocity[1] + linear_acceleration[1] * dT;
            velocity[2] = velocity[2] + linear_acceleration[2] * dT;

            position[0] = position[0] + velocity[0] * dT;
            position[1] = position[1] + velocity[1] * dT;
            position[2] = position[2] + velocity[2] * dT;
        }
         timestamp_vel = event.timestamp;

          Log.i("***速度***","X轴速度"+velocity[0]+"\nY轴速度"+velocity[1]+"\nZ轴速度"+velocity[2]);
          Log.i("***位置***","X轴位置"+position[0]+"\nY轴位置"+position[1]+"\nZ轴位置"+position[2]);
    }

}
