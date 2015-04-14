package com.example.acceldatarecorderphone;

import jama.Matrix;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.LinkedList;
import java.util.Locale;
import java.util.Queue;
import java.util.Timer;
import java.util.TimerTask;

import jkalman.JKalman;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.ProgressDialog;
import android.content.Intent;
import android.content.res.Resources;
import android.content.res.TypedArray;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;
import android.os.PowerManager;
import android.util.Log;
import android.view.MenuItem;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.example.acceldatarecorderphone.Catagories.SensorLocation;

public class AccelDataRecorderPhone extends Activity implements SensorEventListener, OnClickListener
{
	private StringBuilder fileNamer = new StringBuilder();
	private SensorManager mSensorManager;
	private PowerManager mPowerManager;
	private Button mStartButton;
	private Button mStopButton;
	private TextView mPlacementView;
	private TextView mRecordingStatusView;
	private boolean mIsTracking;
	private long mtimestamp;
	private boolean mExistingFile;
	private boolean mCheckEmptyName;
	public static final int MAX_QUEUE_CAPACITY = 300;
	public boolean recoredTracer = false;
	private int numAccelSamples;
	private int numGyroSamples;
	private int numMagnetoSamples;
	private int numLinAccelSamples;
	private Date startTime;
	private Date endTime;
	private String str1;
	private SensorLocation location = SensorLocation.NOT_DEFINED;
	private float [] gyro = new float [3];
	private float [] accel = new float [3];
	private float [] magneto = new float [3];
	private float [] orientation = new float [3];
	private float [] linAccel = new float [3];
    // accelerometer and magnetometer based rotation matrix
    private float[] rotationMatrix = new float[9];
    private float[] accMagOrientation = new float[3];
    // rotation matrix from gyro data
    private float[] gyroMatrix = new float[9];
    // orientation angles from gyro matrix
    private float[] gyroOrientation = new float[3];
    private float[] fusedOrientation = new float[3];
    private float[] finalFusedOrientation = new float[3];
    private float timestamp;
	private boolean initState = true;
	private PowerManager.WakeLock lock;
	 public static final float EPSILON = 0.000000001f;
	    private static final float NS2S = 1.0f / 1000000000.0f;
    
	public static final int TIME_CONSTANT = 30;
	public static final float FILTER_COEFFICIENT = 0.98f;
	private Timer fuseTimer = new Timer();
	
	private String mFilename;
	private StringBuilder mNowDDMMYYYY;
	private SimpleDateFormat logFormat = new SimpleDateFormat("hh:mm:ss:SSS");
	
	private Queue<SensorModelXYZ> mAccData;
	private Queue<SensorModelXYZ> mGyroData;
	private Queue<SensorModelXYZ> mMagnetoData;
	private Queue<SensorModelXYZ> mKalmanAccData;
	private Queue<SensorModelXYZ> mKalmanGyroData;
	private Queue<SensorModelXYZ> mKalmanMagnetoData;
	private Queue<SensorModelXYZ> finalFusedData;
	private Queue<SensorModelXYZ> mLinAccData;
	DecimalFormat d = new DecimalFormat("#.##");
	
	private static Matrix mS;
	private static Matrix mC;
	private static Matrix mM;
	private static JKalman mKalman;
	


	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_accel_data_recorder_phone);
		
		
	    mStartButton = (Button)findViewById(R.id.button1);
	    mStopButton = (Button)findViewById(R.id.button2);
	    mPlacementView = (TextView)findViewById(R.id.PlacementView);
	    mRecordingStatusView= (TextView)findViewById(R.id.recordingStatusView);
	    mPlacementView.setTextColor(Color.RED);
	    
	    
	    Resources resources = getResources();
	    TypedArray preSets = resources.obtainTypedArray(R.array.Pre_Sets);
	    TypedArray locations = resources.obtainTypedArray(R.array.sensor_locations);
	    String locationStr = locations.getString(preSets.getInt(1,0));
	    mPlacementView.setText(locationStr);
	    //displayLocation(mFilename);
		// get sensorManager and initialise sensor listeners
		mSensorManager = (SensorManager) this.getSystemService(SENSOR_SERVICE);
		mStartButton.setOnClickListener(this);
		mStopButton.setOnClickListener(this);
		
		//Power Manager
		
		mPowerManager = (PowerManager) getSystemService(POWER_SERVICE);
		lock = mPowerManager.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "SensorRead");
		lock.acquire();
		/////////////////// 
		mAccData = new LinkedList<SensorModelXYZ>();
		mLinAccData = new LinkedList<SensorModelXYZ>();
		mMagnetoData = new LinkedList<SensorModelXYZ>();
		mGyroData = new LinkedList<SensorModelXYZ>();
		mKalmanAccData = new LinkedList<SensorModelXYZ>();
		mKalmanGyroData = new LinkedList<SensorModelXYZ>();
		finalFusedData = new LinkedList<SensorModelXYZ>();
		
		/////////

		/////////
		mIsTracking = false;
		initListeners(); 
		
		   gyroOrientation[0] = 0.0f;
	       gyroOrientation[1] = 0.0f;
	       gyroOrientation[2] = 0.0f;
		
	    fuseTimer.scheduleAtFixedRate(new calculateFusedOrientationTask(),1000, TIME_CONSTANT);
		// initialise gyroMatrix with identity matrix
        gyroMatrix[0] = 1.0f; gyroMatrix[1] = 0.0f; gyroMatrix[2] = 0.0f;
        gyroMatrix[3] = 0.0f; gyroMatrix[4] = 1.0f; gyroMatrix[5] = 0.0f;
        gyroMatrix[6] = 0.0f; gyroMatrix[7] = 0.0f; gyroMatrix[8] = 1.0f;
		
		try {
			mKalman = new JKalman(6, 3);

//			Random rand = new Random(System.currentTimeMillis() % 2011);
			double x = 0;
			double y = 0;
			double z = 0;
			// constant velocity
//			double dx = rand.nextDouble(); // da/dt //a =9.81m/sÂ²
//			double dy = rand.nextDouble();
//			double dz = rand.nextDouble();

			mS = new Matrix(9, 1);
			mC = new Matrix(9, 1);

			mM = new Matrix(3, 1);
			mM.set(0, 0, x);
			mM.set(1, 0, y);
			mM.set(2, 0, z);

			// transitions for x, y, dx, dy,
			/*
			 * double[][] tr = { {1, 0, 1, 0}, {0, 1, 0, 1}, {0, 0, 1, 0}, {0,
			 * 0, 0, 1} };
			 */
			// transitions for x, y, z, dx, dy, dz,
			double[][] tr = { { 1, 0, 0, 1, 0, 0 }, { 0, 1, 0, 0, 1, 0 },
					{ 0, 0, 1, 0, 0, 1 }, { 0, 0, 0, 1, 0, 0 },
					{ 0, 0, 0, 0, 1, 0 }, { 0, 0, 0, 0, 0, 1 } };
			mKalman.setTransition_matrix(new Matrix(tr));

			// 1s somewhere?
			mKalman.setError_cov_post(mKalman.getError_cov_post().identity());

			// init first assumption similar to first observation (cheat :)
			// kalman.setState_post(kalman.getState_post());

			// report what happend first :)
			// System.out.println("first x:" + x + ", y:" + y +
			// ", z: "+z+", dx:" + dx + ", dy:" + dy+", dz"+dz);
			// System.out.println("no; x; y; dx; dy; predictionX; predictionY; predictionDx; predictionDy; correctionX; correctionY; correctionDx; correctionDy;");

			// For debug only
			/*
			 * for (int i = 0; i < 200; ++i) {
			 * 
			 * calcAndPrintValues(mKalman, rand, i);
			 * 
			 * }
			 */
		} catch (Exception ex) {
			System.out.println(ex.getMessage());
		}
	}
		

	protected void onActivityResult(int requestCode, int resultCode, Intent data){
		Bundle bundle = data.getBundleExtra(Settings.DATA_KEY);
		int pid = bundle.getInt(Settings.PID_KEY);
		int sensorLoc = bundle.getInt(Settings.LOC_KEY);
		int shoeType = bundle.getInt(Settings.SHOE_KEY);
		int lowerType = bundle.getInt(Settings.LOWER_KEY);
		int surfaceType = bundle.getInt(Settings.SURFACE_KEY);
		int walkType = bundle.getInt(Settings.WALK_KEY);
		int sessionID = bundle.getInt(Settings.SESSION_KEY);
		
		
	    //////////////////// This code is to get back String names ////////////////////////////
		// Make object of XML and use that object to return the String names 
		//Resources resources = getResources();
		//TypedArray sensorLocation = resources.obtainTypedArray(R.array.sensor_locations);
	    //String locationStr = sensorLocation.getString(sensorLoc-1);
	    //////////////////////////////////////////////////////////////////////////////////////
		
		fileNamer.append(getTwoDigitRepresentation(pid));
		fileNamer.append("_");
		fileNamer.append(getTwoDigitRepresentation(shoeType));
		fileNamer.append("_");
		fileNamer.append(getTwoDigitRepresentation(lowerType));
		fileNamer.append("_");
		fileNamer.append(getTwoDigitRepresentation(sensorLoc));
		fileNamer.append("_");
		fileNamer.append(getTwoDigitRepresentation(surfaceType));
		fileNamer.append("_");
		fileNamer.append(getTwoDigitRepresentation(walkType));
		fileNamer.append("_");
		fileNamer.append(getTwoDigitRepresentation(sessionID));
		
		Log.e("read Data",fileNamer.toString());
		String filename;
		filename = fileNamer.toString();
		 Calendar c = Calendar.getInstance();
		 SimpleDateFormat df = new SimpleDateFormat("HH-mm-ss");
		 String myDate = df.format(c.getTime());
		//String myDate = java.text.DateFormat.getDateTimeInstance().format(Calendar.getInstance().getTime());
		String filenameDate = myDate+"_"+filename;
		mFilename = filenameDate;
		
		//Log.e("fileName:",""+filename);
		Log.i("Your File name", mFilename);
		   mIsTracking = true;
		    if (mFilename != null && !mFilename.isEmpty()){
		    recoredTracer = true;
		    }
		fileNamer = new StringBuilder();
		
		Resources resources = getResources();
	    TypedArray locations = resources.obtainTypedArray(R.array.sensor_locations);
	    String locationStr = locations.getString(sensorLoc);
	    TypedArray persons = resources.obtainTypedArray(R.array.person_id);
	    String personStr = persons.getString(pid);
		mPlacementView.setText(locationStr);
		mRecordingStatusView.setText("Recording for PersonID: "+personStr);
	}
	
	private String getTwoDigitRepresentation(int _num)
	{
		StringBuilder res= new StringBuilder();
		if(_num<10)
		{
			res.append("0");
		}
		res.append(_num);
		return res.toString();
	}
	/*@Override
	public boolean onCreateOptionsMenu(android.view.Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		MenuInflater menuInflater = getMenuInflater();
		menuInflater.inflate(R.menu.accel_data_recorder_phone, menu);
		return true;
	}*/
	


 /* private void displayLocation(String filename)
  {
	  if (mIsTracking == true){
		  
		  //mTextView1.setText(location.name()); 
		  mRecordingStatusView.setText("Active Recording:"+filename);
	  }
	  else {
		  mRecordingStatusView.setText("Not Active Recording");
	  }
		  
  }*/
	private void initListeners() {
		// TODO Auto-generated method stub
		mSensorManager.registerListener(this,
				mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),0);
				//SensorManager.SENSOR_DELAY_FASTEST);
		mSensorManager.registerListener(this,
				mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),0);
				//SensorManager.SENSOR_DELAY_FASTEST);
		
		mSensorManager.registerListener(this,
				mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),10000);
				//SensorManager.SENSOR_DELAY_NORMAL);
		mSensorManager.registerListener(this, 
				mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION),
				SensorManager.SENSOR_DELAY_FASTEST);
		
	}
	

	public void onSensorChanged(SensorEvent event) {
    	
		if (startTime==null){
			startTime = new Date(); 
		}
		
    	// TODO Auto-generated method stub
    	mtimestamp = (event.timestamp)/1000000000; // Convert Nano Seconds to Seconds
    	
    	switch (event.sensor.getType()) {
    		case Sensor.TYPE_ACCELEROMETER:
    			numAccelSamples++;
    			float [] tempKalmanAccel = new float[3];
    			System.arraycopy(event.values, 0, accel, 0, 3);
    			SensorModelXYZ entry = new SensorModelXYZ(mtimestamp, accel[0], accel[1], accel[2] );		
    			tempKalmanAccel = calcAndPrintValues(mKalman, accel[0], accel[1], accel[2]);
    			SensorModelXYZ entryKal = new SensorModelXYZ(mtimestamp, tempKalmanAccel[0], tempKalmanAccel[1], tempKalmanAccel[2]);
    			if (mIsTracking && recoredTracer){
    				mAccData.add(entry); 	
    				mKalmanAccData.add(entryKal);
    				
    			}
    			
    			break; 
    		
    		case Sensor.TYPE_GYROSCOPE:
    			numGyroSamples ++;
    			float [] tempKalmanGyro = new float[3];
    			System.arraycopy(event.values, 0, gyro, 0, 3);
    			SensorModelXYZ entry1 = new SensorModelXYZ(mtimestamp, gyro[0],gyro[1],gyro[2] );
    			tempKalmanGyro = calcAndPrintValues(mKalman, gyro[0], gyro[1], gyro[2]);
    			SensorModelXYZ entry1Kal = new SensorModelXYZ(mtimestamp, tempKalmanGyro[0], tempKalmanGyro[1], tempKalmanGyro[2]);
    			if (mIsTracking && recoredTracer){
    				mGyroData.add(entry1);
    				mKalmanGyroData.add(entry1Kal);
    			}
    			
    		case Sensor.TYPE_MAGNETIC_FIELD:
    			numMagnetoSamples++;
    	//		float [] tempKalmanMagneto = new float [3];
    			System.arraycopy(event.values, 0, magneto, 0, 3);
    			SensorModelXYZ entry2 = new SensorModelXYZ(mtimestamp, magneto[0],magneto[1],magneto[2] );
    	//		tempKalmanMagneto = calcAndPrintValues(mKalman, magneto[0], magneto[1], magneto[2]);
    		//	SensorModelXYZ entryMagnetoKal = new SensorModelXYZ(mtimestamp, tempKalmanMagneto[0], tempKalmanMagneto[1], tempKalmanMagneto[2] );
    			if (mIsTracking && recoredTracer){
    				mMagnetoData.add(entry2);
    				//mKalmanMagnetoData.add(entryMagnetoKal);
    			}
    				    	
    			break; 
    		case Sensor.TYPE_LINEAR_ACCELERATION:
    			numLinAccelSamples++;
    			System.arraycopy(event.values, 0, linAccel, 0, 3);
    			SensorModelXYZ entry3 = new SensorModelXYZ(mtimestamp, linAccel[0], linAccel[1], linAccel[2]);
    			//entry 3 is used below for fused Data...
    			if (mIsTracking && recoredTracer){
    				mLinAccData.add(entry3);
    			}
    				    	
    			break; 
    	
    	}
    	calculateAccMagOrientation();
    	gyroFunction( event);
    	
    	// wait for one second until gyroscope and magnetometer/accelerometer
        // data is initialised then scedule the complementary filter task

        updateOreintation();
        //DecimalFormat df = new DecimalFormat("##.##");
        // mTextView.setText("TimeStamp:"+logFormat.format(entry.getTimestamp())+" "+ mtimestamp+"; X:"+ accel[0]+ "; Y:"+ accel[1]+ "; Z:"+ accel[2]);
    	/*mTextView.setText(
    			"TimeStamp:"+mtimestamp+"; X:"+ accel[0]+ "; Y:"+ accel[1]+ "; Z:"+ accel[2]+"\n"+
    			"TimeStamp:"+mtimestamp+"; X:"+ linAccel[0]+ "; linY:"+ linAccel[1]+ "; LinZ:"+ linAccel[2]+"\n"+
    		    "TimeStamp:"+mtimestamp+"; X:"+ gyro[0]+ "; Y:"+ gyro[1]+ "; Z:"+gyro[2]+"\n"+
    		    "TimeStamp:"+mtimestamp+"; X:"+ magneto[0]+ "; Y:"+magneto[1]+ "; Z:"+magneto[2]+"\n"+
    		    "TimeStamp:"+mtimestamp+"; X:"+ accMagOrientation[0]+ "; Y:"+ accMagOrientation[1]+ "; Z:"+ accMagOrientation[2]+"\n"+
    			"TimeStamp:"+mtimestamp+"; X:"+ gyroOrientation[0]+ "; Y:"+ gyroOrientation[1]+ "; Z:"+ gyroOrientation[2]+"\n"+
    			"TimeStamp:"+mtimestamp+"; X:"+ finalFusedOrientation[0]+ "; Y:"+ finalFusedOrientation[1]+ "; Z:"+finalFusedOrientation[2]+"\n");
    			       	*/	
    			
    	/*"TimeStamp:"+mtimestamp+"; X:"+ df.format(accel[0])+ "; Y:"+ df.format(accel[1])+ "; Z:"+ df.format(accel[2])+"\n"+
    	"TimeStamp:"+mtimestamp+"; X:"+ df.format(linAccel[0])+ "; linY:"+ df.format(linAccel[1])+ "; LinZ:"+ df.format(linAccel[2])+"\n"+
	 	"TimeStamp:"+mtimestamp+"; X:"+ df.format(gyro[0])+ "; Y:"+ df.format(gyro[1])+ "; Z:"+df.format( gyro[2])+"\n"+
		"TimeStamp:"+mtimestamp+"; X:"+ df.format(magneto[0])+ "; Y:"+ df.format(magneto[1])+ "; Z:"+df.format(magneto[2])+"\n"+
		"TimeStamp:"+mtimestamp+"; X:"+ df.format(accMagOrientation[0])+ "; Y:"+ df.format(accMagOrientation[1])+ "; Z:"+ df.format(accMagOrientation[2])+"\n"+
		"TimeStamp:"+mtimestamp+"; X:"+ df.format(gyroOrientation[0])+ "; Y:"+ df.format(gyroOrientation[1])+ "; Z:"+ df.format(gyroOrientation[2])+"\n"+
		"TimeStamp:"+mtimestamp+"; X:"+ df.format(finalFusedOrientation[0])+ "; Y:"+ df.format(finalFusedOrientation[1])+ "; Z:"+ df.format(finalFusedOrientation[2])+"\n");
       */
    	SensorModelXYZ entry4 = new SensorModelXYZ(mtimestamp, finalFusedOrientation[0], finalFusedOrientation[1],finalFusedOrientation[2]);
    	if (mIsTracking && recoredTracer){
    		finalFusedData.add(entry4); 
		}
    	
	}
	
	// calculates orientation angles from accelerometer and magnetometer output
	public void calculateAccMagOrientation() {
	    if(SensorManager.getRotationMatrix(rotationMatrix, null, accel, magneto)) {
	        SensorManager.getOrientation(rotationMatrix, accMagOrientation);
	    }
	   
	}
	
	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
    	//Log.e("toast", "Options Menu");
    	//Handle Item Selection
    	switch (item.getItemId())
    	{
    	
    	case R.id.settings:
    		
    		startActivityForResult(new Intent(getApplicationContext(),Settings.class),1);
    		
    	/*case R.id.left:
    		Toast.makeText(AccelDataRecorderPhone.this, "Left Pocket is Selected", Toast.LENGTH_LONG).show();		
    		location = SensorLocation.LEFT_POCKET;
    		//Log.e("toast","Left Pocket is Selected" );
    		break; */   		
    	/*case R.id.right:
    		Toast.makeText(AccelDataRecorderPhone.this, "Right Pocket is Selected", Toast.LENGTH_LONG).show();
    	    //settings();
    		location = SensorLocation.RIGHT_POCKET;
    		//Log.e("toast","Right Pocket is Selected" );
    	    break;*/
    	 
    	/*default:
    		location = SensorLocation.NOT_DEFINED;*/
    	}
    	
    //	displayLocation(mFilename); 
    	return true;
    }   
	
    
	// This function is borrowed from the Android reference
			// at http://developer.android.com/reference/android/hardware/SensorEvent.html#values
			// It calculates a rotation vector from the gyroscope angular speed values.
		    private void getRotationVectorFromGyro(float[] gyroValues,
		            float[] deltaRotationVector,
		            float timeFactor)
			{
				float[] normValues = new float[3];
				
				// Calculate the angular speed of the sample
				float omegaMagnitude =
				(float)Math.sqrt(gyroValues[0] * gyroValues[0] +
				gyroValues[1] * gyroValues[1] +
				gyroValues[2] * gyroValues[2]);
				
				// Normalize the rotation vector if it's big enough to get the axis
				if(omegaMagnitude > EPSILON) {
				normValues[0] = gyroValues[0] / omegaMagnitude;
				normValues[1] = gyroValues[1] / omegaMagnitude;
				normValues[2] = gyroValues[2] / omegaMagnitude;
				}
				
				// Integrate around this axis with the angular speed by the timestep
				// in order to get a delta rotation from this sample over the timestep
				// We will convert this axis-angle representation of the delta rotation
				// into a quaternion before turning it into the rotation matrix.
				float thetaOverTwo = omegaMagnitude * timeFactor;
				float sinThetaOverTwo = (float)Math.sin(thetaOverTwo);
				float cosThetaOverTwo = (float)Math.cos(thetaOverTwo);
				deltaRotationVector[0] = sinThetaOverTwo * normValues[0];
				deltaRotationVector[1] = sinThetaOverTwo * normValues[1];
				deltaRotationVector[2] = sinThetaOverTwo * normValues[2];
				deltaRotationVector[3] = cosThetaOverTwo;
			}
	
	// This function performs the integration of the gyroscope data.
    // It writes the gyroscope based orientation into gyroOrientation.
    
	@SuppressLint("NewApi")
	public void gyroFunction(SensorEvent event) {
        // don't start until first accelerometer/magnetometer orientation has been acquired
        if (accMagOrientation == null)
            return;
     
        // initialisation of the gyroscope based rotation matrix
        if(initState) {
            float[] initMatrix = new float[9];
            initMatrix = getRotationMatrixFromOrientation(accMagOrientation);
            float[] test = new float[3];
            SensorManager.getOrientation(initMatrix, test);
            gyroMatrix = matrixMultiplication(gyroMatrix, initMatrix);
            initState = false;
        }
     
        // copy the new gyro values into the gyro array
        // convert the raw gyro data into a rotation vector
        float[] deltaVector = new float[4];
        if(timestamp != 0) {
            final float dT = (event.timestamp - timestamp) * NS2S;
         //   System.arraycopy(event.values, 0, gyro, 0, 3);
         //   mPrevGyro = LowPassFilter.filter(gyro, mPrevGyro);    
        getRotationVectorFromGyro(gyro, deltaVector, dT / 2.0f);
        }
     
        // measurement done, save current time for next interval
        timestamp = event.timestamp;
     
        // convert rotation vector into rotation matrix
        float[] deltaMatrix = new float[9];
        SensorManager.getRotationMatrixFromVector(deltaMatrix, deltaVector);
     
        // apply the new rotation interval on the gyroscope based rotation matrix
        gyroMatrix = matrixMultiplication(gyroMatrix, deltaMatrix);
     
        // get the gyroscope based orientation from the rotation matrix
        SensorManager.getOrientation(gyroMatrix, gyroOrientation);
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

    @Override
    protected void onResume() {      
    //    initListeners();
        super.onResume();
    }
    @Override
    protected void onPause() {
    	//initListeners();
        super.onPause();
    }
    
    @Override 
    protected void onStop()
    {//	initListeners();
    	Log.i("Ïnfo", "On Stop is Called");
    	super.onStop();
    	
    }
    
  
	@Override
	protected void onDestroy() {
		// TODO Auto-generated method stub
		super.onDestroy();
		mSensorManager.unregisterListener(this);
		lock.release();
	}

	private boolean createBasicFileTree() {
		Calendar cal = Calendar.getInstance();
		cal.setTimeInMillis(System.currentTimeMillis());
		Date date = cal.getTime();
		SimpleDateFormat dateformatMMDDYYYY = new SimpleDateFormat(
				"ddMMyyyy", Locale.GERMANY);
		mNowDDMMYYYY = new StringBuilder(
				dateformatMMDDYYYY.format(date));

		File folder = new File(Environment.getExternalStorageDirectory()
				+ "/" + mNowDDMMYYYY.toString());
		boolean success = true;
		if (!folder.exists()) {
			success = folder.mkdir();
		}
		if (success) {
			// Do something on success
			Log.i("FOLDER_CREATION", "Creating successfull");
			return true;
		} else {
			// Do something else on failure
			Log.i("FOLDER_CREATION", "Creating failed");
			return false;
		}
		
	}

	@SuppressLint("NewApi")
	private void showSettings() {
		Log.e("Called","ask User CAlled");
		startActivityForResult(new Intent(getApplicationContext(),Settings.class),1);
		
		// get prompts.xml view
		//LayoutInflater li = LayoutInflater.from(this);
		//View promptsView = li.inflate(R.layout.file_name_selection, null);

		//AlertDialog.Builder alertDialogBuilder = new AlertDialog.Builder(
		//		this);

		// set prompts.xml to alertdialog builder
		//alertDialogBuilder.setView(promptsView);

		//final EditText userInput = (EditText) promptsView
		//		.findViewById(R.id.editTextDialogUserInput);

		// set dialog message
		//alertDialogBuilder
		//	.setCancelable(false)
		//	.setPositiveButton("OK",
		//	  new DialogInterface.OnClickListener() {
			    
		//		public void onClick(DialogInterface dialog,int id) {
				// get user input and set it to result
				// edit text
		//	    String filename;
			  //  filename = userInput.getText().toString();
			    
			 //   filename = fileNamer.toString();
			 //   Log.e("fileName:",""+filename);
			 //   mFilename = filename;
			//    Log.i("Your File name", mFilename);
			//    mIsTracking = true;
		//	    if (mFilename != null && !mFilename.isEmpty()){
		//	    recoredTracer = true;
		//	    }
			   // }

		//	  })
		//	.setNegativeButton("Cancel",
		//	  new DialogInterface.OnClickListener() {
		//	    public void onClick(DialogInterface dialog,int id) {
		//		dialog.cancel();
		//		mIsTracking = false;
		//		recoredTracer = false;
				
		//	    }
		//	  });

		// create alert dialog
	//	AlertDialog alertDialog = alertDialogBuilder.create();

		// show it
	//	alertDialog.show();

		
	}

	
	private boolean checkExistingFilename(String filename) {
		// check if Test-pdf already exists on SD-card
		File file = new File(Environment.getExternalStorageDirectory() + "/gaitDataRecording" + "/sensorDataCapturingRight" + mNowDDMMYYYY + "/acc", filename + ".txt");
		if (!file.exists()) {
			// Copy the file to the sd-card
			return false;
		}else{
			return true;
		}
	}



	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
		// TODO Auto-generated method stub
		
	}
	
	
	public void kalmanInit(){
	}
    public float[] calcAndPrintValues(JKalman kalman, double x, double y,
				double z) {
			// check state before
			mS = kalman.Predict();

			//DecimalFormat df = new DecimalFormat("#.####");
			// function init :)
			// m.set(1, 0, rand.nextDouble());
			/*
			 * x = rand.nextGaussian(); y = rand.nextGaussian(); z =
			 * rand.nextGaussian();
			 */
			mM.set(0, 0,/* mM.get(0, 0)+ */x /* + 9.81 /*+ rand.nextGaussian() */);
			mM.set(1, 0, /* mM.get(1, 0)+ */y /* + 9.81 /*+ rand.nextGaussian() */);
			mM.set(2, 0, /* mM.get(2, 0)+ */z /* + 9.81 /*+ rand.nextGaussian() */);

			// a missing value (more then 1/4 times)
			/*
			 * if (rand.nextGaussian() < -0.8) { System.out.println("" + ";;;;;" +
			 * mS.get(0, 0) + ";" + mS.get(1, 0) + ";" + mS.get(2, 0) + ";" +
			 * mS.get(3, 0) + ";"); } else {
			 */// measurement is ok :)
				// look better
			mC = kalman.Correct(mM);

//			final String sensorValues = "(Accelerometer)Raw:\n" + df.format(x)
//					+ "--" + df.format(y) + "--" + df.format(z) + "\n\n"/*
//																		 * mM.get(0,
//																		 * 0) +
//																		 * ";\n" +
//																		 * mM.get(1,
//																		 * 0) +
//																		 * ";\n" +
//																		 * /*x + ";"
//																		 * + y + ";"
//																		 */
//					+ "Predicted:\n" + df.format(mS.get(0, 0)) + "--"
//					+ df.format(mS.get(1, 0)) + "--" + df.format(mS.get(2, 0))
//					+ "\n\n"/*
//							 * + mS.get(3, 0) + ";"
//							 */
//					+ "Corrected:\n" + df.format(mC.get(0, 0)) + "--"
//					+ df.format(mC.get(1, 0)) + "--" + df.format(mC.get(2, 0))
//					+ "\n ........-------......" /* + mC.get(3, 0) + ";" */;
			double cX = mC.get(0, 0);
			double cY = mC.get(1, 0);
			double cZ = mC.get(2, 0);
			float[] corrected = new float[3];
			corrected[0] = (float) cX;
			corrected[1] = (float) cY;
			corrected[2] = (float) cZ;
			return corrected;

		}
    
	 class calculateFusedOrientationTask extends TimerTask {
	        public void run() {
	            float oneMinusCoeff = 1.0f - FILTER_COEFFICIENT;
	            
	            /*
	             * Fix for 179° <--> -179° transition problem:
	             * Check whether one of the two orientation angles (gyro or accMag) is negative while the other one is positive.
	             * If so, add 360° (2 * math.PI) to the negative value, perform the sensor fusion, and remove the 360° from the result
	             * if it is greater than 180°. This stabilizes the output in positive-to-negative-transition cases.
	             */
	            
	            // azimuth
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
	            
	            gyroMatrix = getRotationMatrixFromOrientation(fusedOrientation);
	            System.arraycopy(fusedOrientation, 0, gyroOrientation, 0, 3);
	        }
	 }  
	 
	 public void updateOreintation() {
		 		
	    		finalFusedOrientation[0]= (float) (fusedOrientation[0] * 180/Math.PI);
	    		finalFusedOrientation[1]= (float) (fusedOrientation[1] * 180/Math.PI);
	    		finalFusedOrientation[2]= (float) (fusedOrientation[2] * 180/Math.PI);
	    	}
	 
	 
	 ////////////////////////
	// Updated Methods
	 
		private void createSubFiles(String filename) {

			// create sub-folders for all files

			File accFilePath = new File(Environment.getExternalStorageDirectory() +
					"/gaitDataRecording"
					+ "/sensorDataCapturingRight" + mNowDDMMYYYY + "/acc");
			// create folders for file path
			if (!accFilePath.exists()) {
				accFilePath.mkdirs();
			}

			File gyroFilePath = new File(Environment.getExternalStorageDirectory()+
					"/gaitDataRecording"
					+ "/sensorDataCapturingRight" + mNowDDMMYYYY + "/gyro");
			// create folders for file path
			if (!gyroFilePath.exists()) {
				gyroFilePath.mkdirs();
			}
			File magnetoFilePath = new File(Environment.getExternalStorageDirectory()+
					"/gaitDataRecording"
							+ "/sensorDataCapturingRight" + mNowDDMMYYYY + "/magneto");
			// create folders for file path
			if (!magnetoFilePath.exists()) {
				magnetoFilePath.mkdirs();
			}

			File kalAccFilePath = new File(Environment.getExternalStorageDirectory()+
					"/gaitDataRecording"
							+ "/sensorDataCapturingRight" + mNowDDMMYYYY + "/kalAcc");
			// create folders for file path
			if (!kalAccFilePath.exists()) {
				kalAccFilePath.mkdirs();
			}

			File kalGyroFilePath = new File(Environment.getExternalStorageDirectory()+
					"/gaitDataRecording"
							+ "/sensorDataCapturingRight" + mNowDDMMYYYY + "/kalGyro");
			// create folders for file path
			if (!kalGyroFilePath.exists()) {
				kalGyroFilePath.mkdirs();
			}

			File kalMagnetoFilePath = new File(Environment.getExternalStorageDirectory()+
					"/gaitDataRecording"
							+ "/sensorDataCapturingRight" + mNowDDMMYYYY + "/kalMagneto");
			// create folders for file path
			if (!kalMagnetoFilePath.exists()) {
				kalMagnetoFilePath.mkdirs();
			}

			File fusedFilePath = new File(Environment.getExternalStorageDirectory()+
					"/gaitDataRecording"
					+ "/sensorDataCapturingRight" + mNowDDMMYYYY + "/fused");
			// create folders for file path
			if (!fusedFilePath.exists()) {
				fusedFilePath.mkdirs();
			}

			File micFilePath = new File(Environment.getExternalStorageDirectory()+
					"/gaitDataRecording"
					+ "/sensorDataCapturingRight" + mNowDDMMYYYY + "/mic");
			// create folders for file path
			if (!micFilePath.exists()) {
				micFilePath.mkdirs();
			}
			File linAccelFilePath = new File(Environment.getExternalStorageDirectory()+
					"/gaitDataRecording"
							+ "/sensorDataCapturingRight" + mNowDDMMYYYY + "/linAccel");
			// create folders for file path
			if (!linAccelFilePath.exists()) {
				linAccelFilePath.mkdirs();
			}

		}
	    
		
		private void stopTrackingAndSaveData(boolean stopTracking) {
			OutputStream fo;
			File tempFile;
			SensorModelXYZ temp;
			String tempString;

			SimpleDateFormat dateformatTS = new SimpleDateFormat("MMddHHmmss");

			String TS;
			
			try {

				if (mAccData.size() > MAX_QUEUE_CAPACITY || stopTracking) {

					tempFile = new File(Environment.getExternalStorageDirectory() +
							"/gaitDataRecording" + "/sensorDataCapturingRight" + mNowDDMMYYYY + "/acc/"
							+ mFilename + ".txt");
					tempFile.createNewFile();
					// write the bytes in file
					if (tempFile.exists()) {
						fo = new FileOutputStream(tempFile, true);

						while (mAccData.peek() != null) {
							temp = mAccData.poll();
							TS = Long.toString(temp.getTS());

							tempString = logFormat.format(temp.getTimestamp()) +"\t" + TS + "\t" + temp.getX() + "\t"
									+ temp.getY() + "\t" + temp.getZ();
							fo.write(tempString.getBytes());
							fo.write(new String("\n").getBytes());

						}

						fo.close();
					}

				}
				
				
				if (mGyroData.size() > MAX_QUEUE_CAPACITY || stopTracking) {

					tempFile = new File(Environment.getExternalStorageDirectory() +
							"/gaitDataRecording" + "/sensorDataCapturingRight" + mNowDDMMYYYY + "/gyro/"
							+ mFilename + ".txt");
					tempFile.createNewFile();
					// write the bytes in file
					if (tempFile.exists()) {
						fo = new FileOutputStream(tempFile, true);

						while (mGyroData.peek() != null) {
							temp = mGyroData.poll();
							TS = Long.toString(temp.getTS());

							tempString =logFormat.format(temp.getTimestamp()) +"\t" + TS + "\t" + temp.getX() + "\t"
									+ temp.getY() + "\t" + temp.getZ();
							fo.write(tempString.getBytes());
							fo.write(new String("\n").getBytes());
						}

						fo.close();
					}

				}

				if (mMagnetoData.size() > MAX_QUEUE_CAPACITY || stopTracking) {

					tempFile = new File(Environment.getExternalStorageDirectory() +
							"/gaitDataRecording" + "/sensorDataCapturingRight" + mNowDDMMYYYY + "/magneto/"
							+ mFilename + ".txt");
					tempFile.createNewFile();
					// write the bytes in file
					if (tempFile.exists()) {
						fo = new FileOutputStream(tempFile, true);

						while (mMagnetoData.peek() != null) {
							temp = mMagnetoData.poll();
							TS = Long.toString(temp.getTS());

							tempString =logFormat.format(temp.getTimestamp()) +"\t" +  TS + "\t" + temp.getX() + "\t"
									+ temp.getY() + "\t" + temp.getZ();
							fo.write(tempString.getBytes());
							fo.write(new String("\n").getBytes());
						}

						fo.close();
					}

				}

				/*if (mKalmanAccData.size() > MAX_QUEUE_CAPACITY || stopTracking) {

					tempFile = new File(Environment.getExternalStorageDirectory() +
							"/gaitDataRecording" + "/sensorDataCapturingRight" + mNowDDMMYYYY + "/kalAcc/"
							+ mFilename + ".txt");
					tempFile.createNewFile();
					// write the bytes in file
					if (tempFile.exists()) {
						fo = new FileOutputStream(tempFile, true);

						while (mKalmanAccData.peek() != null) {
							temp = mKalmanAccData.poll();
							TS = Long.toString(temp.getTS());

							tempString = logFormat.format(temp.getTimestamp()) +"\t" + TS + "\t" + temp.getX() + "\t"
									+ temp.getY() + "\t" + temp.getZ();
							fo.write(tempString.getBytes());
							fo.write(new String("\n").getBytes());
						}

						fo.close();
					}

				}

				if (mKalmanGyroData.size() > MAX_QUEUE_CAPACITY || stopTracking) {

					tempFile = new File(Environment.getExternalStorageDirectory() +
							"/gaitDataRecording" + "/sensorDataCapturingRight" + mNowDDMMYYYY + "/kalGyro/"
							+ mFilename + ".txt");
					tempFile.createNewFile();
					// write the bytes in file
					if (tempFile.exists()) {
						fo = new FileOutputStream(tempFile, true);

						while (mKalmanGyroData.peek() != null) {
							temp = mKalmanGyroData.poll();
							TS = Long.toString(temp.getTS());

							tempString = logFormat.format(temp.getTimestamp()) +"\t" + TS + "\t" + temp.getX() + "\t"
									+ temp.getY() + "\t" + temp.getZ();
							fo.write(tempString.getBytes());
							fo.write(new String("\n").getBytes());
						}

						fo.close();
					}

				}
*/
/*				if (mKalmanMagnetoData.size() > MAX_QUEUE_CAPACITY || stopTracking) {

					tempFile = new File(Environment.getExternalStorageDirectory()
							+ "/sensorDataCapturingRight" + mNowDDMMYYYY
							+ "/kalMagneto/" + mFilename + ".txt");
					tempFile.createNewFile();
					// write the bytes in file
					if (tempFile.exists()) {
						fo = new FileOutputStream(tempFile, true);

						while (mKalmanMagnetoData.peek() != null) {
							temp = mKalmanMagnetoData.poll();
							TS = Long.toString(temp.getTS());

							tempString = TS + "\t" + temp.getX() + "\t"
									+ temp.getY() + "\t" + temp.getZ();
							fo.write(tempString.getBytes());
							fo.write(new String("\n").getBytes());
						}

						fo.close();
					}
				}*/

				/*if (finalFusedData.size() > MAX_QUEUE_CAPACITY || stopTracking) {

					tempFile = new File(Environment.getExternalStorageDirectory() +
							"/gaitDataRecording" + "/sensorDataCapturingRight" + mNowDDMMYYYY + "/fused/"
							+ mFilename + ".txt");
					tempFile.createNewFile();
					// write the bytes in file
					if (tempFile.exists()) {
						fo = new FileOutputStream(tempFile);

						while (finalFusedData.peek() != null) {
							temp = finalFusedData.poll();
							TS = Long.toString(temp.getTS());

							tempString = logFormat.format(temp.getTimestamp()) +"\t"+ TS + "\t" + temp.getX() + "\t"
									+ temp.getY() + "\t" + temp.getZ();
							fo.write(tempString.getBytes());
							fo.write(new String("\n").getBytes());
						}

						fo.close();
					}
				}*/
				
				if (mLinAccData.size() > MAX_QUEUE_CAPACITY || stopTracking) {

					tempFile = new File(Environment.getExternalStorageDirectory() +
							"/gaitDataRecording" + "/sensorDataCapturingRight" + mNowDDMMYYYY + "/linAccel/"
							+ mFilename + ".txt");
					tempFile.createNewFile();
					// write the bytes in file
					if (tempFile.exists()) {
						fo = new FileOutputStream(tempFile, true);

						while (mLinAccData.peek() != null) {
							temp = mLinAccData.poll();
							TS = Long.toString(temp.getTS());

							tempString = logFormat.format(temp.getTimestamp()) +"\t" + TS + "\t" + temp.getX() + "\t"
									+ temp.getY() + "\t" + temp.getZ();
							fo.write(tempString.getBytes());
							fo.write(new String("\n").getBytes());

						}

						fo.close();
					}

				}


			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

		}
				@Override
		public void onClick(View _view) {

			if (_view.equals(mStopButton)) {
				
				long timeDiff = new Date().getTime()-startTime.getTime();
				double accelFrequency = ((double)numAccelSamples/(double)timeDiff)*1000;
				double linAccelFrequency = ((double)numLinAccelSamples/(double)timeDiff)*1000;
				double gyroFrequency = ((double)numGyroSamples/(double)timeDiff)*1000;
				double magnetoFrequency = ((double)numMagnetoSamples/(double)timeDiff)*1000;
				String message = "Accel Freq: "+accelFrequency+"\nLinearAccel Freq: "+linAccelFrequency+"\nGryro Freq: "+gyroFrequency+"\nMag Freq: "+magnetoFrequency;
			//	String message2 = "NumAccel: "+numAccelSamples+"\nnumGryro: "+numGyroSamples+"\nnumMag : "+numMagnetoSamples+"\ntimeDiff: "+timeDiff;
				Log.e("Toast",message);
				Toast.makeText(getApplicationContext(), message, Toast.LENGTH_LONG).show();
				//Log.e("Toast",message2);
				//Toast.makeText(getApplicationContext(), message2, Toast.LENGTH_LONG).show();
				if (!mIsTracking) {
					return;
				}

				// start save method
				mIsTracking = false;
				recoredTracer= false;
			    mRecordingStatusView.setText("Not Recording");
				//ProgressDialog pd = ProgressDialog.show(this, "title", "message");
				
				
			
					new ExportTask().execute(new Boolean[]{true});
			
			//	pd.dismiss();
			/*	Thread t = new Thread(new Runnable() {
					
					@Override
					public void run() {
						// TODO Auto-generated method stub
						stopTrackingAndSaveData(true);
					}
				});
				t.start();*/
				//stopTrackingAndSaveData(true);
				// mAudioRecording.stopRecording();
				// mAudioRecording.stop();
			//	fuseTimer.cancel();
			//	fuseTimer.purge();

			} else if (_view.equals(mStartButton)) {
				
				if (mIsTracking) {
					return;
				}
				Log.e("StartButton is Clicked:","YEs");
				createAndSetDate();

				mExistingFile = true;

				// ask user for input again if user name already exists
				//while (mExistingFile) {
				showSettings();
				
				//while (mCheckEmptyName = false){
					   
			   // onCallSettingsMenu();
				//}
			    
				//}
				// create all sub files in folders
				createSubFiles(mFilename);

				// start timer for tracking
				mIsTracking = true;
			   // displayLocation(mFilename);
				// mAudioRecording.start();
				// mAudioRecording.startRecording(mTrackTimestamp, mNowDDMMYYYY);

			}
		}
				
		private  void onCallSettingsMenu()
		{
			
	
		Log.e("read Data",fileNamer.toString());
		
		}		

		private void createAndSetDate() {

			Calendar cal = Calendar.getInstance();
			cal.setTimeInMillis(System.currentTimeMillis());
			Date date = cal.getTime();
			SimpleDateFormat dateformatMMDDYYYY = new SimpleDateFormat("ddMMyyyy",
					Locale.GERMANY);
			mNowDDMMYYYY = new StringBuilder(dateformatMMDDYYYY.format(date));

		}
		
		
		private class ExportTask extends AsyncTask<Boolean, Void, Void> {
			
		
			 private ProgressDialog dialog;
			    /** application context. */
			  
			 public ExportTask() {
				 dialog = new ProgressDialog(AccelDataRecorderPhone.this);
				// TODO Auto-generated constructor stub
			}
			   public void onPreExecute() {
			        this.dialog.setMessage("Please wait! Saving Data");
			        this.dialog.show();
			        Log.e("TAG","PreExe");
			        
			    }

			/*   public void onPostExecute() {
				     Log.e("TAG","PostExe");
			      
			    }*/
			@Override
			protected Void doInBackground(Boolean... arg0) {
				
				stopTrackingAndSaveData(arg0[0].booleanValue());
				
				  if (dialog.isShowing()) {
			            dialog.dismiss();	            
			        }
				// TODO Auto-generated method stub
				return null;
			}

		}
		
}


