package com.example.acceldatarecorderphone;

import android.app.Activity;
import android.content.Intent;
import android.content.res.Resources;
import android.content.res.TypedArray;
import android.os.Bundle;
import android.util.Log;
import android.widget.ArrayAdapter;
import android.widget.Spinner;
import android.widget.Toast;

public class Settings extends Activity{
public static String DATA_KEY = "com.example.acceldatarecorderphone.data";	
public static String PID_KEY =	"com.example.acceldatarecorderphone.pid";
public static String LOC_KEY =	"com.example.acceldatarecorderphone.loc";
public static String SHOE_KEY = "com.example.acceldatarecorderphone.shoe";
public static String LOWER_KEY = "com.example.acceldatarecorderphone.lower";
public static String SURFACE_KEY = "com.example.acceldatarecorderphone.surface";
public static String WALK_KEY = "com.example.acceldatarecorderphone.walk";
public static String SESSION_KEY = "com.example.acceldatarecorderphone.session";

//public static String PID_KEY = "pid";
private Spinner pidSpinner = null;	
private Spinner locSpinner = null;
private Spinner shoeSpinner = null;
private Spinner lowerSpinner = null;
private Spinner surfaceSpinner = null;
private Spinner walkSpinner = null;
private Spinner sessionSpinner = null;


	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.settings_layout);
		
		pidSpinner = (Spinner)findViewById(R.id.pid_spinner);
		locSpinner = (Spinner)findViewById(R.id.location_spinner);
		shoeSpinner = (Spinner)findViewById(R.id.shoe_spinner);
		lowerSpinner = (Spinner)findViewById(R.id.lower_spinner);
		surfaceSpinner = (Spinner)findViewById(R.id.surface_spinner);
		walkSpinner = (Spinner)findViewById(R.id.walk_spinner);
		sessionSpinner = (Spinner)findViewById(R.id.session_spinner);
		
		/*ArrayAdapter<CharSequence> pIDAdapter = new ArrayAdapter<CharSequence>(getApplicationContext(), android.R.layout.simple_spinner_item, Catagories.getPIDArray());
		pidSpinner.setAdapter(pIDAdapter);*/
		
		ArrayAdapter<CharSequence> pidAdapter = ArrayAdapter.createFromResource(this,
		        R.array.person_id, android.R.layout.simple_spinner_item);
		pidSpinner.setAdapter(pidAdapter);
		
		ArrayAdapter<CharSequence> locAdapter = ArrayAdapter.createFromResource(this,
		        R.array.sensor_locations, android.R.layout.simple_spinner_item);
		locSpinner.setAdapter(locAdapter);
		
		ArrayAdapter<CharSequence> shoeAdapter = ArrayAdapter.createFromResource(this,
		        R.array.shoe_types, android.R.layout.simple_spinner_item);
		shoeSpinner.setAdapter(shoeAdapter);
		
		ArrayAdapter<CharSequence> lowerAdapter = ArrayAdapter.createFromResource(this,
		        R.array.lower_types, android.R.layout.simple_spinner_item);
		lowerSpinner.setAdapter(lowerAdapter);
		
		ArrayAdapter<CharSequence> surfaceAdapter = ArrayAdapter.createFromResource(this,
		        R.array.suface_types, android.R.layout.simple_spinner_item);
		surfaceSpinner.setAdapter(surfaceAdapter);
		
		ArrayAdapter<CharSequence> walkAdapter = ArrayAdapter.createFromResource(this,
		        R.array.walk_speed, android.R.layout.simple_spinner_item);
		walkSpinner.setAdapter(walkAdapter);
		
		ArrayAdapter<CharSequence> sessionAdapter = ArrayAdapter.createFromResource(this,
		        R.array.session_id, android.R.layout.simple_spinner_item);
		sessionSpinner.setAdapter(sessionAdapter);
		
		
		Resources resources = getResources();
	    TypedArray preSets = resources.obtainTypedArray(R.array.Pre_Sets);
	    Log.e("settings",preSets.toString());
		pidSpinner.setSelection(preSets.getInt(0,0));
		locSpinner.setSelection(preSets.getInt(1,0));
		shoeSpinner.setSelection(preSets.getInt(2,0));
		lowerSpinner.setSelection(preSets.getInt(3,0));
		surfaceSpinner.setSelection(preSets.getInt(4,0));
		walkSpinner.setSelection(preSets.getInt(5,0));
		sessionSpinner.setSelection(preSets.getInt(6,0));
	}


	@Override
	protected void onDestroy() {
		
		super.onDestroy();
		
	}
	
	@Override
	public void finish() {
		int[] settings = new int[]{pidSpinner.getSelectedItemPosition(),shoeSpinner.getSelectedItemPosition(),
				lowerSpinner.getSelectedItemPosition(),locSpinner.getSelectedItemPosition(),
				surfaceSpinner.getSelectedItemPosition(),walkSpinner.getSelectedItemPosition(),
				sessionSpinner.getSelectedItemPosition()};
	    
		for(int i = 0; i<3; i++){
			if (settings[i]==0)
			{
				Toast.makeText(getApplicationContext(), "!!Please enter Person ID, Shoe Type and Lower Type!!", Toast.LENGTH_LONG).show();	
				return;
			}
			
		}
		
		Bundle data = new Bundle();
		data.putInt(PID_KEY,settings[0]);
		data.putInt(SHOE_KEY,settings[1]);
		data.putInt(LOWER_KEY, settings[2]);
		data.putInt(LOC_KEY, settings[3]);
		data.putInt(SURFACE_KEY, settings[4]);
		data.putInt(WALK_KEY, settings[5]);
		data.putInt(SESSION_KEY,settings[6]);
		
		
		//Log.e("read Data", "PID: "+pidSpinner.getSelectedItemPosition()+ "\nLOCID:"+locSpinner.getSelectedItemPosition());
		Intent intent = new Intent();
		intent.putExtra(DATA_KEY, data);
		setResult(RESULT_OK, intent);
		
	   super.finish();
	}
	
	
	
}
