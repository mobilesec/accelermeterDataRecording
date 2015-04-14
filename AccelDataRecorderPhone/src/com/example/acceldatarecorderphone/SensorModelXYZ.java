package com.example.acceldatarecorderphone;

import java.util.Date;

public class SensorModelXYZ {

	private Date timestampDateF;
	private long timestamp;
	private float x;
	private float y;
	private float z;
	private StringBuilder mNowDDMMYYYY;
	
	
	public SensorModelXYZ(long _timestamp,float _x, float _y, float _z){
		timestamp = _timestamp;
		timestampDateF = new Date();
		x = _x;
		y = _y;
		z = _z;
	}
	
	protected void setTS(long _timestamp){
		timestamp = _timestamp;
		
	}
	
	protected long getTS(){
		 
		return timestamp;
	}
	
	public Date getTimestamp() {
		return timestampDateF;
	}

	protected float getX(){
		return x;
	}
	
	protected float getY(){
		return y;
	}
	
	protected float getZ(){
		return z;
	}
	
	
}
