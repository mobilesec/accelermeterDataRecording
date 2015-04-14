/**
 * 
 */
package com.example.acceldatarecorderphone;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * @author admin
 *
 */
public class Catagories {
	
	public enum SensorLocation 
	{
		LEFT_POCKET, 
		RIGHT_POCKET, 
		NOT_DEFINED
	}
	
	public static String[] getPIDArray(){
		String[] results = new String[35];
		for(int i = 0 ; i < results.length; i++)
		{
			results[i]= ""+(i+1);
		}
		return results;
	} 
	public static String[] getSensorLocationArray(){
		
		String[] results = new String[SensorLocation.values().length];
		for(int i = 0 ; i < results.length; i++)
		{
			results[i]= SensorLocation.values()[i].name();
		}
		return results;
	}

}
