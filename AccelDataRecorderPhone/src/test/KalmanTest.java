 /*******************************************************************************
  * 
  *  JKalman - KALMAN FILTER (Java) TestBench
  *
  *  Copyright (C) 2007 Petr Chmelar
  *
  *  By downloading, copying, installing or using the software you agree to 
  *  the license in licenseIntel.txt or in licenseGNU.txt
  *
  **************************************************************************** */

package test;

import jama.Matrix;

import java.util.Random;

import jkalman.JKalman;



/**
 * JKalman TestBench
 */
public class KalmanTest {
    private static Matrix mS;
	private static Matrix mC;
	private static Matrix mM;
	private static JKalman mKalman;

	/**
     * Constructor
     */
    public KalmanTest() {
    }

    /**
     * Main method
     * @param args
     */
    public static void main(String[] args) {

        try {
            mKalman = new JKalman(6, 3);

            Random rand = new Random(System.currentTimeMillis() % 2011);
            double x = 0;
            double y = 0;
            double z = 0;
            // constant velocity
            double dx = rand.nextDouble(); //da/dt //a =9.81m/s²
            double dy = rand.nextDouble();
            double dz = rand.nextDouble();
            
            mS = new Matrix(9, 1);
            mC = new Matrix(9, 1);
            
            mM = new Matrix(3, 1);
            mM.set(0, 0, x);
            mM.set(1, 0, y);
            mM.set(2, 0, z);

            // transitions for x, y, dx, dy, 
           /* double[][] tr = { {1, 0, 1, 0}, 
                              {0, 1, 0, 1}, 
                              {0, 0, 1, 0}, 
                              {0, 0, 0, 1} };*/
            // transitions for x, y, z, dx, dy, dz, 
            double[][] tr = { {1, 0, 0, 1, 0, 0}, 
                              {0, 1, 0, 0, 1, 0}, 
                              {0, 0, 1, 0, 0, 1}, 
                              {0, 0, 0, 1, 0, 0}, 
                              {0, 0, 0, 0, 1, 0}, 
                              {0, 0, 0, 0, 0, 1} };
            mKalman.setTransition_matrix(new Matrix(tr));
            
            // 1s somewhere?
            mKalman.setError_cov_post(mKalman.getError_cov_post().identity());

            // init first assumption similar to first observation (cheat :)
            // kalman.setState_post(kalman.getState_post());

            // report what happend first :)
            System.out.println("first x:" + x + ", y:" + y + ", z: "+z+", dx:" + dx + ", dy:" + dy+", dz"+dz);
            System.out.println("no; x; y; dx; dy; predictionX; predictionY; predictionDx; predictionDy; correctionX; correctionY; correctionDx; correctionDy;");
            
            // For debug only
            for (int i = 0; i < 200; ++i) {
                           
                calcAndPrintValues(mKalman, rand, i);

            }
        } catch (Exception ex) {
            System.out.println(ex.getMessage());
        }
    }

	private static void calcAndPrintValues(JKalman kalman, Random rand,
			 int i) {
		double x;
		double y;
		double z;
		// check state before
		mS = kalman.Predict();
		
		// function init :)
		// m.set(1, 0, rand.nextDouble());
		x = rand.nextGaussian();
		y = rand.nextGaussian();
		z = rand.nextGaussian();
         
		mM.set(0, 0, mM.get(0, 0) + 9.81 + rand.nextGaussian());
		mM.set(1, 0, mM.get(1, 0) + 9.81 + rand.nextGaussian());   
		mM.set(2, 0, mM.get(2, 0) + 9.81 + rand.nextGaussian());                
         
		// a missing value (more then 1/4 times)
		if (rand.nextGaussian() < -0.8) { 
		    System.out.println("" + i + ";;;;;"
		             + mS.get(0, 0) + ";" + mS.get(1, 0) + ";" + mS.get(2, 0) + ";" + mS.get(3, 0) + ";");
		}
		else { // measurement is ok :)
		    // look better
		    mC = kalman.Correct(mM);
		
		    System.out.println("" + i + ";" +  mM.get(0, 0) + ";" + mM.get(1, 0) + ";" + x + ";" + y + ";"
		             + mS.get(0, 0) + ";" + mS.get(1, 0) + ";" + mS.get(2, 0) + ";" + mS.get(3, 0) + ";"
		             + mC.get(0, 0) + ";" + mC.get(1, 0) + ";" + mC.get(2, 0) + ";" + mC.get(3, 0) + ";");
		}
	}
}
