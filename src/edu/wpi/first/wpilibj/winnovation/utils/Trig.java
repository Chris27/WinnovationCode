/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.winnovation.utils;

import com.sun.squawk.util.MathUtils;

/**
 * I was getting some bizarre results from the MathUtils atan2 method so here
 * is some code that I know is legit.
 *
 * @author Chris
 */
public class Trig {
    	public static double atan2(double y, double x) {
           
		if (x > 0)
			return MathUtils.atan(y/x);
		else if( y >= 0 && x < 0)
			return Math.PI + MathUtils.atan(y/x);
		else if(y<0 && x < 0)
			return -Math.PI + MathUtils.atan(y/x);
		else if(y > 0 && x == 0)
			return Math.PI/2.0;
		else if(y < 0 && x == 0)
			return -Math.PI/2.0;
		else
			return 0;
	}

}
