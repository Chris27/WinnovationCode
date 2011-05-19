/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.winnovation.utils;

/**
 *
 * @author Chris
 */
public abstract class Point2D {

    public abstract double getX();
    public abstract double getY();
    public abstract void setLocation(double x, double y);

    public static class Double extends Point2D {

        public double x;
        public double y;

        public Double() {
            x = y = 0;
        }

        public Double(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public void setLocation(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    public static class Float extends Point2D{

        public float x;
        public float y;

        public Float() {
            x = y = 0;
        }

        public Float(float x, float y) {
            this.x = x;
            this.y = y;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }
        
        public void setLocation(double x, double y) {
            this.x = (float) x;
            this.y = (float) y;
        }
        
        public void setLocation(float x, float y) {
            this.x = x;
            this.y = y;
        }

    }

}
