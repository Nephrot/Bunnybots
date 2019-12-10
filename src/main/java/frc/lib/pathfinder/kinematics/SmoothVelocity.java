package frc.lib.pathfinder.kinematics;

import java.util.ArrayList;

public class SmoothVelocity {
	public static double smoothWeightB = 0.82;
    public static double dataWeightA = 1 - smoothWeightB;
    public static double tolerance = 0.001;
    public static double pastVelocity = 0;
    public static ArrayList<Double> velocities = new ArrayList<Double>(); 
    public static ArrayList<Double> leftVelocities = new ArrayList<Double>(); 
    public static ArrayList<Double> rightVelocities = new ArrayList<Double>(); 
    
    public static ArrayList<Double> smoothedVelocities = new ArrayList<Double>(); 
    public static ArrayList<Double> smoothedRightVelocities = new ArrayList<Double>();
    public static ArrayList<Double> smoothedLeftVelocities = new ArrayList<Double>();
//	public static double[][] arrayListToArrayCopy(ArrayList<Point> points)
//	{
//
//		//size first dimension of array
//		double[][] temp = new double[points.size()][1];
//
//		for(int i=0; i<points.size(); i++)
//		{
//			//Resize second dimension of array
//			temp[i] = new double[1];
//
//			//Copy Contents
//			for(int j=0; j<1; j++) {
//				if(j == 0) {
//				  temp[i][j] = points.get(i).getX();
//				}
//				else {
//				  temp[i][j] = points.get(i).getY();
//				}
//		   }
//		}
//
//		return temp;
//
//	}
	
    public static ArrayList<Double> smoothVelocity(ArrayList<Double> path, double a, double b, double t) {
    	for(Double velocity: path) {
    		smoothedVelocities.add(velocity);
    	}
        double change = tolerance;
        while(change >= tolerance) {
        	change = 0.0;
        	for(int i = 1; i < path.size()-1; i++) {
        		double pastVelocity = smoothedVelocities.get(i);
				smoothedVelocities.set(i, smoothedVelocities.get(i) + a * (path.get(i) - smoothedVelocities.get(i)) + b * (smoothedVelocities.get(i-1) + smoothedVelocities.get(i+1)- (2.0 * smoothedVelocities.get(i))));
				change += Math.abs(pastVelocity - smoothedVelocities.get(i));
//                System.out.println("Past:" + path.get(i).getX() + ", New" + newPathPoints.get(i).getX());;
        	}
        }
        		
		return smoothedVelocities;	
    }
    
    public static ArrayList<Double> smoothLeftVelocity(ArrayList<Double> path, double a, double b, double t) {
    	for(Double velocity: path) {
    		smoothedLeftVelocities.add(velocity);
    	}
        double change = tolerance;
        while(change >= tolerance) {
        	change = 0.0;
        	for(int i = 1; i < path.size()-1; i++) {
        		double pastVelocity = smoothedLeftVelocities.get(i);
				smoothedLeftVelocities.set(i, smoothedLeftVelocities.get(i) + a * (path.get(i) - smoothedLeftVelocities.get(i)) + b * (smoothedLeftVelocities.get(i-1) + smoothedLeftVelocities.get(i+1)- (2.0 * smoothedLeftVelocities.get(i))));
				change += Math.abs(pastVelocity - smoothedLeftVelocities.get(i));
//                System.out.println("Past:" + path.get(i).getX() + ", New" + newPathPoints.get(i).getX());;
        	}
        }
        		
		return smoothedLeftVelocities;	
    }
    
    public static ArrayList<Double> smoothRightVelocity(ArrayList<Double> path, double a, double b, double t) {
    	for(Double velocity: path) {
    		smoothedRightVelocities.add(velocity);
    	}
        double change = tolerance;
        while(change >= tolerance) {
        	change = 0.0;
        	for(int i = 1; i < path.size()-1; i++) {
        		double pastVelocity = smoothedRightVelocities.get(i);
				smoothedRightVelocities.set(i, smoothedRightVelocities.get(i) + a * (path.get(i) - smoothedRightVelocities.get(i)) + b * (smoothedRightVelocities.get(i-1) + smoothedRightVelocities.get(i+1)- (2.0 * smoothedRightVelocities.get(i))));
				change += Math.abs(pastVelocity - smoothedRightVelocities.get(i));
//                System.out.println("Past:" + path.get(i).getX() + ", New" + newPathPoints.get(i).getX());;
        	}
        }
        		
		return smoothedRightVelocities;	
    }
    
    
    public static void main(String [] args) {
//    	SystematicConverter.createDataSet();
//    	int i = 0;
//    	for(Point point: smoothPath(SystematicConverter.finalPoints, dataWeightA, smoothWeightB, tolerance)) {
//    		pathPoints.add(point);
//    		System.out.println(SystematicConverter.finalPoints.get(i).getX());
//	        System.out.println(point.getX());
//	        i++;
//    	}
    	
//    	for(int i = 0; i < pathPoints.size(); i++) {
//	        System.out.println(i + " X:" + pathPoints.get(i).getX());
//	        System.out.println(i + " Old X Points:" + SystematicConverter.finalPoints.get(i).getX());
//	        System.out.println(i + " Y:" + pathPoints.get(i).getY());
//	        System.out.println(i + " Old Y Points:" + SystematicConverter.finalPoints.get(i).getY());
//    	}    	
    }
   
    

}
