package frc.lib.pathfinder.pathCreator;

import java.util.ArrayList;

import frc.lib.util.Point;

public class SmoothPosition {
	public static double smoothWeightB = 0.93;
    public static double dataWeightA = 1 - smoothWeightB;
    public static double tolerance = 0.001;
    public static double pastLocationX = 0;
    public static double pastLocationY = 0;
    public static ArrayList<Point> pathPoints = new ArrayList<Point>(); 
    
    public static ArrayList<Point> newPathPoints = new ArrayList<Point>(); 
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
	
    public static ArrayList<Point> smoothPath(ArrayList<Point> path, double a, double b, double t) {
    	for(Point point: path) {
    		newPathPoints.add(new Point(point));
    	}
        double change = tolerance;
        while(change >= tolerance) {
        	change = 0.0;
        	for(int i = 1; i < path.size()-1; i++) {
        		double pastLocationX = newPathPoints.get(i).getX();
				newPathPoints.get(i).setX(newPathPoints.get(i).getX() + a * (path.get(i).getX() - newPathPoints.get(i).getX()) + b * (newPathPoints.get(i-1).getX() + newPathPoints.get(i+1).getX() - (2.0 * newPathPoints.get(i).getX())));
				change += Math.abs(pastLocationX - newPathPoints.get(i).getX());
//                System.out.println("Past:" + path.get(i).getX() + ", New" + newPathPoints.get(i).getX());
				double pastLocationY = newPathPoints.get(i).getY();
				newPathPoints.get(i).setY(newPathPoints.get(i).getY() + a * (path.get(i).getY() - newPathPoints.get(i).getY()) + b * (newPathPoints.get(i-1).getY() + newPathPoints.get(i+1).getY() - (2.0 * newPathPoints.get(i).getY())));
				change += Math.abs(pastLocationY - newPathPoints.get(i).getY());
        	}
        }
        		
		return newPathPoints;	
    }
}
