
package frc.lib.pathfinder.pathCreator;



import java.awt.Color;

import java.awt.Graphics;

import java.awt.Graphics2D;

import java.awt.RenderingHints;

import java.awt.Stroke;

import java.util.ArrayList;

import java.util.List;



import frc.lib.util.Point;

import frc.lib.util.Vector;



public class PathGenerator {

	private static final long serialVersionUID = 1L;

	   public static double spacing = 0.5;

	   public static ArrayList<Point> newPoints = new ArrayList<Point>();

	   public static ArrayList<Vector> newVectors = new ArrayList<Vector>();

	   public static ArrayList<Integer> newNumOPoints = new ArrayList<Integer>();

	   public static ArrayList<Point> finalPoints = new ArrayList<Point>();

	   

	   public static void addPoint(double x, double y) {

		newPoints.add(new Point(x, y));

		}

	   

	   public static void resetPoints() {

		//		   newPoints.remov

}

	   /** 

* 

	 */

	 

	 // Testing Variables

//	   System.out.println("Points:");

//	   for(int i = 0; i < newPoints.size(); i++) {

	//		   System.out.println(i + " X:" + newPoints.get(i).getX());

//		   System.out.println(i + " Y:" + newPoints.get(i).getY());

//	   }



//	   System.out.println("Vectors:");

//	   for(int i = 0; i < newVectors.size(); i++) {

	//	      System.out.println(i +" Line Segment Vector In X Direction:" + newVectors.get(i).getX());

//	      System.out.println(i +" Line Segment Vector In Y Direction:" + newVectors.get(i).getY());

//       }



public static void createDataSet(){

	//Add some points to the array

	   addPoint(0, 0);

	   addPoint(6.5, 0);




	   //Add (points.length - 1) vectors to appropriate the slope 
	   for(int i = 0; i < newPoints.size() - 1; i++) {
		  newVectors.add(new Vector(newPoints.get(i+1).getX() - newPoints.get(i).getX(), newPoints.get(i+1).getY() - newPoints.get(i).getY()));
	   }

	   //Check for how much points you want 
	   for(int i = 0; i < newVectors.size(); i++) {
		   newNumOPoints.add(new Integer((int) Math.ceil(newVectors.get(i).getMagnitude() / spacing)));
	   }
//	   for(int i = 0; i < newVectors.size(); i++) {
//		  System.out.println(i + " Number of Points in Line Segment:" + newNumOPoints.get(i).intValue());
//	   }
	   for(int i = 0; i < newVectors.size(); i++) {
		      double magnitude = newVectors.get(i).getMagnitude();
			  newVectors.get(i).setX((newVectors.get(i).getX() / magnitude) * spacing);
			  newVectors.get(i).setY((newVectors.get(i).getY() / magnitude) * spacing);
	   }
//	   for(int i = 0; i < newVectors.size(); i++) {
//		      System.out.println(newVectors.get(i).getX());
//		      System.out.println(newVectors.get(i).getY());
//	   }
	   for (int i = 0; i < newNumOPoints.size(); i++) {
		   for(int j = 0; j < newNumOPoints.get(i); j++) {
			   finalPoints.add(new Point(newPoints.get(i).getX() + newVectors.get(i).getX() * j, 
					           newPoints.get(i).getY() + newVectors.get(i).getY() * j));
		   }
		   
	   }
	   finalPoints.add(newPoints.get(newPoints.size()-1));
//	   for(int i = 0; i < finalPoints.size(); i++) {
////		   System.out.println(i + " X:" + finalPoints.get(i).getX());
//		   System.out.println(i + " Y:" + finalPoints.get(i).getY());
//	   }
   }
}