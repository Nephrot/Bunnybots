package frc.lib.util;

public class Point {
   private double x;
   private double y;

   public Point(double x, double y) {
	   this.x = x;
	   this.y = y;   
   }
   
	// TODO Auto-generated constructor stub

   public Point(Point point) {
	   this.x = point.getX();
	   this.y = point.getY();
   }
	// TODO Auto-generated constructor stub


   public double getX() {
	   return x;
   }
   
   public double getY() {
	   return y;
   }
   
   public void setY(double y) {
	   this.y = y;
   }
   
   public void setX(double x) {
	   this.x = x;
   }
   
   public double distance(Point x) {
	   return Math.sqrt(Math.pow( x.getX() - this.getX() , 2) + Math.pow( x.getY() - this.getY() , 2));
   }
}
