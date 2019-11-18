package frc.lib.util;

public class AngleMath {
	public static double d2r(double degrees) {
		return Math.toRadians(degrees);
	}
	
	public static double r2d(double radians) {
		return Math.toDegrees(radians);
	}
	
	public static double boundDegrees(double angleDegrees) {	
		while(angleDegrees >= 180.0) {
			angleDegrees -= 360;
		}
		while(angleDegrees < -180.0) {
			angleDegrees += 360;
		}
		return angleDegrees;
	}
	
	public static double calculateAngleDifference(double theoretical, double actual) {
		return boundDegrees(theoretical - actual);	
	}
}
