package frc.lib.pathfinder.kinematics;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.NoSuchElementException;

import frc.lib.pathfinder.pathCreator.SmoothPosition;
// import frc.lib.pathfinder.robot.OutputCalculator;
import frc.lib.pathfinder.pathCreator.PathGenerator;

public class TimeStepCalculator {
   public static ArrayList<Double> timeOutlined = new ArrayList<Double>();
   public static double timeStep = 0;
   
   public static void calculateTimeSteps() {
	   for(int i = 1; i < SmoothPosition.newPathPoints.size(); i++) {
		timeOutlined.add(timeStep);
	    timeStep += SmoothPosition.newPathPoints.get(i).distance(SmoothPosition.newPathPoints.get(i-1)) / SmoothVelocity.smoothedVelocities.get(i); 
	   }
   }
   
   public static double getNearestTimeStep(double time) {
	   double c = timeOutlined.stream()
	            .min(Comparator.comparingDouble(i -> Math.abs(i - time)))
	            .orElseThrow(() -> new NoSuchElementException("No value present"));
	   return c;
   }

   public static int getNearestTimeStepIndex(double time) {
	int j = 0;
	for(int i = 0; time > timeOutlined.get(i); i++) {
		j = i;
	}
	return j;
   }
   
   public static void printAtTimeStep(double timeStep, RobotTracing robotPath) {
	   System.out.println("Time Step:" + getNearestTimeStep(timeStep));
	   System.out.println("Velocity:" + SmoothVelocity.smoothedVelocities.get(getNearestTimeStepIndex(timeStep)));;
	   System.out.println("Distance Left Path:" + KinematicsCalculator.leftDistance.get(getNearestTimeStepIndex(timeStep)));
	   System.out.println("Distance Right Path:" + KinematicsCalculator.rightDistance.get(getNearestTimeStepIndex(timeStep)));
	   System.out.println("Heading:" + robotPath.heading.get(getNearestTimeStepIndex(timeStep)));
   }
//     public static void main(String [] args) {
// 	   PathGenerator.createDataSet();
// 	   SmoothPosition.smoothPath(PathGenerator.finalPoints, SmoothPosition.dataWeightA,
// 				SmoothPosition.smoothWeightB, SmoothPosition.tolerance);
// 	    KinematicsCalculator.calculuateCurvature();
// 		KinematicsCalculator.calculateVelocities();
		
// 		KinematicsCalculator.rateLimiter();
// 		SmoothVelocity.smoothVelocity(KinematicsCalculator.velocity, SmoothVelocity.dataWeightA, SmoothVelocity.smoothWeightB, SmoothVelocity.tolerance);
		
// 		RobotTracing robotPath = new RobotTracing(SmoothPosition.newPathPoints, 2);
// 		robotPath.leftRight(SmoothPosition.newPathPoints, 2);
// 		KinematicsCalculator.calculateLeftDistance(robotPath.leftPath);
// 		KinematicsCalculator.calculateRightDistance(robotPath.rightPath);
// 		KinematicsCalculator.calculateLeftVelocities(robotPath.leftPath);
// 		KinematicsCalculator.calculateRightVelocities(robotPath.rightPath);

// 	    SmoothVelocity.smoothLeftVelocity(KinematicsCalculator.leftVelocity, SmoothVelocity.dataWeightA, SmoothVelocity.smoothWeightB, SmoothVelocity.tolerance);
// 		SmoothVelocity.smoothRightVelocity(KinematicsCalculator.rightVelocity, SmoothVelocity.dataWeightA, SmoothVelocity.smoothWeightB, SmoothVelocity.tolerance);
	    
// 		calculateTimeSteps();
// //	    for(double time: timeOutlined) 
// //		 System.out.println(time);
// //	    System.out.println(getNearestTimeStepIndex(3.7));
// 	    OutputCalculator output = new OutputCalculator(0.2, 0.0089, 0.07, 6, 890);
// 	    double x = 0;
// 	    System.out.println("");
// 	    for(double l = 0; l < timeOutlined.get(timeOutlined.size()-3); l += (10 * Math.random() + 1)/100) {
// 	      System.out.println(" ");
// 	      printAtTimeStep(l, robotPath);
// 	      System.out.println(" ");
	      
// 	      x += (300 * Math.random()) * output.calculateLeftOutput(x, getNearestTimeStepIndex(l), robotPath) * 10;
// 	    }
//    } 
}
