package frc.robot.commands.chassis;

import frc.lib.pathfinder.kinematics.KinematicsCalculator;
import frc.lib.pathfinder.kinematics.TimeStepCalculator;
import frc.robot.Robot;
import frc.lib.pathfinder.kinematics.SmoothVelocity;
import frc.lib.pathfinder.kinematics.RobotTracing;

public class OutputCalculator {
	
	 double kP, kD, kV;
	 double lastError;
	 double distancePerPulse;
	 
	 public OutputCalculator(double kP, double kD, double kV, double wheelDiameterLength, double encoderSpinValue) {
		 this.kP = kP;
		 this.kD = kD;
		 this.kV = kV;
		 
		 lastError = 0;
		 
		 this.distancePerPulse = ((wheelDiameterLength/12) * Math.PI) / encoderSpinValue;
	 }
	 
     public double calculateDefaultOutput(double encoderTick, int i) {
    	 double distanceCovered = (encoderTick * distancePerPulse);
    	 double error = (KinematicsCalculator.distance.get(i) - distanceCovered);
    	 double calculatedValue = kP * error 
    			                  + 
    			                  kD * ((error-lastError) / TimeStepCalculator.timeOutlined.get(i))
    			                  +
    			                  kV * SmoothVelocity.smoothedVelocities.get(i).doubleValue();
    	 
    	 return calculatedValue;
     }
     
     public void setLastError(double error) {
    	lastError = error; 
     }
     
     public double calculateLeftOutput(double encoderTick, int i, RobotTracing robotPath) {
    	 double distanceCovered = (encoderTick * distancePerPulse);
    	 double error = (Robot.m_drivetrain.leftDistance.get(i) - distanceCovered);
    	 double calculatedValue = kP * error 
        			              + 
        			              kV * Robot.m_drivetrain.velocity.get(i).doubleValue();
    	 lastError = error;
    	 return calculatedValue;
     }
     
     public double calculateRightOutput(double encoderTick, int i, RobotTracing robotPath) {
    	 double distanceCovered = (encoderTick * distancePerPulse);
    	 double error = (Robot.m_drivetrain.rightDistance.get(i) - distanceCovered);
    	 double calculatedValue = kP * error 
    			                  +
    			                  kV * Robot.m_drivetrain.velocity.get(i).doubleValue();
    	 return calculatedValue;
     }
}
