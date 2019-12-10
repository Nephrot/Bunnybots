package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.chassis.OutputCalculator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends Subsystem {
        private NetworkTable table;
        private NetworkTableEntry tx;
        private NetworkTableEntry ty;
        private NetworkTableEntry ta;
        double x, y, area, targetDistance, tanY;
    
        private NetworkTableEntry camMode;
        private NetworkTableEntry pipeline;
        
        public Vision() {
            table = NetworkTableInstance.getDefault().getTable("limelight");
            tx = table.getEntry("tx");
            ty = table.getEntry("ty");
            ta = table.getEntry("ta");
        
            camMode = table.getEntry("camMode");
	     	pipeline = table.getEntry("pipeline");
		    setPipeline(2);
		    setCamMode(0);
        }
    
        public void initDefaultCommand() {
            // setDefaultCommand(new VisionProcessing());
        }

        public double getCamMode() {
            return camMode.getDouble(0);
        }
    
        public void setCamMode(double camMode) {
            this.camMode.setDouble(camMode);
        }
    
        public double getPipeline() {
            return pipeline.getDouble(0);
        }
    
        public void setPipeline(double pipeline) {
            this.pipeline.setDouble(pipeline);
        }    
    
        public double getDistance() {
            table = NetworkTableInstance.getDefault().getTable("limelight");
            tx = table.getEntry("tx");
            ty = table.getEntry("ty");
            ta = table.getEntry("ta");
            x = tx.getDouble(0.0);
            y = ty.getDouble(0.0);
            area = ta.getDouble(0.0);  
            tanY = Math.tan(Math.toRadians(y));

            targetDistance = RobotMap.limelightToTarget/tanY; 
            return targetDistance;
        }

        public void printTelemetry() {
            table = NetworkTableInstance.getDefault().getTable("limelight");
            tx = table.getEntry("tx");
            ty = table.getEntry("ty");
            ta = table.getEntry("ta");
            x = tx.getDouble(0.0);
            y = ty.getDouble(0.0);
            area = ta.getDouble(0.0);  
            tanY = Math.tan(Math.toRadians(y));

            targetDistance = RobotMap.limelightToTarget/tanY; 
            
            SmartDashboard.putNumber("Limelight: X-Value", x);
            SmartDashboard.putNumber("Limelight: Y-Value", y);
            SmartDashboard.putNumber("Limelight: Tan Value of Y", tanY);
            SmartDashboard.putNumber("Limelight: Area", area);
            SmartDashboard.putNumber("Limelight: Distance to Target", targetDistance);
        }
    
        public double getArea() {
            return ta.getDouble(0.0);
        }
    
        public double getX() {
            return tx.getDouble(0.0);
        }
    
        public double getY() {
            return ty.getDouble(0.0);
        }
    
        public void setLED(double mode) {
            table.getEntry("ledMode").setDouble(mode);
        }
    
        public void stop() {
        }
    
        public void reset() {
        }
}