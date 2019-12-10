package frc.robot.commands.chassis;
 
import frc.robot.OI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.lib.pathfinder.pathCreator.PathGenerator;
import frc.lib.pathfinder.pathCreator.SmoothPosition;
import frc.lib.pathfinder.kinematics.KinematicsCalculator;
import frc.lib.pathfinder.kinematics.SmoothVelocity;
import frc.lib.pathfinder.kinematics.RobotTracing;
import frc.lib.pathfinder.kinematics.TimeStepCalculator;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;

import com.revrobotics.CANSparkMax;

import frc.lib.util.AngleMath;
import frc.lib.util.Point;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.pathfinder.kinematics.TimeStepCalculator;
import frc.robot.subsystems.chassis.Drivetrain;
 
public class FollowPath extends Command {
   public Timer timer;
   public ArrayList<Point> leftPath, rightPath;
    public ArrayList<Double> timeOutlined;
    public ArrayList<Double> heading;
    
    public double gyroHeading;
    public double desiredHeading;
    public double angleDifference;
    public double headingValue;
 
    public boolean isFinished = false;
 
    public FollowPath() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
		heading = new ArrayList(Drivetrain.robotPath.heading);
    }
 
    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.m_drivetrain.leftMotorA.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.m_drivetrain.leftMotorB.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.m_drivetrain.rightMotorA.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.m_drivetrain.rightMotorB.setIdleMode(CANSparkMax.IdleMode.kCoast);	
		Robot.m_drivetrain.leftMotorA.getEncoder().setPosition(0);  
		Robot.m_drivetrain.leftMotorB.getEncoder().setPosition(0);  
		Robot.m_drivetrain.rightMotorA.getEncoder().setPosition(0);  
		Robot.m_drivetrain.rightMotorB.getEncoder().setPosition(0);  
        TimeStepCalculator.calculateTimeSteps();
        timeOutlined = new ArrayList(TimeStepCalculator.timeOutlined);
		heading = new ArrayList(Drivetrain.robotPath.heading);
		heading.add(0, 0.0);
        timer = new Timer();
       timer.reset();
       timer.start();
    }
 
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        try {
        calculateHeading();
        
        Robot.m_drivetrain.leftMotorA.set(Robot.m_drivetrain.outputCalculator.calculateLeftOutput(Robot.m_drivetrain.leftMotorA.getEncoder().getPosition(), TimeStepCalculator.getNearestTimeStepIndex(timer.get()), Robot.m_drivetrain.robotPath) + (headingValue * RobotMap.hValue));
        Robot.m_drivetrain.rightMotorA.set(-Robot.m_drivetrain.outputCalculator.calculateRightOutput(-Robot.m_drivetrain.rightMotorA.getEncoder().getPosition(), TimeStepCalculator.getNearestTimeStepIndex(timer.get()), Robot.m_drivetrain.robotPath) + (headingValue * RobotMap.hValue));
 	    SmartDashboard.putNumber("Output (Left Wheels)", Robot.m_drivetrain.outputCalculator.calculateLeftOutput(-Robot.m_drivetrain.leftMotorA.getEncoder().getPosition(), getNearestTimeStepIndex(timer.get()), Robot.m_drivetrain.robotPath) - (headingValue * RobotMap.hValue));
 	    SmartDashboard.putNumber("Output (Right Wheels)", Robot.m_drivetrain.outputCalculator.calculateRightOutput(Robot.m_drivetrain.rightMotorA.getEncoder().getPosition(), getNearestTimeStepIndex(timer.get()), Robot.m_drivetrain.robotPath) + (headingValue * RobotMap.hValue));
        SmartDashboard.putNumber("Heading ", getNearestTimeStepIndex(timer.get()));
        }
        catch(java.lang.IndexOutOfBoundsException E) {
          isFinished = true;
        }
    }
 
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isFinished;
    }
 
    // Called once after isFinished returns true
    protected void end() {
		Robot.m_drivetrain.leftMotorA.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.m_drivetrain.leftMotorB.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.m_drivetrain.rightMotorA.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Robot.m_drivetrain.rightMotorB.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }
 
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
 
    public int getNearestTimeStepIndex(double time) {
        int j = 0;
        for(int i = 0; time > timeOutlined.get(i); i++) {
            j = i;
        }
        return j;
    }
 
    public void calculateHeading() {
        gyroHeading = Robot.m_navX.getAngle();
       desiredHeading = heading.get(getNearestTimeStepIndex(timer.get()));
       angleDifference = AngleMath.boundDegrees(desiredHeading - gyroHeading);
        headingValue = angleDifference;
    }
}
 
