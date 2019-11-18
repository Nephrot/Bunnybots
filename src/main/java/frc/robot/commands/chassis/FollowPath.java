package frc.robot.commands.chassis;

import frc.robot.OI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.lib.pathfinder.pathCreator.PathGenerator;
import frc.lib.pathfinder.pathCreator.SmoothPosition;
import frc.lib.pathfinder.kinematics.KinematicsCalculator;
import frc.lib.pathfinder.kinematics.SmoothVelocity;
import frc.lib.pathfinder.kinematics.RobotTracing;
import frc.lib.pathfinder.kinematics.TimeStepCalculator;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import frc.lib.util.Point;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.pathfinder.kinematics.TimeStepCalculator;

public class FollowPath extends Command {
    public Timer timer;
    public ArrayList<Point> leftPath, rightPath;
	public ArrayList<Double> timeOutlined;
	public FollowPath() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {	    
		TimeStepCalculator.calculateTimeSteps();
		timeOutlined = new ArrayList(TimeStepCalculator.timeOutlined);
		timer = new Timer();
        timer.reset();
        timer.start();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.m_drivetrain.leftMotorA.set(Robot.m_drivetrain.outputCalculator.calculateLeftOutput(Robot.m_drivetrain.leftMotorA.getEncoder().getPosition(), TimeStepCalculator.getNearestTimeStepIndex(timer.get()), Robot.m_drivetrain.robotPath));
        Robot.m_drivetrain.rightMotorA.set(Robot.m_drivetrain.outputCalculator.calculateRightOutput(Robot.m_drivetrain.rightMotorA.getEncoder().getPosition(), TimeStepCalculator.getNearestTimeStepIndex(timer.get()), Robot.m_drivetrain.robotPath));
	    SmartDashboard.putNumber("Output (Left Wheels)", Robot.m_drivetrain.outputCalculator.calculateLeftOutput(Robot.m_drivetrain.leftMotorA.getEncoder().getPosition(), getNearestTimeStepIndex(timer.get()), Robot.m_drivetrain.robotPath));
	    SmartDashboard.putNumber("Output (Right Wheels)", Robot.m_drivetrain.outputCalculator.calculateRightOutput(Robot.m_drivetrain.rightMotorA.getEncoder().getPosition(), getNearestTimeStepIndex(timer.get()), Robot.m_drivetrain.robotPath));
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return timer.get() > timeOutlined.get(timeOutlined.size() - 1);
	}

	// Called once after isFinished returns true
	protected void end() {
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
}
