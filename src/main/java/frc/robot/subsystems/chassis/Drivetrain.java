package frc.robot.subsystems.chassis;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.chassis.OutputCalculator;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.chassis.CurvatureDrive;
import frc.lib.pathfinder.pathCreator.PathGenerator;
import frc.lib.pathfinder.pathCreator.SmoothPosition;
import frc.lib.pathfinder.kinematics.*;
import frc.lib.pathfinder.kinematics.RobotTracing;
import java.util.ArrayList;
import frc.robot.commands.chassis.ArcadeDrive;

public class Drivetrain extends Subsystem {

    public CANSparkMax leftMotorA, leftMotorB, rightMotorA, rightMotorB;
    public static DifferentialDrive drivetrain;
    public OutputCalculator outputCalculator;
    public static double P, I, D, V;
    public RobotTracing robotPath;
    public ArrayList<Double> velocity, leftDistance, rightDistance;

    public Drivetrain() {
        leftMotorA = new CANSparkMax(RobotMap.leftMotorAPort, RobotMap.brushless);
        leftMotorB = new CANSparkMax(RobotMap.leftMotorBPort, RobotMap.brushless);
        rightMotorA = new CANSparkMax(RobotMap.rightMotorAPort, RobotMap.brushless);
        rightMotorB = new CANSparkMax(RobotMap.rightMotorBPort, RobotMap.brushless);

        drivetrain = new DifferentialDrive(leftMotorA, rightMotorA);
		leftMotorA.setInverted(true);
		rightMotorA.setInverted(true);
		leftMotorB.follow(leftMotorA);
        rightMotorB.follow(rightMotorA);

        outputCalculator = new OutputCalculator(RobotMap.pValue, RobotMap.dValue, RobotMap.vValue, RobotMap.wheelDiameter, RobotMap.ticksInARevolution);
        PathGenerator.createDataSet();
	    SmoothPosition.smoothPath(PathGenerator.finalPoints, SmoothPosition.dataWeightA,
				SmoothPosition.smoothWeightB, SmoothPosition.tolerance);
	    KinematicsCalculator.calculuateCurvature();
		KinematicsCalculator.calculateVelocities();
		
		KinematicsCalculator.rateLimiter();
		SmoothVelocity.smoothVelocity(KinematicsCalculator.velocity, SmoothVelocity.dataWeightA, SmoothVelocity.smoothWeightB, SmoothVelocity.tolerance);
		velocity = new ArrayList(SmoothVelocity.smoothVelocity(KinematicsCalculator.velocity, SmoothVelocity.dataWeightA, SmoothVelocity.smoothWeightB, SmoothVelocity.tolerance));
		robotPath = new RobotTracing(SmoothPosition.newPathPoints, 2);
        robotPath.leftRight(SmoothPosition.newPathPoints, 2);

		KinematicsCalculator.calculateLeftDistance(robotPath.leftPath);
        KinematicsCalculator.calculateRightDistance(robotPath.rightPath);
        leftDistance = new ArrayList(KinematicsCalculator.leftDistance);
        rightDistance = new ArrayList(KinematicsCalculator.rightDistance);
		KinematicsCalculator.calculateLeftVelocities(robotPath.leftPath);
		KinematicsCalculator.calculateRightVelocities(robotPath.rightPath);

	    SmoothVelocity.smoothLeftVelocity(KinematicsCalculator.leftVelocity, SmoothVelocity.dataWeightA, SmoothVelocity.smoothWeightB, SmoothVelocity.tolerance);
        SmoothVelocity.smoothRightVelocity(KinematicsCalculator.rightVelocity, SmoothVelocity.dataWeightA, SmoothVelocity.smoothWeightB, SmoothVelocity.tolerance);
        
    }

    public void arcadeDrive(double speed, double rotateValue) {
        drivetrain.arcadeDrive(speed, rotateValue);
    }

    public void curvatureDrive(double speed, double rotateValue) {
        drivetrain.curvatureDrive(speed, rotateValue, OI.driverController.getAButton());
    }

    public double distanceInFeet(double encoderValue) {
        return  encoderValue * (((RobotMap.wheelDiameter/12) * Math.PI) / RobotMap.ticksInARevolution);
    }
    public void initDefaultCommand() {
        setDefaultCommand(new ArcadeDrive());
    }
}