package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.chassis.FollowPath;
import frc.robot.commands.misc.BlinkForATime;
public class LeftPath extends CommandGroup {
    public LeftPath() {
        addSequential(new BlinkForATime(3));
        addSequential(new FollowPath());
        addSequential(new BlinkForATime(3));
        // Robot.m_drivetrain.resetPath();
        // Robot.m_drivetrain.addPoint(0, 0);
        // Robot.m_drivetrain.addPoint(-9, 0);
        // Robot.m_drivetrain.generatePath();
        // addSequential(new FollowPath());
    }
}