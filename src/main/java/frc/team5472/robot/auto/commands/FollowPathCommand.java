package frc.team5472.robot.auto.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team5472.robot.Drivetrain;
import frc.team5472.robot.Robot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

import java.util.TimerTask;

public class FollowPathCommand extends Command {

    protected boolean enabled = true;

    private static final double p = 1.0;
    private static final double d = 0.0;
    private static final double v = 0.0;
    private static final double a = 0.0;
    private static final double g = 0.0;

    private Drivetrain drive;

    EncoderFollower leftFollower, rightFollower;
    TimerTask controlTask;

    public FollowPathCommand(Trajectory left, Trajectory right) {
        this.drive = Robot.getDrive();

        leftFollower = new EncoderFollower();
        leftFollower.configurePIDVA(p, 0.0, d, v, a);
        leftFollower.configureEncoder(drive.getLeftRaw(), 4096, 0.10);
        leftFollower.setTrajectory(left);

        rightFollower = new EncoderFollower();
        rightFollower.configurePIDVA(p, 0.0, d, v, a);
        rightFollower.configureEncoder(drive.getRightRaw(), 4096, 0.10);
        rightFollower.setTrajectory(right);

        controlTask = new TimerTask() {
            public void run() {
                double leftOutput = leftFollower.calculate(drive.getLeftRaw());
                double rightOutput = rightFollower.calculate(drive.getRightRaw());

                double desired = Pathfinder.r2d(leftFollower.getHeading());
                double current = drive.getHeading();

                drive.drive(leftOutput, rightOutput);
            }
        };
    }

    protected boolean isFinished() {
        return leftFollower.isFinished() || rightFollower.isFinished();
    }
}
