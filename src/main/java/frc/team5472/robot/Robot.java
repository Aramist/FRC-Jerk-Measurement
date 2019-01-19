package frc.team5472.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Robot extends IterativeRobot {

    private static Drivetrain drive;
    private static Controls ctrl;

    ArrayList<Double> positions = new ArrayList<Double>();
    ArrayList<Double> times = new ArrayList<Double>();

    @Override
    public void robotInit() {
        drive = new Drivetrain();
        ctrl = new Controls();
    }

    @Override
    public void disabledInit() {
        if (positions.size() > 0) {
            double[] velocities = new double[positions.size() - 1];
            double[] velTimes = new double[positions.size() - 1];
            double maxVelocity = 0.0;
            for (int i = 0; i < velocities.length; i++) {
                velocities[i] = (positions.get(i + 1) - positions.get(i))
                        / (times.get(i + 1) - times.get(i));
                velTimes[i] = (times.get(i) + times.get(i + 1)) / 2.0;
                if (velocities[i] > maxVelocity)
                    maxVelocity = velocities[i];
            }

            double[] accelerations = new double[velocities.length - 1];
            double[] accelTimes = new double[velocities.length - 1];
            double maxAccel = 0.0;

            for (int i = 0; i < accelerations.length; i++) {
                accelerations[i] = (velocities[i + 1] - velocities[i])
                        / (velTimes[i + 1] - velTimes[i]);
                accelTimes[i] = (velTimes[i] + velTimes[i + 1]) / 2.0;
                if (accelerations[i] > maxAccel)
                    maxAccel = accelerations[i];
            }

            double[] jerks = new double[accelerations.length - 1];
            double maxJerk = 0.0;

            for (int i = 0; i < jerks.length; i++) {
                jerks[i] = (accelerations[i + 1] - accelerations[i])
                        / (accelTimes[i + 1] - accelTimes[i]);
                if (jerks[i] > maxJerk)
                    maxJerk = jerks[i];
            }


            SmartDashboard.putNumber("Max Velocity", maxVelocity);
            SmartDashboard.putNumber("Max Acceleration", maxAccel);
            SmartDashboard.putNumber("Max Jerk", maxJerk);
        }
    }

    @Override
    public void autonomousInit() {
        drive.drive(0, 0);
        drive.resetEncoders();
        drive.resetHeading();
    }

    @Override
    public void teleopInit() {
        drive.drive(0, 0);
        drive.resetEncoders();
        drive.resetHeading();
    }

    @Override
    public void testInit() {
    }


    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    double lastVelocity = 0.0;
    double lastAccel = 0.0;
    double lastTime = 0.0;
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();

        double time = Timer.getFPGATimestamp();

        double velocity = (drive.getLeftVelocity() + drive.getRightVelocity()) / 2.0;
        double accel = (velocity - lastVelocity) / (time - lastTime);
        double jerk = (accel - lastAccel) / (time - lastTime);
        SmartDashboard.putNumber("Instantaneous Velocity", velocity);
        SmartDashboard.putNumber("\"Instantaneous\" Acceleration", accel);
        SmartDashboard.putNumber("\"Instantaneous\" Jerk", jerk);
        lastAccel = accel;
        lastVelocity = velocity;
        lastTime = time;
    }

    @Override
    public void testPeriodic() {
    }

    public static Controls getControls() {
        return ctrl;
    }

    public static Drivetrain getDrive() {
        return drive;
    }
}