package frc.team5472.robot.teleop.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import frc.team5472.robot.Drivetrain;
import frc.team5472.robot.Robot;

public class JoystickCommand extends Command {

    private Drivetrain drive;
    private XboxController instance;
    private boolean finished = false;

    public JoystickCommand(){
        drive = Robot.getDrive();
        instance = Robot.getControls().getDrivePad();
        requires(drive);
    }

    protected void execute(){
        double throttle = -instance.getY(GenericHID.Hand.kLeft);
        double twist = instance. getX(GenericHID.Hand.kLeft);

        drive.drive(throttle + twist, throttle - twist);
    }

    protected boolean isFinished(){return finished;}
}
