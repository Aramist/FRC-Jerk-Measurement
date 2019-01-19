package frc.team5472.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Controls {

    private XboxController drivePad;

    public Controls(){
        drivePad = new XboxController(Consts.JOY_ID);
    }

    public XboxController getDrivePad(){
        return drivePad;
    }
}
