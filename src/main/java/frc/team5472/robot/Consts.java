package frc.team5472.robot;

public class Consts {

    public static final double WHEEL_DIAMETER = 0.1016;
    public static final double ROBOT_WHEELBASE_WIDTH = 0.6731;

    public static final int TICKS_PER_REV = 4096;
    public static final double TICKS_PER_METER = TICKS_PER_REV / WHEEL_DIAMETER / Math.PI;


    public static final int DRIVE_LEFT_TALON_CAN = 1;
    public static final int DRIVE_LEFT_FOLLOWER_CAN = 2;
    public static final int DRIVE_RIGHT_TALON_CAN = 4;
    public static final int DRIVE_RIGHT_FOLLOWER_CAN = 3;
    public static final int DRIVE_SHIFT_SOLENOID = 0;
    public static final int JOY_ID = 0;
}
