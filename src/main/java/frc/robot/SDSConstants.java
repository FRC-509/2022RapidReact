package frc.robot;

public class SDSConstants {
    private static final double TICKS_PER_ROTATION = 2048.0;
    public static final double MK4_L1_DRIVE_REDUCTION = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
    public static final double MK4_L1_GEAR_RATIO = 1.0 / MK4_L1_DRIVE_REDUCTION;
    public static final double MK4_L1_WHEEL_DIAMETER = 0.10033;
    public static final double MK4_L1_WHEEL_CIRCUMFERENCE = MK4_L1_WHEEL_DIAMETER * Math.PI;
    public static final double SENSOR_POSITION_COEFF = MK4_L1_WHEEL_CIRCUMFERENCE * MK4_L1_DRIVE_REDUCTION / TICKS_PER_ROTATION;
    public static final double SENSOR_VELOCITY_COEFF = SENSOR_POSITION_COEFF * 10.0;
    public static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * MK4_L1_DRIVE_REDUCTION * MK4_L1_WHEEL_CIRCUMFERENCE;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);
    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
    public static final boolean USE_INTEGRATED_PID = true;
}
