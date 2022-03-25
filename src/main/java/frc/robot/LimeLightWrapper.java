
package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLightWrapper {

    public static double getX() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }
    
    public static double getY() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }

    public static double getZ() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tz").getDouble(0);
    }
    
    public static double getLatency() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0);
    }
    
    public static double getArea() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    }
    
    public static boolean hasTarget() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) != 0;
    }

    public static double getSkewAngle() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
    }

    public static double[] getCamtranData() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray((double[])null);
    }

    public static void turnOFF(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }
    
    public static void turnON(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    }
    
    public static void setLed(double state) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(state);
        SmartDashboard.putNumber("LimeLight State", state);
    }

    public static double getDistanceToHighTarget() {
        return 73.5d / Math.tan(getCamtranData()[4] + 35.0d);
    }
}
