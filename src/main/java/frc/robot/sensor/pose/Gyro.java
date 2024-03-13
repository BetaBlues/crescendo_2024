package frc.robot.sensor.pose;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utility.conversion.AngleUtil;



public class Gyro {
    

    public enum GyroMode{
        Primary, //primary gyro
        Estimation //pose estimation - second line of defense
    }

    private static GyroMode mode = GyroMode.Primary; 
    
    //primary gryo
    private static AHRS navx; 

    private static Rotation2d estimate = Rotation2d.fromDegrees(0); 

    static{
        try{
            navx = new AHRS(I2C.Port.kMXP);
        }
        catch (Exception e){
            mode = GyroMode.Estimation; 
            System.out.println("Failed to connect to NavX");
        }
    }

    /**
     * updates rotation estimate to current of the robot position since last tick
     * 
     */
    public static void updateEstimationDelta(Rotation2d estimateDelta){
        estimate = Rotation2d.fromDegrees(estimate.getDegrees() + estimateDelta.getDegrees());
    }

    public static void resetGyro(){
        navx.zeroYaw();
    }

    public static GyroMode getMode(){
        return mode; 
    }

    /**
     * returns current robot heading
     */
    public static Rotation2d getHeading(){
        return switch (mode){
            case Primary -> Rotation2d.fromRotations(AngleUtil.unsignedRangeRotations(navx.getYaw() /  360));
            case Estimation -> estimate; 
        };
    }
}