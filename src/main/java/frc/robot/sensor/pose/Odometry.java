package frc.robot.sensor.pose;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;


import frc.robot.Constants;
import frc.robot.Subsystems.ChassisSubsystem;

import org.littletonrobotics.junction.Logger;



public class Odometry {
    
    private static final MecanumDrivePoseEstimator estimator; 
    private static ChassisSpeeds fieldRel; 
    private static final Timer timer; 



    static{
        estimator = new MecanumDrivePoseEstimator(new MecanumDriveKinematics(Constants.Mecanum.modulePositions[0], Constants.Mecanum.modulePositions[1], Constants.Mecanum.modulePositions[2], Constants.Mecanum.modulePositions[3]), Gyro.getHeading(), 
        new MecanumDriveWheelPositions(
            //TODO: get distance traveled by modules - encoders
        )
        ,  new Pose2d());
        timer = new Timer(); 
        timer.start(); 
    }

    /**
     * updates odometry pose estimate. called periodically by {@link Mecanum}
     */
    public static void updateEstimatePositions(){
       // estimator.update(Gyro.getHeading(), Mecanum.getPositions());
    }

    /**
     * pudate current position estimate w/ vision info
     * @param pose estimated {@link Pose2d} from vision frame
     * @param timestamp frame timestamp
     * @param stdDevs standard deviations of vision estimate
     */
    public static void updateEstimateVision(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs){
        estimator.addVisionMeasurement(pose, timestamp, stdDevs);
    }


    /**
     * resets robot pose estimate
     * @param pose the pose to reset to 
     */

    public static void resetPose(Pose2d pose){
     //   estimator.resetPosition(Gyro.getHeading(), Mecanum.getPositions(), pose);
    
    }

    public static void updateChassisSpeedsEstimate(ChassisSpeeds estimate){
        fieldRel = estimate; 
    }

    /**
     * get current robot pose
     */

     public static Pose2d getPose(){
        return estimator.getEstimatedPosition(); 
     }

     public static Translation2d getVelocity(){
        return new Translation2d(fieldRel.vxMetersPerSecond, fieldRel.vyMetersPerSecond);
     }

     public static void log(){
        Logger.recordOutput("Odom", estimator.getEstimatedPosition());
        Logger.recordOutput("Gyro", Gyro.getHeading());
     }
    }

