package frc.robot.sensor.vision;

import static frc.robot.Constants.aprilTagFieldLayout;
import static frc.robot.sensor.vision.AprilTagConstants.*; 
import static java.lang.System.arraycopy;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.sensor.vision.AprilTagConstants.CameraResolution;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class AprilTagVisionRobot implements VisionAprilTag{

    //front facing camera
    private final PhotonCamera frontCam; 
    private final PhotonPoseEstimator frontPhotonPoseEstimator; 

    //left facing camera
    private final PhotonCamera leftCam; 
    private final PhotonPoseEstimator leftPhotonPoseEstimator; 

    //right facing camera
    private final PhotonCamera rightCam; 
    private final PhotonPoseEstimator rightPhotonPoseEstimator; 

    private Pose3d[] poseArray = new Pose3d[3];
    private double[] timestampArray = new double[3];
    private double[] visionStdArray = new double[9];

    public AprilTagVisionRobot(){
        
        
        frontCam = new PhotonCamera("front");
        frontPhotonPoseEstimator = 
        new PhotonPoseEstimator(aprilTagFieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, frontCam, frontCamToRobot);

        leftCam = new PhotonCamera("left");
        leftPhotonPoseEstimator = 
        new PhotonPoseEstimator(aprilTagFieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, leftCam, leftCamToRobot);
        
        rightCam = new PhotonCamera("right");
        rightPhotonPoseEstimator = 
        new PhotonPoseEstimator(aprilTagFieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, rightCam, rightCamToRobot);
    
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs){
        getEstimatedPoseUpdates();
        inputs.visionPoses = poseArray;
        inputs.timestamps = timestampArray;
        inputs.visionStdDevs = visionStdArray;
    }
    
  public void getEstimatedPoseUpdates() {
    Matrix<N3, N1> infiniteStdevs =
        VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    Optional<EstimatedRobotPose> pose = frontPhotonPoseEstimator.update();
    
    
    pose.ifPresentOrElse(
        estimatedRobotPose -> {
          poseArray[0] = estimatedRobotPose.estimatedPose;
          timestampArray[0] = estimatedRobotPose.timestampSeconds;
          Matrix<N3, N1> stdDevs =
              getEstimationStdDevs(estimatedRobotPose, CameraResolution.HIGH_RES);
          arraycopy(stdDevs.getData(), 0, visionStdArray, 0, 3);
        },

        () -> {
          poseArray[0] = new Pose3d();
          timestampArray[0] = 0.0;
          arraycopy(infiniteStdevs.getData(), 0, visionStdArray, 0, 3);
        });


    pose = leftPhotonPoseEstimator.update();
    pose.ifPresentOrElse(
        estimatedRobotPose -> {
          poseArray[1] = estimatedRobotPose.estimatedPose;
          timestampArray[1] = estimatedRobotPose.timestampSeconds;
          Matrix<N3, N1> stdDevs =
              getEstimationStdDevs(
                  estimatedRobotPose,
                    CameraResolution.NORMAL);
                  
          arraycopy(stdDevs.getData(), 0, visionStdArray, 3, 3);
        },

        () -> {
          poseArray[1] = new Pose3d();
          timestampArray[1] = 0.0;
          arraycopy(infiniteStdevs.getData(), 0, visionStdArray, 0, 3);
        });



    pose = rightPhotonPoseEstimator.update();
    pose.ifPresentOrElse(
        estimatedRobotPose -> {
          poseArray[2] = estimatedRobotPose.estimatedPose;
          timestampArray[2] = estimatedRobotPose.timestampSeconds;
          Matrix<N3, N1> stdDevs =
              getEstimationStdDevs(estimatedRobotPose, CameraResolution.NORMAL);
          arraycopy(stdDevs.getData(), 0, visionStdArray, 6, 3);
        },
        () -> {
          poseArray[2] = new Pose3d();
          timestampArray[2] = 0.0;
          arraycopy(infiniteStdevs.getData(), 0, visionStdArray, 0, 3);
        });
    }
}