package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.ChassisSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.utility.generator.AutoAlign;

public class AlignAutos {
    
    public static Command alignAmp(ChassisSubsystem chassis){        
        
        Pose2d poseAmp = new Pose2d(Constants.FieldConstants.getAmp(), new Rotation2d());
        Pose2d poseRobot = new Pose2d();  //TODO: gyror get robot pose
        Command alignAmp = AutoAlign.withNav(poseAmp); 

        return alignAmp;
    }


}
