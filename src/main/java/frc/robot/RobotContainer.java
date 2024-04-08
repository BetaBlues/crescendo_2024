// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.Constants.k_chassis;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.k_xbox;
// import frc.robot.commands.AutonomousCommand;
import frc.robot.Commands.DriveCommand;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.ChassisSubsystem;
import frc.robot.Subsystems.ClimbingSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
//import frc.robot.Subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.Joystick.AxisType;
// import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.PneumaticsModuleType;



public class RobotContainer {
  public final ChassisSubsystem m_chassis = Constants.hasDrive ?  new ChassisSubsystem() : null;
  private final IntakeSubsystem m_IntakeSubsystem = Constants.hasIntake ? new IntakeSubsystem() : null;
  private final ArmSubsystem m_ArmSubsystem = Constants.hasArm ? new ArmSubsystem() : null;
  private final ClimbingSubsystem m_ClimbingSubsystem = Constants.hasPiston ? new ClimbingSubsystem(PneumaticsModuleType.CTREPCM, 0, 1) : null;
  
  // private Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);

  //creates controller
  XboxController m_chassisController = new XboxController(1); //connect XboxController to port 1
  XboxController m_MechanismController = new XboxController(0); //connect XboxController to port 0
  // ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kMXP); // Creates an ADXRS450_Gyro object on the onboard SPI port
  
  public RobotContainer() {
    configureButtonBindings();
    
    if (Constants.hasDrive)
    { 
     m_chassis.setDefaultCommand(new DriveCommand(m_chassis, m_chassisController));
    //new method of moving chassis. Eliminates need for Chassis subsystem because the chassis is the defult command
    
   /* 
      m_chassis.setDefaultCommand(new RunCommand(() -> m_chassis.driveCartesian(
          m_chassisController.getRawAxis(k_xbox.leftYAxis) * Constants.k_chassis.normalDriveSpeed,
          -m_chassisController.getRawAxis(k_xbox.leftXAxis) * Constants.k_chassis.normalDriveSpeed,
          m_chassisController.getRawAxis(k_xbox.rightXAxis) * -1 * Constants.k_chassis.normalRotationSpeed), m_chassis)); //eventually should add the gyro sensor as a 4th parameter. This will make feild orriented drive work.
   */
        }
    if (Constants.hasPiston)
    {
      new JoystickButton(m_MechanismController, k_xbox.buttonX).onTrue(new InstantCommand(() -> m_ClimbingSubsystem.toggleCommand())); 
      // m_ClimbingSubsystem.setDefaultCommand(new RunCommand(() -> m_ClimbingSubsystem.disableCompressor()));
    }
  }
  
  private void configureButtonBindings() {
    if(Constants.hasDrive){

      new JoystickButton(m_chassisController, XboxController.Button.kA.value).onTrue(new InstantCommand(() -> m_chassis.stopComplete()));
    }
    
    if (Constants.hasIntake)
    {
      //shooting/loading joystick
      // final Joystick leftJoystick = new Joystick(0);
      m_IntakeSubsystem.setDefaultCommand(new InstantCommand(() -> m_IntakeSubsystem.IntakeSpeed(m_MechanismController.getRightY()), m_IntakeSubsystem));
    }

    if(Constants.hasArm)
    {
      // final Joystick rightJoystick = new Joystick(5);
      m_ArmSubsystem.setDefaultCommand(
        Commands.parallel(new RunCommand(() -> m_ArmSubsystem.MoveArm(m_MechanismController.getLeftY()), m_ArmSubsystem), new RunCommand(() -> m_ArmSubsystem.runAutomatic(), m_ArmSubsystem)));

      m_ArmSubsystem.offsetPosition();


      //brake motors 

     
      //arm position                                                                                                                                                      --
      // //loading
     new JoystickButton(m_MechanismController, XboxController.Button.kA.value).onTrue(new InstantCommand(() -> m_ArmSubsystem.position_Loading()));
      // //shooting
      new JoystickButton(m_MechanismController, XboxController.Button.kB.value).onTrue(new InstantCommand(() -> m_ArmSubsystem.position_Shooting()));
      // // //resting
       new JoystickButton(m_MechanismController, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> m_ArmSubsystem.position_Resting()));

      new JoystickButton(m_MechanismController, XboxController.Button.kB.value).onTrue(new InstantCommand(() -> m_ArmSubsystem.dec_setpoint()));
       new JoystickButton(m_MechanismController, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> m_ArmSubsystem.inc_setpoint()));

       //before or after button config? --> believe after
    }

    if (Constants.hasSeesaw)
    {
      new JoystickButton(m_MechanismController, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> m_ArmSubsystem.moveToSawPosition()));
      new JoystickButton(m_MechanismController, XboxController.Button.kA.value).onTrue(new InstantCommand(() -> m_ArmSubsystem.moveToSeePosition()));
      m_ArmSubsystem.setDefaultCommand(new RunCommand(() -> m_ArmSubsystem.runAutomatic(), m_ArmSubsystem)); //before or after button config? --> believe after
    }
  }

  


  
 public Command getAutonomousCommand() {
  Command manualLeave = new RunCommand(() -> m_chassis.driveCartesian(0.3, 0, 0), m_chassis).withTimeout(2.5); 
  
  return manualLeave;
 }

}