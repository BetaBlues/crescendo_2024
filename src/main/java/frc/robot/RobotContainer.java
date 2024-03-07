// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final ChassisSubsystem m_chassis = new ChassisSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final ClimbingSubsystem m_ClimbingSubsystem = new ClimbingSubsystem(PneumaticsModuleType.CTREPCM, 0, 1);
  
  private Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);

  //creates controller
  XboxController m_chassisController = new XboxController(1); //connect XboxController to port 1
  XboxController m_MechanismController = new XboxController(0); //connect XboxController to port 0
  ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kMXP); // Creates an ADXRS450_Gyro object on the onboard SPI port
  
  public RobotContainer() {
    configureButtonBindings();
    
    m_chassis.setDefaultCommand(new DriveCommand(m_chassis, m_chassisController));
    //new method of moving chassis. Eliminates need for Chassis subsystem because the chassis is the defult command
    
    m_chassis.setDefaultCommand(new RunCommand(() -> m_chassis.driveCartesian(
        m_chassisController.getRawAxis(k_xbox.leftYAxis) * Constants.k_chassis.normalDriveSpeed,
        m_chassisController.getRawAxis(k_xbox.leftXAxis) * Constants.k_chassis.normalDriveSpeed,
        m_chassisController.getRawAxis(k_xbox.rightXAxis) * Constants.k_chassis.normalRotationSpeed, 
        gyro.getRotation2d()), m_chassis)); //eventually should add the gyro sensor as a 4th parameter. This will make feild orriented drive work.
 
    new JoystickButton(m_MechanismController, k_xbox.buttonX).onTrue(new InstantCommand(() -> m_ClimbingSubsystem.toggleCommand())); 
    m_ClimbingSubsystem.setDefaultCommand(new RunCommand(() -> m_ClimbingSubsystem.disableCompressor()));

  }
  
  private void configureButtonBindings() {

    //shooting/loading joystick
    final Joystick leftJoystick = new Joystick(0);
    m_IntakeSubsystem.setDefaultCommand(new RunCommand(() -> m_IntakeSubsystem.IntakeSpeed(leftJoystick.getY()), m_IntakeSubsystem));
    
    // m_ArmSubsystem.offsetPosition();

    //arm position
    //loading
    // new JoystickButton(m_MechanismController, XboxController.Button.kA.value).onTrue(new InstantCommand(() -> m_ArmSubsystem.position_Loading()));
    // //shooting
    // new JoystickButton(m_MechanismController, XboxController.Button.kB.value).onTrue(new InstantCommand(() -> m_ArmSubsystem.position_Shooting()));
    // //resting
    // new JoystickButton(m_MechanismController, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> m_ArmSubsystem.position_Resting()));

    // m_ArmSubsystem.setDefaultCommand(new RunCommand(() -> m_ArmSubsystem.runAutomatic(), m_ArmSubsystem)); //before or after button config? --> believe after

    //increase setpoint by 500
    // new JoystickButton(m_MechanismController, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> m_ArmSubsystem.inc_setpoint()));

    // //decrease setpoint by 500
    // new JoystickButton(m_MechanismController, XboxController.Button.kA.value).onTrue(new InstantCommand(() -> m_ArmSubsystem.dec_setpoint()));

  }

  


 // public Command getAutonomousCommand() {
 //   return m_autoCommand;
 // }

}