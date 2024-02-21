// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.Constants.k_chassis;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.k_xbox;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.Joystick.AxisType;
// import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class RobotContainer {
  private final ChassisSubsystem m_chassis = new ChassisSubsystem();
  private final AutonomousCommand m_autoCommand = new AutonomousCommand();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ClimbingSubsystem ClimbingSubsystem = new ClimbingSubsystem(PneumaticsModuleType.CTREPCM, 1, 0);
  

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
 
    new JoystickButton(m_chassisController, k_xbox.buttonA).onTrue(new InstantCommand(() -> ClimbingSubsystem.toggleCommand())); 
  }
  
  private void configureButtonBindings() {
    //add button bindings here

    //shooting/loading joystick
    final Joystick leftJoystick = new Joystick(1);
      //number = port
    
//    m_chassis.setDefaultCommand(new RunCommand(() -> m_IntakeSubsystem.IntakeSpeed(leftJoystick.getY()), m_IntakeSubsystem));
    //another way to do it (using raw axis):
    //shoulderSubsystem.setDefaultCommand(new RunCommand(() -> shoulderSubsystem.move(manipulator.getRawAxis(xboxConstants.rightYAxis) * 0.15), shoulderSubsystem));

    //arm buttons
// new JoystickButton(manipulator, XboxController.Button.kRightBumper.value)
// .onTrue(new RunCommand(() -> gripper.move(Constants.gripperConstants.gripperOpenSpeed), gripper)).onFalse(new RunCommand(() -> gripper.stop(), gripper));
// new JoystickButton(manipulator, XboxController.Button.kLeftBumper.value)

    //final Button aButton = new Button(m_MechanismController, k_xbox.buttonA);
    // final Button bButton = new Button(m_MechanismController, k_xbox.buttonB);
    // final Button yButton = new Button(m_MechanismController, k_xbox.buttonY);
    
    //loading
    new JoystickButton(m_MechanismController, XboxController.Button.kA.value).onTrue(new RunCommand(() -> m_IntakeSubsystem.setPosition(IntakeConstants.p_loading))).onFalse(new RunCommand(() -> m_IntakeSubsystem.stopArm()));
    //shooting
    new JoystickButton(m_MechanismController, XboxController.Button.kY.value).onTrue(new RunCommand(() -> m_IntakeSubsystem.setPosition(IntakeConstants.p_shooting))).onFalse(new RunCommand(() -> m_IntakeSubsystem.stopArm()));
    //resting
    new JoystickButton(m_MechanismController, XboxController.Button.kB.value).onTrue(new RunCommand(() -> m_IntakeSubsystem.setPosition(IntakeConstants.p_resting))).onFalse(new RunCommand(() -> m_IntakeSubsystem.stopArm()));
    

  }


  public Command getAutonomousCommand() {
    return m_autoCommand;
  }

}