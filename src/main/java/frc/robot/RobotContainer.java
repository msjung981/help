// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.Climber;
import frc.robot.subsystems.swervedrive.Intake;
import frc.robot.subsystems.swervedrive.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  private final Intake m_Intake = Intake.getInstance();
  private final Shooter m_Shooter = Shooter.getInstance();
  private final Climber m_Climber = Climber.getInstance();

  
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);
  
 
public Command getAutonomousCommand() {
    //return drivebase.getAutonomousCommand("Shoot+AnotherTaxi");
    return drivebase.getAutonomousCommand("Shoot+ThirdPath");
}
  



 

   
 
  public RobotContainer()
  {
    NamedCommands.registerCommand("stopfeed", m_Shooter.stopFeederCommand());
    NamedCommands.registerCommand("AutoFeed", m_Shooter.getFeederCommand2());
    NamedCommands.registerCommand("AutoShoot", m_Shooter.getShooterCommand());
     NamedCommands.registerCommand("Auto1", m_Shooter.getShooterCommand());
      NamedCommands.registerCommand("Auto2", m_Shooter.getFeederCommand2());

    NamedCommands.registerCommand("AutoShootandFeed", m_Shooter.getFeederCommand2().withTimeout(1.5)
      .alongWith(m_Shooter.getShooterCommand().withTimeout(1.5)).andThen(m_Shooter.stopFeederCommand()));
    NamedCommands.registerCommand("RunIntake", m_Intake.getIntakeCommand().withTimeout(4));

    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed);

    
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
    () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    () -> driverXbox.getRightX()); //was MULTIPLIED BY 0.5

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    

    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    driverXbox.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));
    
    operatorXbox.a().whileTrue(m_Intake.getIntakeCommand());
    operatorXbox.b().onTrue(m_Shooter.getFeederCommand2()).onFalse(
      new InstantCommand(()->{
        m_Shooter.stopShooter();
        m_Shooter.stopFeeder();
      }));
    operatorXbox.rightTrigger().whileTrue(m_Shooter.getShooterCommand());
    operatorXbox.leftTrigger().whileTrue(m_Shooter.getShooterCommand2());

    operatorXbox.x().whileTrue(m_Climber.getClimberCommand());
    operatorXbox.rightBumper().whileTrue(m_Climber.retractRightClimber());
    operatorXbox.leftBumper().whileTrue(m_Climber.retractLeftClimber());
  }

 

  public void setDriveMode()
  {
    
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
  



  
  
  
}
