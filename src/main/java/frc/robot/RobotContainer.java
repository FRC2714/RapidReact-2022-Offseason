// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.JoystickCommand;
import frc.robot.commands.auto.*;
import frc.robot.commands.auto.FiveBallAuto;
import frc.robot.commands.auto.HelperSCurve;
import frc.robot.commands.auto.SCurve;
import frc.robot.commands.climber.MoveClimber;
import frc.robot.commands.climber.MoveClimber.ClimberState;
import frc.robot.commands.drivetrain.AutoAlign;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeCommand.IntakeState;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.subsystems.*;
import frc.robot.commands.shooter.TeleOpShooter;
import frc.robot.commands.shooter.TeleOpShooter.ShooterState;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Limelight limelight = new Limelight();
  private final Shooter shooter = new Shooter(limelight);
  private final Hood hood = new Hood(limelight);
  private final Index index = new Index();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick operatorJoystick = new Joystick(1);

  private JoystickButton driverAButton = new JoystickButton(driverJoystick, 1);
	private JoystickButton driverBButton = new JoystickButton(driverJoystick, 2);
  private JoystickButton driverXButton = new JoystickButton(driverJoystick, 3);
  private JoystickButton driverYButton = new JoystickButton(driverJoystick, 4);
  private JoystickButton driverLeftBumper = new JoystickButton(driverJoystick, 5);
  private JoystickButton driverRightBumper = new JoystickButton(driverJoystick, 6);

  private JoystickButton operatorAButton = new JoystickButton(operatorJoystick,1);
  private JoystickButton operatorBButton = new JoystickButton(operatorJoystick,2);
  private JoystickButton operatorXButton = new JoystickButton(operatorJoystick,3);
  private JoystickButton operatorYButton = new JoystickButton(operatorJoystick,4);
  private POVButton operatorDPadUp = new POVButton(operatorJoystick, 0);
	private POVButton operatorDPadLeft = new POVButton(operatorJoystick, 90);
	private POVButton operatorDPadDown = new POVButton(operatorJoystick, 180);
	private POVButton operatorDPadRight = new POVButton(operatorJoystick, 270);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new JoystickCommand(
      swerveSubsystem,
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverRightBumper.whileHeld(new AutoAlign(swerveSubsystem, limelight));
    driverLeftBumper.whileHeld(new IntakeCommand(intake, IntakeState.INTAKE, index));
    driverYButton.whenPressed(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

    operatorAButton.whileHeld(new IntakeCommand(intake, IntakeState.INTAKE, index));
    operatorBButton.whileHeld(new TeleOpShooter(shooter, ShooterState.DYNAMIC, hood, index));
    operatorXButton.whileHeld(new IntakeCommand(intake, IntakeState.EXTAKE, index));

    operatorDPadUp.whileHeld(new MoveClimber(climber, ClimberState.EXTEND));
		operatorDPadDown.whileHeld(new MoveClimber(climber, ClimberState.RETRACT));
		operatorDPadLeft.whileHeld(new MoveClimber(climber, ClimberState.REACH));
		operatorDPadRight.whileHeld(new MoveClimber(climber, ClimberState.PULL));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getSCurveAuto() {
		return new SCurve(swerveSubsystem);
	}

  public Command getHelperSCurveAuto() {
    return new HelperSCurve(swerveSubsystem);
  }

  public Command getNothingAuto() {
    return new NothingAuto();
  }

  public Command getBallStealAuto() {
    return new BallStealAuto(swerveSubsystem, limelight, shooter, hood, intake, index);
  }

  public Command getFiveBallAuto() {
    return new FiveBallAuto(swerveSubsystem, limelight, shooter, hood, intake, index);
  }
}

