// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.settings.Constants.PS4Driver.*;
import static frc.robot.settings.Constants.ShooterConstants.PRAC_AMP_RPS;
import static frc.robot.settings.Constants.ShooterConstants.LONG_SHOOTING_RPS;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.management.InstanceNotFoundException;

import static frc.robot.settings.Constants.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.commands.AimRobotMoving;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveTimeCommand;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.settings.Constants.Field;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.commands.MoveMeters;
import frc.robot.commands.WaitUntil;


import frc.robot.commands.goToPose.GoToAmp;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

// preferences are information saved on the Rio. They are initialized once, then gotten every time we run the code.
  private final boolean intakeExists = Preferences.getBoolean("Intake", true);
  private final boolean shooterExists = Preferences.getBoolean("Shooter", true);
  private final boolean angleShooterExists = Preferences.getBoolean("AngleShooter", true);
  private final boolean climberExists = Preferences.getBoolean("Climber", true);
  private final boolean lightsExist = Preferences.getBoolean("Lights", true);
  private final boolean indexerExists = Preferences.getBoolean("Indexer", true);
  private final boolean sideWheelsExists = Preferences.getBoolean("IntakeSideWheels", true);
  //private final boolean autosExist = Preferences.getBoolean("Autos", true);
  private final boolean useDetectorLimelight = Preferences.getBoolean("Detector Limelight", true);

  private DrivetrainSubsystem driveTrain;
  
  private Drive defaultDriveCommand;
  private PS4Controller driverController;
  private PS4Controller operatorController;
  //private PS4Controller operatorController;
  private Limelight limelight;
  private SendableChooser<String> climbSpotChooser;
  private SendableChooser<Command> autoChooser;
  private PowerDistribution PDP;

  BooleanSupplier ZeroGyroSup;
  BooleanSupplier AimWhileMovingSup;
  BooleanSupplier ShootIfReadySup;
  BooleanSupplier SubwooferAngleSup;
  BooleanSupplier StageAngleSup;
  BooleanSupplier HumanPlaySup;
  BooleanSupplier AmpAngleSup;
  BooleanSupplier SourcePickupSup;
  BooleanSupplier ClimberDownSup;
  BooleanSupplier ShooterUpManualSup;
  BooleanSupplier ManualShootSup;
  BooleanSupplier ForceVisionSup;
  BooleanSupplier GroundIntakeSup;
  BooleanSupplier FarStageAngleSup;
  BooleanSupplier OperatorRevToZero;
  BooleanSupplier OverStagePassSup;
  BooleanSupplier OppositeStageShotSup;
  BooleanSupplier falseSup;
  DoubleSupplier zeroSup;
  BooleanSupplier AutoPickupSup;
  BooleanSupplier CenterAmpPassSup;

  BooleanSupplier intakeReverse;
  Command autoPickup;
  Command podiumAutoPickup;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //preferences are initialized IF they don't already exist on the Rio
    SmartDashboard.putNumber("amp RPS", ShooterConstants.PRAC_AMP_RPS);

    Preferences.initBoolean("Brushes", false);
    Preferences.initBoolean("CompBot", true);
    Preferences.initBoolean("Climber", true);
    Preferences.initBoolean("Intake", true);
    Preferences.initBoolean("IntakeSideWheels", false);
    Preferences.initBoolean("Shooter", true);
    Preferences.initBoolean("AngleShooter", true);
    Preferences.initBoolean("Lights", true);
    Preferences.initBoolean("Indexer", true);
    Preferences.initBoolean("Detector Limelight", false);
    Preferences.initBoolean("Use Limelight", true);
    Preferences.initBoolean("Use 2 Limelights", true);
    Preferences.initDouble("wait # of seconds", 0);

    DataLogManager.start(); //Start logging
    DriverStation.startDataLog(DataLogManager.getLog()); //Joystick Data logging

    // DataLogManager.start();
    // URCL.start();
    // SignalLogger.setPath("/media/sda1/ctre-logs/");
    // SignalLogger.start();
    driverController = new PS4Controller(DRIVE_CONTROLLER_ID);
    operatorController = new PS4Controller(OPERATOR_CONTROLLER_ID);
    //operatorController = new PS4Controller(OPERATOs_CONTROLLER_ID);
    PDP = new PowerDistribution(1, ModuleType.kRev);

    ZeroGyroSup = driverController::getPSButton;
    ForceVisionSup = driverController::getOptionsButton;

    AimWhileMovingSup = driverController::getL2Button;
    HumanPlaySup = driverController::getR1Button;
    AmpAngleSup = ()->driverController.getPOV() == 90||driverController.getPOV() == 45||driverController.getPOV() == 135;;
    ManualShootSup = driverController::getR2Button;
    ClimberDownSup = operatorController::getPSButton;
    GroundIntakeSup = driverController::getL1Button;
    OperatorRevToZero = ()->operatorController.getPOV() != -1;
    SubwooferAngleSup =()-> driverController.getCrossButton()||operatorController.getCrossButton();
    StageAngleSup = ()->operatorController.getTriangleButton()||driverController.getTriangleButton();;
    FarStageAngleSup = ()->operatorController.getSquareButton()||driverController.getSquareButton();
    OppositeStageShotSup = ()->operatorController.getCircleButton()||driverController.getCircleButton();
    OverStagePassSup = operatorController::getL1Button;
    CenterAmpPassSup = operatorController::getL2Button;
    AutoPickupSup = ()->operatorController.getTouchpad()||driverController.getTouchpad();
    zeroSup = ()->0;
    falseSup = ()->false;
    //discontinued buttons:
    intakeReverse = ()->false;
    ShooterUpManualSup = ()->false;
    ForceVisionSup = ()->false;
    ShootIfReadySup = ()->false;
    
    // = new PathPlannerPath(null, DEFAUL_PATH_CONSTRAINTS, null, climberExists);
    limelightInit();
    driveTrainInst();

    // Configure the trigger bindings
  }

  private void driveTrainInst() {
    driveTrain = new DrivetrainSubsystem();
    defaultDriveCommand = new Drive(
      driveTrain, 
      () -> false,
      () -> modifyAxis(-driverController.getRawAxis(Y_AXIS), DEADBAND_NORMAL),
      () -> modifyAxis(-driverController.getRawAxis(X_AXIS), DEADBAND_NORMAL),
      () -> modifyAxis(-driverController.getRawAxis(Z_AXIS), DEADBAND_NORMAL));
      driveTrain.setDefaultCommand(defaultDriveCommand);
  }


  private void autoInit() {
    configureDriveTrain();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  private void limelightInit() {
    limelight = Limelight.getInstance();
  }
  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new Trigger(AmpAngleSup).onTrue(new InstantCommand(driveTrain::pointWheelsInward, driveTrain));
    SmartDashboard.putData("drivetrain", driveTrain);
    Command setGyroTo180 = new InstantCommand(()->driveTrain.zeroGyroscope(180)) {
      public boolean runsWhenDisabled() {
              return true;
      };
    };
    SmartDashboard.putData("set gyro 180", setGyroTo180);
    // new Trigger(driverController::getCrossButton).onTrue(new autoAimParallel(driveTrain/*, shooter*/));
    new Trigger(ZeroGyroSup).onTrue(new InstantCommand(driveTrain::zeroGyroscope));
    // new Trigger(driverController::getCircleButton).whileTrue(new GoToAmp(driveTrain)); unused becuase we dont pickup from amp with a path anymore
    new Trigger(AimWhileMovingSup).whileTrue(new AimRobotMoving(
      driveTrain,
      () -> modifyAxis(-driverController.getRawAxis(Z_AXIS), DEADBAND_NORMAL),
      () -> modifyAxis(-driverController.getRawAxis(Y_AXIS), DEADBAND_NORMAL),
      () -> modifyAxis(-driverController.getRawAxis(X_AXIS), DEADBAND_NORMAL),
      driverController::getL2Button,
      StageAngleSup,
      FarStageAngleSup,
      SubwooferAngleSup,
      OverStagePassSup,
      OppositeStageShotSup,
      falseSup
      ));
    // new Trigger(()->driverController.getL3Button()&&driverController.getR3Button()).onTrue(
    //   new SequentialCommandGroup(
    //     new InstantCommand(()->angleShooterSubsystem.setDesiredShooterAngle(22.5), angleShooterSubsystem),
    //     new ParallelRaceGroup(
    //       new AimRobotMoving(driveTrain, zeroSup, zeroSup, zeroSup, ()->true, falseSup, falseSup, falseSup, falseSup, falseSup, ()->true),
    //       new ShootNote(indexer, 0.65, 0.4, intake)
    //     )
    //   )
    // );

    

    SmartDashboard.putData("move 1 meter", new MoveMeters(driveTrain, 1, 0.2, 0.2, 0.2));
    InstantCommand setOffsets = new InstantCommand(driveTrain::setEncoderOffsets) {
      public boolean runsWhenDisabled() {
        return true;
      };
    };
    SmartDashboard.putData("set offsets", setOffsets);
    SmartDashboard.putData(new InstantCommand(driveTrain::forceUpdateOdometryWithVision));
/* bindings:
 *    L2: aim at speaker and rev up shooter to max (hold)
 *    L1: manually feed shooter (hold)
 *    R2: shoot if everything is lined up (hold)
 *    Circle: lineup with the amp +shoot at amp speed (hold)
 *    D-Pad down: move shooter up manually (hold)
 *    R1: aim shooter at amp (hold)
 *    Options button: collect note from human player
 *    Square: auto-pick up note
 *    Touchpad: manually turn on Intake (hold) [only works if intake code doesn't exist in IndexCommand]
 *    L1,L2,R1,R2 held: aim shooter at speaker and set shooter to shooter speed
 * 
 *  operator:
 *    Triangle: climber up (hold)
 *    Cross: climber down (hold)
 *    R1: auto pickup note from ground (hold)
 *    
 */
//FOR TESTING PURPOSES:
  };


    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void autonomousInit() {
    SmartDashboard.putNumber("autos ran", SmartDashboard.getNumber("autos ran", 0)+1);
  }
  private double modifyAxis(double value, double deadband) {
    // Deadband
    value = MathUtil.applyDeadband(value, deadband);
    // Square the axis
    value = Math.copySign(value * value, value);
    return value;
  }

  private void configureDriveTrain() {
    AutoBuilder.configureHolonomic(
                driveTrain::getPose, // Pose2d supplier
                driveTrain::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
                driveTrain::getChassisSpeeds,
                driveTrain::driveWhileAimed,
                new HolonomicPathFollowerConfig(
                    new PIDConstants(
                        k_XY_P,
                        k_XY_I,
                        k_XY_D), // PID constants to correct for translation error (used to create the X
                                 // and Y PID controllers)
                    new PIDConstants(
                        k_THETA_P,
                        k_THETA_I,
                        k_THETA_D), // PID constants to correct for rotation error (used to create the
                                    // rotation controller)
                    4, //max module speed //TODO find actual values
                    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0).getNorm(), //drive base radius
                    new ReplanningConfig()
                ),
                ()->DriverStation.getAlliance().get().equals(Alliance.Red),
                driveTrain
    );
  }

  public void teleopInit() {

  }
  public void teleopPeriodic() {
    SmartDashboard.putData(driveTrain.getCurrentCommand());
    driveTrain.calculateSpeakerAngle();
    if(useDetectorLimelight) {
      SmartDashboard.putNumber("Is Note Seen?", limelight.getNeuralDetectorValues().ta);
      RobotState.getInstance().IsNoteSeen = limelight.getNeuralDetectorValues().isResultValid;
    } else {
      RobotState.getInstance().IsNoteSeen = false;
    }
    SmartDashboard.putBoolean("is note seen", RobotState.getInstance().IsNoteSeen);
		SmartDashboard.putBoolean("shooter in range", RobotState.getInstance().ShooterInRange);
  }
 
  public void logPower(){
    for(int i = 0; i < 16; i++) { 
      SmartDashboard.putNumber("PDP Current " + i, PDP.getCurrent(i));
    }
  }
  public void robotPeriodic() {
    // logPower();
  }
  public void disabledPeriodic() {
  
  }

  public void disabledInit() {
  }
}
