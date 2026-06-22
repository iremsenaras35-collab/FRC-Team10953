// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final SparkMax leftDriveLead = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax leftDriveFollow = new SparkMax(4, MotorType.kBrushed);
  private final SparkMax rightDriveLead = new SparkMax(1, MotorType.kBrushed);
  private final SparkMax rightDriveFollow = new SparkMax(2, MotorType.kBrushed);

  private final SparkMax feederRoller = new SparkMax(5, MotorType.kBrushed);
  private final SparkMax intakeAndLauncherRoller = new SparkMax(6, MotorType.kBrushed);

  private final DifferentialDrive LyronDrive = new DifferentialDrive(leftDriveLead, rightDriveLead);

  private final Timer autoTimer = new Timer();
  private final Timer spinUpTimer = new Timer();
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(1.5); 
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(2.5);  

  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);

  private static final double INTAKING_INTAKE_VOLTAGE = 9.0;
  private static final double INTAKING_FEEDER_VOLTAGE = -12.0;
  private static final double LAUNCHING_LAUNCHER_VOLTAGE = 11.6;
  private static final double LAUNCHING_FEEDER_VOLTAGE = 9.0;
  private static final double SPIN_UP_FEEDER_VOLTAGE = -11.0;
  private static final double SPIN_UP_SECONDS = 1.0;

  private static final double AUTO_DRIVE_SECONDS   = 0.5;
  private static final double AUTO_TURN_SECONDS    = 0.0;
  private static final double AUTO_DRIVE_2_SECONDS = 0.0;
  private static final double AUTO_SPIN_UP_SECONDS = 2.0;
  private static final double AUTO_LAUNCH_SECONDS  = 1.0;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("My Auto", kCustomAuto);
    m_chooser.addOption("Default Auto", kDefaultAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    SparkMaxConfig leftLeadConfig = new SparkMaxConfig();
    leftLeadConfig.voltageCompensation(12);
    leftLeadConfig.smartCurrentLimit(60);
    leftDriveLead.configure(leftLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig leftFollowConfig = new SparkMaxConfig();
    leftFollowConfig.voltageCompensation(12);
    leftFollowConfig.smartCurrentLimit(60);
    leftFollowConfig.follow(leftDriveLead);
    leftDriveFollow.configure(leftFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightLeadConfig = new SparkMaxConfig();
    rightLeadConfig.voltageCompensation(12);
    rightLeadConfig.smartCurrentLimit(60);
    rightLeadConfig.inverted(true);
    rightDriveLead.configure(rightLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightFollowConfig = new SparkMaxConfig();
    rightFollowConfig.voltageCompensation(12);
    rightFollowConfig.smartCurrentLimit(60);
    rightFollowConfig.follow(rightDriveLead);
    rightDriveFollow.configure(rightFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    if (m_autoSelected == null) {
        m_autoSelected = kCustomAuto;
    }
    autoTimer.reset();
    autoTimer.start();
  }

  @Override
  public void autonomousPeriodic() {
    double time = autoTimer.get();

    switch (m_autoSelected) {
      case kDefaultAuto:
        if (time < 1.0) {
          LyronDrive.tankDrive(0.5, 0.5); 
        } else if (time < 1.7) { 
          LyronDrive.tankDrive(0.5, -0.5); 
        } else {
          LyronDrive.stopMotor();
        }
        break;

      case kCustomAuto:
      default:
        double phase1End = AUTO_DRIVE_SECONDS;
        double phase2End = phase1End + AUTO_TURN_SECONDS;
        double phase3End = phase2End + AUTO_DRIVE_2_SECONDS;
        double phase4End = phase3End + AUTO_SPIN_UP_SECONDS;
        double phase5End = phase4End + AUTO_LAUNCH_SECONDS;

        if (time < phase1End) {
          LyronDrive.arcadeDrive(0.6, 0); 
          intakeAndLauncherRoller.stopMotor();
          feederRoller.stopMotor();
        }
        else if (time < phase2End) {
          LyronDrive.arcadeDrive(0, 0.5);
          intakeAndLauncherRoller.stopMotor();
          feederRoller.stopMotor();
        }
        else if (time < phase3End) {
          LyronDrive.arcadeDrive(0.6, 0); 
          intakeAndLauncherRoller.stopMotor();
          feederRoller.stopMotor();
        }
        else if (time < phase4End) {
          LyronDrive.stopMotor();
          intakeAndLauncherRoller.setVoltage(LAUNCHING_LAUNCHER_VOLTAGE); 
          feederRoller.setVoltage(SPIN_UP_FEEDER_VOLTAGE);                
        }
        else if (time < phase5End) {
          LyronDrive.stopMotor();
          intakeAndLauncherRoller.setVoltage(LAUNCHING_LAUNCHER_VOLTAGE); 
          feederRoller.setVoltage(LAUNCHING_FEEDER_VOLTAGE);              
        }
        else {
          LyronDrive.stopMotor();
          intakeAndLauncherRoller.stopMotor();
          feederRoller.stopMotor();
        }
        break;
    }
  }

  @Override
  public void teleopInit() {
    spinUpTimer.reset();
    spinUpTimer.start();
  }

  @Override
  public void teleopPeriodic() {
    double driveSpeed = driverController.getLeftY();
    double rotateSpeed = driverController.getRightX();
    LyronDrive.arcadeDrive(driveSpeed, rotateSpeed);

    if(operatorController.getBButton()){
        if(operatorController.getBButtonPressed()){
            spinUpTimer.reset();
        }
        
        if(spinUpTimer.get() < SPIN_UP_SECONDS){
            intakeAndLauncherRoller.setVoltage(LAUNCHING_LAUNCHER_VOLTAGE);
            feederRoller.setVoltage(SPIN_UP_FEEDER_VOLTAGE);
        } else {
            intakeAndLauncherRoller.setVoltage(LAUNCHING_LAUNCHER_VOLTAGE);
            feederRoller.setVoltage(LAUNCHING_FEEDER_VOLTAGE);
        }
    }
    else if(operatorController.getXButton()){ 
        intakeAndLauncherRoller.setVoltage(-INTAKING_INTAKE_VOLTAGE);
        feederRoller.setVoltage(INTAKING_FEEDER_VOLTAGE);
    }
    else if(operatorController.getLeftBumper()){ 
       intakeAndLauncherRoller.setVoltage(-INTAKING_INTAKE_VOLTAGE);
       feederRoller.setVoltage(-INTAKING_FEEDER_VOLTAGE);        
    }
    else { 
        intakeAndLauncherRoller.stopMotor();
        feederRoller.stopMotor();
    }
  }

  @Override
  public void disabledInit() {
    LyronDrive.stopMotor();
  }
}