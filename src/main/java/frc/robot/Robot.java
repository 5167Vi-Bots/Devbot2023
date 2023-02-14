// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

// Set up drivetrain and drive motors
  private WPI_TalonFX left3 = new WPI_TalonFX(3);
  private WPI_TalonFX left4 = new WPI_TalonFX(4);
  private MotorControllerGroup LeftMotors = new MotorControllerGroup(left3, left4 );

  private WPI_TalonFX right1 = new WPI_TalonFX(1);
  private WPI_TalonFX right2 = new WPI_TalonFX(2);
  private MotorControllerGroup RightMotors = new MotorControllerGroup(right1,right2);

  private DifferentialDrive Drive = new DifferentialDrive(LeftMotors,RightMotors);

  

// Set up motors for game pieces.
  private WPI_TalonSRX armRotate = new WPI_TalonSRX(5);
  private WPI_TalonFX  armAngle = new WPI_TalonFX(6);
  private WPI_TalonSRX armExtend = new WPI_TalonSRX(7);

// Set up joysticks
  // private Joystick drivestick = new Joystick(0);
  private XboxController drivestick = new XboxController(0);

//Set up LED controler and variables.
  Spark LED;

  public static final double DarkRed = 0.59;
  public static final double BreathingRed = -0.17;
  public static final double HeartbeatRed = -0.25;
  public static final double StrobeRed = -0.11;
 
  public static final double DarkBlue = 0.85;
  public static final double BreathingBlue = -0.15;
  public static final double HeartbeatBlue = -0.23;
  public static final double StrobeBlue = -0.09;
  
  public static final double White = 0.93;
  public static final double BreathingWhite = -0.13;
  public static final double HeartbeatWhite = -0.21;
  public static final double StrobeWhite = -0.05;
   
  private static final String LeaveCommunityAuto = "LeaveCommunity Auto";
  private static final String TwoConeAuto = "TwoCone Auto";
  private static final String ConeChargeAuto = "ConeCharge Auto";
  private String m_autoSelected;
   
// Set up prematch option chooser
  SendableChooser <String> autonchooser = new SendableChooser<>();
  SendableChooser <Double> allianceChooser = new SendableChooser<>();
  
// Set up pigoen imu.
  PigeonIMU pigeon1 = new PigeonIMU(0);

  double yaw;
  double pitch;

  
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    
// Set drivetrain values.  
    right1.setInverted(true);
    right2.setInverted(true);

    right1.setNeutralMode(NeutralMode.Brake);
    right2.setNeutralMode(NeutralMode.Brake);
    left3.setNeutralMode(NeutralMode.Brake);
    left4.setNeutralMode(NeutralMode.Brake);

//armRotate TalonSRX
  armRotate.config_kP(0, 0.0);
  armRotate.config_kI(0, 0.0);
  armRotate.config_kD(0, 0.0);
  armRotate.config_kF(0, 0.0);
  armRotate.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	 0, 30);
  armRotate.setInverted(true);
  armRotate.setSensorPhase(true);
  armRotate.setSelectedSensorPosition(0, 0, 10);

//armAngle TalonSRX
  armAngle.config_kP(0, 0.0);
  armAngle.config_kI(0, 0.0);
  armAngle.config_kD(0, 0.0);
  armAngle.config_kF(0, 0.0);
  armAngle.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,	 0, 30);
  armAngle.setInverted(true);
  armAngle.setSensorPhase(true);
  armAngle.setSelectedSensorPosition(0, 0, 10); 
  armAngle.set(ControlMode.Position, 0);

 //armExtend TalonSRX
  armExtend.config_kP(0, 0.0);
  armExtend.config_kI(0, 0.0);
  armExtend.config_kD(0, 0.0);
  armExtend.config_kF(0, 0.0);
  armExtend.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	 0, 30);
  armExtend.setInverted(true);
  armExtend.setSensorPhase(true);
  armExtend.setSelectedSensorPosition(0, 0, 10);
    
// Set LED channel.
    LED = new Spark(9);

// Place items on prematch tadb.    
    ShuffleboardTab preMatch = Shuffleboard.getTab("PreMatch");
    
    allianceChooser.addOption("Blue Alliance", DarkBlue);
    allianceChooser.addOption("Red Alliance", DarkRed);
    allianceChooser.setDefaultOption("WeForgot Alliance", White);
    preMatch.add("Alliance", allianceChooser);
    
    autonchooser.addOption("TwoCone Auto", TwoConeAuto);
    autonchooser.addOption("ConeCharge Auto", ConeChargeAuto);
    autonchooser.setDefaultOption("LeaveCommunity Auto", LeaveCommunityAuto);
    preMatch.add("Auton", autonchooser);
    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

// Put data on dashboard.
    SmartDashboard.putNumber("Yaw1", pigeon1.getYaw());
    SmartDashboard.putNumber("Pitch1", pigeon1.getPitch());
    
    SmartDashboard.putNumber("Timer", Timer.getMatchTime());

    SmartDashboard.putNumber("LeftStickY", drivestick.getLeftY());
    SmartDashboard.putNumber("RightStickY", drivestick.getRightX());

    SmartDashboard.putNumber("armRotate Encoder", armRotate.getSelectedSensorPosition());
    SmartDashboard.putNumber("armAngle Encoder", armAngle.getSelectedSensorPosition());
    SmartDashboard.putNumber("armExtend Encoder", armExtend.getSelectedSensorPosition());


    if (isDisabled()) {
      LED.set(allianceChooser.getSelected());
    }

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    
    LED.set(allianceChooser.getSelected());

    m_autoSelected = autonchooser.getSelected();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    switch (m_autoSelected) {
      case TwoConeAuto:
         //Put custom auto code here
         Timer.delay(4);
         LED.set(BreathingWhite);
        break;
      case ConeChargeAuto:
         //Put custom auto code here
         Timer.delay(4);
         LED.set(HeartbeatWhite);
         break;
      case LeaveCommunityAuto:
         // Put default auto code here
          Timer.delay(4);
          LED.set(StrobeWhite);
        break;
    }

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    LED.set(allianceChooser.getSelected());

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    

// flight stick
// Xaxis = 1
// Yaxis = 2
// Zaxis = 3

    double drive1 = -drivestick.getLeftY();
    double steer1 = -drivestick.getLeftX();

              
    if (drivestick.getYButton()){
      pigeon1.setYaw(0);
    }

    if (drivestick.getAButton()){ 
      armAngle.set(ControlMode.Position, 5000);
    }
    else if (drivestick.getBButton()){
      armAngle.set(ControlMode.Position, 10000);
    }
    else if (drivestick.getYButton()){
      armAngle.set(ControlMode.Position, 0);
    }


// LED strip control.
if (Timer.getMatchTime() <=135 && Timer.getMatchTime() >= 60) 
{
  if (allianceChooser.getSelected() == DarkRed) {
    LED.set(BreathingRed);
  } else if (allianceChooser.getSelected() == DarkBlue){
  LED.set(BreathingBlue);
  } else {
    LED.set(BreathingWhite);
  }
}
else if(Timer.getMatchTime() <=59 && Timer.getMatchTime() >= 30){
  if (allianceChooser.getSelected() == DarkRed) {
    LED.set(HeartbeatRed);
  } else if (allianceChooser.getSelected() == DarkBlue){
    LED.set(HeartbeatBlue);
  } else {
    LED.set(HeartbeatWhite);
  }
}
  else if (Timer.getMatchTime() <=29 && Timer.getMatchTime() >=0){
    if (allianceChooser.getSelected() == DarkRed) {
      LED.set(StrobeRed);
    } else if (allianceChooser.getSelected() == DarkBlue){
      LED.set(StrobeBlue);
    } else {
      LED.set(StrobeWhite);
   }
  }


  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {

    LED.set(allianceChooser.getSelected());

  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

 




}
