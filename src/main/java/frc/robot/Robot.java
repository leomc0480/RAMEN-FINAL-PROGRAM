// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final XboxController control1 = new XboxController(0);
  private final XboxController control2 = new XboxController(1);

  private TalonSRX Motor1FrontRight = new TalonSRX(5);
  private TalonSRX Motor2FrontLeft = new TalonSRX(7);
  private TalonSRX Motor3BackRight = new TalonSRX(3);
  private TalonSRX Motor4BackLeft = new TalonSRX(14);
  private TalonSRX Motor5Intake = new TalonSRX(8);
  private TalonSRX Motor6Hooks = new TalonSRX(6);

  double Joystick1;
  double Joystick2;
  boolean YButton;
  double Rightdemand;
  double Leftdemand;
  boolean BButton;
  boolean XButton;
  boolean AButton;
  double absoluteTimer;
  double relativeTimer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //Camara
  CameraServer.startAutomaticCapture();
  //iniciar los motores para q no inicien solos :D
    Motor1FrontRight.set(ControlMode.PercentOutput, 0);
    Motor2FrontLeft.set(ControlMode.PercentOutput, 0);
    Motor3BackRight.set(ControlMode.PercentOutput, 0);
    Motor4BackLeft.set(ControlMode.PercentOutput, 0);
    Motor5Intake.set(ControlMode.PercentOutput, 0);


    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
  m_autonomousCommand = m_robotContainer.getAutonomousCommand();

  class GetTimeAction{

  
    public void autoAbsoluteTimeControl(){
      absoluteTimer = Timer.getFPGATimestamp();
    }
    public void autoRelativeTimeControl(){
      relativeTimer = Timer.getFPGATimestamp();
    }
    public double getAbsoluteTimer(){
      return absoluteTimer;
    }
    public double getRelativeTimer(){
      return relativeTimer;
    }
  }
    

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    if (relativeTimer > 2 && relativeTimer < 4){
    Motor1FrontRight.set(ControlMode.PercentOutput,(1*0.795));
    Motor2FrontLeft.set(ControlMode.PercentOutput,(-1));
    Motor3BackRight.set(ControlMode.PercentOutput,(1*0.795));
    Motor4BackLeft.set(ControlMode.PercentOutput,(-1));
    }
    else if (relativeTimer > 4 && relativeTimer < 7){
    Motor6Hooks.set(ControlMode.PercentOutput, (0.2));
      }
    else if (relativeTimer > 7 && relativeTimer < 7.7){
      Motor1FrontRight.set(ControlMode.PercentOutput,(-1*0.795));
      Motor2FrontLeft.set(ControlMode.PercentOutput,(1));
      Motor3BackRight.set(ControlMode.PercentOutput,(-1*0.795));
      Motor4BackLeft.set(ControlMode.PercentOutput,(1));
    }
    else if (relativeTimer > 7.7 && relativeTimer < 15){
      Motor5Intake.set(ControlMode.PercentOutput, (1));
    }
    else (relativeTimer > 15){
      Motor1FrontRight.set(ControlMode.PercentOutput,(0));
      Motor2FrontLeft.set(ControlMode.PercentOutput,(0));
      Motor3BackRight.set(ControlMode.PercentOutput,(0));
      Motor4BackLeft.set(ControlMode.PercentOutput,(0));
      Motor5Intake.set(ControlMode.PercentOutput, (0));
      Motor6Hooks.set(ControlMode.PercentOutput, (0));
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    Joystick1 = control1.getRawAxis(1);
    Joystick2 = control1.getRawAxis(4);
    YButton = control2.getRawButton(4);
    Rightdemand = control1.getRawAxis(3);
    Leftdemand = control1.getRawAxis(2);
    BButton = control2.getRawButton(2);
    XButton = control2.getRawButton(3);
    AButton = control2.getRawButton(1);

    Joystick2 = Math.abs(Joystick2) < 0.15 ? 0 : control1.getRawAxis(4);
    Joystick1 = Math.abs(Joystick1) < 0.15 ? 0 : control1.getRawAxis(1);
    Rightdemand = Math.abs(Rightdemand) < 0.15 ? 0 : control1.getRawAxis(3);
    Leftdemand= Math.abs(Leftdemand) < 0.15 ? 0: control1.getRawAxis(2);


    //Enfrente y atrás
    /*Motor1FrontRight.set(ControlMode.PercentOutput,(Joystick1* 0.795));
    Motor2FrontLeft.set(ControlMode.PercentOutput,(-Joystick1));
    Motor3BackRight.set(ControlMode.PercentOutput,(Joystick1* 0.795));
    Motor4BackLeft.set(ControlMode.PercentOutput,(-Joystick1));*/

    // Intake
// usamos if y else pero como el motor es de velocidades se setea la velocidad y no true o false
    if(YButton) Motor5Intake.set(ControlMode.PercentOutput, (1));
    else if(BButton) Motor5Intake.set(ControlMode.PercentOutput, (-0.6));
    else Motor5Intake.set(ControlMode.PercentOutput, (0.0000000)); 

//Hooks
    if(XButton) Motor6Hooks.set(ControlMode.PercentOutput, (0.2));
    else if (AButton) Motor6Hooks.set(ControlMode.PercentOutput, (-0.2));
    else Motor6Hooks.set(ControlMode.PercentOutput, (0.00000000));

//En frente y atrás Rapido
    if(Rightdemand > 0){ 
    Motor1FrontRight.set(ControlMode.PercentOutput,(-Rightdemand*0.795));
    Motor2FrontLeft.set(ControlMode.PercentOutput,(Rightdemand));
    Motor3BackRight.set(ControlMode.PercentOutput,(-Rightdemand*0.795));
    Motor4BackLeft.set(ControlMode.PercentOutput,(Rightdemand));
    } 
    else if(Leftdemand > 0){
    Motor1FrontRight.set(ControlMode.PercentOutput,(Leftdemand*0.795));
    Motor2FrontLeft.set(ControlMode.PercentOutput,(-Leftdemand));
    Motor3BackRight.set(ControlMode.PercentOutput,(Leftdemand*0.795));
    Motor4BackLeft.set(ControlMode.PercentOutput,(-Leftdemand));
    }

//De lados y enfrente 
    else {
    double leftPWM = +Joystick2 + Joystick1;
    double rightPWM = +Joystick2 - Joystick1;

    Motor1FrontRight.set(ControlMode.PercentOutput, (leftPWM*0.795));
    Motor2FrontLeft.set(ControlMode.PercentOutput,(rightPWM));
    Motor3BackRight.set(ControlMode.PercentOutput,(leftPWM*0.795));
    Motor4BackLeft.set(ControlMode.PercentOutput,(rightPWM));
    }

    /*Motor1FrontRight.set(ControlMode.PercentOutput, (Joystick2-Joystick1));
    Motor2FrontLeft.set(ControlMode.PercentOutput,(-Joystick2+Joystick1));
    Motor3BackRight.set(ControlMode.PercentOutput,(Joystick2-Joystick1));
    Motor4BackLeft.set(ControlMode.PercentOutput,(-Joystick2+Joystick1));*/




    SmartDashboard.putNumber("Joystick1", Joystick1);
    SmartDashboard.putNumber("Joystick2", Joystick2);
    SmartDashboard.putNumber("Rightdemand", Rightdemand);
    SmartDashboard.putNumber("Leftdemand", Leftdemand);

  }



  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

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
