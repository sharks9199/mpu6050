// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot; 

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import frc.robot.MPU6050;



//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


import com.ctre.phoenix.motorcontrol.ControlMode;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
 // WPI_VictorSPX m_leftDrive = new WPI_VictorSPX(1);
  //WPI_VictorSPX m_rightDrive = new WPI_VictorSPX(0);
//  DifferentialDrive m_testRobot = new DifferentialDrive(m_leftDrive,m_rightDrive);
  BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
  MPU6050 m_Gyro = new MPU6050();
  
//private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
 // private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();
 // Thread m_visionThread;
 float anterior;

 // CvSink cvsink = CameraServer.getVideo();
 // CvSource outputStream = CameraServer.putVideo("rectangle", 640, 480);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // m_visionThread = new Thread(()->
    // {UsbCamera camera = CameraServer.startAutomaticCapture();
    //   camera.setResolution(640, 480);});
    // m_visionThread.setDaemon(true);
    // m_visionThread.start();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
   // m_leftDrive.set(ControlMode.PercentOutput,0);
   // m_rightDrive.setInverted(true);
    
   // m_rightDrive.set(ControlMode.PercentOutput,0);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 1000.0) {
      //m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    //  m_leftDrive.set(ControlMode.PercentOutput,0.1);
    //  m_rightDrive.set(ControlMode.PercentOutput,0.1);
    } else {
       // stop robot
    }

    
    //m_rightDrive.set(ControlMode.PercentOutput,100);
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    
    SmartDashboard.putNumber("Accel X", accelerometer.getX());
    SmartDashboard.putNumber("Accel Y", accelerometer.getY());
    SmartDashboard.putNumber("Accel Z", accelerometer.getZ());
    SmartDashboard.putNumber("Ângulo em X", Math.toDegrees(Math.atan(-accelerometer.getY()/-accelerometer.getZ())*Math.PI/2));
    SmartDashboard.putNumber("Ângulo em Y", Math.toDegrees(Math.atan(-accelerometer.getX()/-accelerometer.getZ())*Math.PI/2));
    SmartDashboard.putNumber("Ângulo em Z", Math.toDegrees(Math.atan(-accelerometer.getY()/-accelerometer.getX())*Math.PI/2));
    //System.out.println(m_Gyro.readXAxis()/131 - anterior);
    SmartDashboard.putNumber("Acell X MPU6050",m_Gyro.readXAxisAccel());
    SmartDashboard.putNumber("Acell Y MPU6050",m_Gyro.readYAxisAccel());
    SmartDashboard.putNumber("Acell Z MPU6050",m_Gyro.readZAxisAccel());
    SmartDashboard.putNumber("Angles X MPU6050",m_Gyro.readXAxisAngle());
    SmartDashboard.putNumber("Angles Y MPU6050",m_Gyro.readYAxisAngle());
    SmartDashboard.putNumber("Angles Z MPU6050",m_Gyro.readZAxisAngle());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
