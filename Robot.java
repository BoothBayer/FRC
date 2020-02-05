/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import java.util.concurrent.TimeUnit;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import java.util.concurrent.TimeUnit;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SPI;

//import java.math.RoundingMode;
import java.text.DecimalFormat;

public class Robot extends TimedRobot {
  private Joystick joystick;
  private PWMVictorSPX frontLeftMotor;
  private PWMVictorSPX rearLeftMotor;
  private PWMVictorSPX frontRightMotor;
  private PWMVictorSPX rearRightMotor;
  private JoystickButton Bbutton;
  private PIDController turnController;
  private AHRS m_gyro;

  private static final double kP = 0.02;
  private static final double kI = 0.00;
  private static final double kD = 0.01;
  

  MecanumDrive my_drive;
  @Override
  public void robotInit() {
    
    joystick = new Joystick(0);
    Bbutton = new JoystickButton(joystick, 2);
    frontLeftMotor = new PWMVictorSPX(0);
    rearLeftMotor = new PWMVictorSPX(1);
    frontRightMotor = new PWMVictorSPX(2);
    rearRightMotor = new PWMVictorSPX(3);

    my_drive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    m_gyro = new AHRS(SPI.Port.kMXP);
  }

  private Double getGyro() {
    DecimalFormat triDec = new DecimalFormat("###.###");
    double gyro = m_gyro.getAngle();
    gyro = Double.parseDouble(triDec.format(gyro));
    return gyro;
  }
  
  private void turn90()
  {
    System.out.println("TURNING 90 DEGREES");
    
    boolean aimDone = false;
    double aimG = (getGyro() + 90);
    turnController = new PIDController(kP, kI, kD);
    
    System.out.println(getGyro());
    System.out.println(aimG);
    //turnController.disableContinuousInput();

    while (!aimDone){
      mecMove(0, 0, turnController.calculate(getGyro(), aimG));

      if ( (getGyro() >= (aimG - 3)) && ((getGyro()) <= (aimG + 3)) ){
        System.out.println("TURNED ~90 DEGREES");
        //turnController.disableContinuousInput();
        aimDone = true;
        break; 
      }
    }
  }

  @Override
  public void teleopPeriodic() {
    if(Math.abs(joystick.getRawAxis(0)) < 0.02) {
      my_drive.driveCartesian(joystick.getRawAxis(1), (joystick.getRawAxis(4) * -1), turnController.calculate(getGyro(), getGyro()));
    }
    my_drive.driveCartesian(joystick.getRawAxis(1), (joystick.getRawAxis(4)* -1), joystick.getRawAxis(0));
    if(joystick.getRawButton(2)){ // B button
      turn90();
    }
    if(joystick.getRawButton(1)){ // A button
      System.out.println(getGyro());
    }
    if(joystick.getRawButton(3)){ // X button
      System.out.println("X");
    }

  }

  public void mecMove(double value1, double value2, double value3)
  {
    my_drive.driveCartesian(value1, value2, value3);
  }
}