/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import java.util.concurrent.TimeUnit;
import edu.wpi.first.wpilibj.TimedRobot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.SPI;
import java.text.DecimalFormat;

public class Robot extends TimedRobot {
  private Joystick joystick;
  
  private PWMVictorSPX frontLeftMotor;
  private PWMVictorSPX rearLeftMotor;
  private PWMVictorSPX frontRightMotor;
  private PWMVictorSPX rearRightMotor;
  private PWMVictorSPX liftMotorHercules;
  private PWMVictorSPX liftMotorWeak;
  private PWMVictorSPX shootMotor;
  private PWMVictorSPX loadMotor;
  private PIDController moveController;
  private AHRS m_gyro;
  private double heading; 

  private static final double MkP = 0.05;
  private static final double MkI = 0.00;
  private static final double MkD = 0.005;
  
  //Timer gyroUpdate;
  
  MecanumDrive my_drive;
  
  @Override
  public void robotInit() {
    joystick = new Joystick(0);
    
    frontLeftMotor = new PWMVictorSPX(7);
    rearLeftMotor = new PWMVictorSPX(9);
    frontRightMotor = new PWMVictorSPX(8);
    rearRightMotor = new PWMVictorSPX(6);
    liftMotorHercules = new PWMVictorSPX(2);
    liftMotorWeak = new PWMVictorSPX(4);
    shootMotor = new PWMVictorSPX(0);
    loadMotor = new PWMVictorSPX(1);
    my_drive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    
    m_gyro = new AHRS(SPI.Port.kMXP);
    m_gyro.reset();
    heading = getGyro();

    moveController = new PIDController(MkP, MkI, MkD);
    moveController.setIntegratorRange(-1, 1);
  }

  private double getGyro() {
    DecimalFormat triDec = new DecimalFormat("###.#####");
    double gyro = m_gyro.getAngle();
    gyro = Double.parseDouble(triDec.format(gyro));
    return gyro;
  }
  
  private void turn90()
  {
    System.out.println("TURNING 90 DEGREES");
    
    boolean aimDone = false;
    double aimG = (getGyro() + 90);
    
    System.out.println(getGyro());
    System.out.println(aimG);

    while (!aimDone){
      my_drive.driveCartesian(0, 0, moveController.calculate(getGyro(), aimG));
      System.out.println(moveController.calculate(getGyro(), aimG));

      if ( (getGyro() >= (aimG - 3)) && ((getGyro()) <= (aimG + 3)) ){
        System.out.println("TURNED ~90 DEGREES");
        heading = getGyro();
        aimDone = true;
        break; 
      }
    }
  }

  @Override
  public void teleopPeriodic() {

    if(Math.abs(joystick.getRawAxis(0)) >= 0.05){
      my_drive.driveCartesian(0, 0, joystick.getRawAxis(0));
      heading = getGyro();  
    } else if(Math.abs(joystick.getRawAxis(1)) >= 0.05) {
      my_drive.driveCartesian(0, (joystick.getRawAxis(1) * -1), moveController.calculate(getGyro(), heading) );
      //my_drive.driveCartesian((joystick.getRawAxis(4)), (joystick.getRawAxis(1) * -1), 0 );
    } else if(Math.abs(joystick.getRawAxis(4)) >= 0.05) {
      my_drive.driveCartesian(joystick.getRawAxis(4), 0, 0);
    } else {
      my_drive.driveCartesian(0, 0, 0);
      heading = getGyro();
    }

    
    if(joystick.getRawButton(1)){ // A button
      //System.out.println(getGyro());
      System.out.println("A");
    }
    
    
    if(joystick.getRawButton(5)){ //Left Shoulder Button || DOWN LIFT
      //Lift Down
      liftMotorWeak.set(1);
      liftMotorHercules.set(-0.2);
    } else if (joystick.getRawButton(6)){ // Right Shoulder Button || UP LIFT
      //Lift Up
      liftMotorWeak.set(-1); 
      liftMotorHercules.set(0.5);
    } else if(joystick.getRawButton(3)){ // X button
      liftMotorWeak.set(1); //Spool Down Motor
      System.out.println("X");
    }
    /*else if(joystick.getRawButton(2)){ // B button
      //Unspool Down Motor
      liftMotorWeak.set(-1);
    } else if (joystick.getRawButton(4)){ //A button
      //Unspool Up Motor
      liftMotorHercules.set(0.25);
    } else if (joystick.getRawButton(1)){ //Y button
      //Spool Up Motor
      liftMotorHercules.set(-0.25);
    }*/ else {
      //Halt Lift
      liftMotorHercules.set(0);
      liftMotorWeak.set(0);
    }

    if (joystick.getRawButton(2)){
      shootMotor.set(1);
      System.out.println("SHOOT");
    } else{
      shootMotor.set(0);
    }
    if (joystick.getRawButton(4)){
      loadMotor.set(1);
      System.out.println("LOAD");
    } else {
      loadMotor.set(0);
    }

  }
}

