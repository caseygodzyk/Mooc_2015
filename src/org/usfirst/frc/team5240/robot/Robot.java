package org.usfirst.frc.team5240.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CANTalon;

import com.usfirst.team5240.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import edu.wpi.first.wpilibj.buttons.Button;

 



/**
 * @author JoshMiller
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
public class Robot extends SampleRobot {
	
	 RobotDrive robotDrive;
	    Joystick DriverStick1;
	    Joystick DriverStick2;
	    Joystick OperatorPad;
	    Joystick VaderUp;
	    Joystick VaderDown;
	    Button VaderForkOut;
	    Button VaderForkIn ;
	    
	    
	    CANTalon frontRight;
	    CANTalon frontLeft;
		CANTalon rearLeft;
		CANTalon rearRight;
		CANTalon elevator1;
		CANTalon elevator2;
		CANTalon VaderFork1;
		CANTalon VaderFork2;
		
	    final int DriverStick1Channel	= 0;
	    final int DriverStick2Channel   = 1;
	    final int OperatorPadChannel = 2;
	   
//	    Speed in which we strafe
	    final int strafeSpeed = 1;
//	    drivetrainSpeed
	    final int speed = 1;
//	    Vader controlling axis
	    final int vaderAxis = 1;
	    
//	    eleVADER speed
	    final int vaderUpSpeed = 1;
	    final int vaderDownSpeed = 1;
	    
//	    The Vader Fork controlling axis
	    final int vaderForkOutAxis = 2;
	    final int vaderForkInAxis = 3;
//	    Speed in which we hella vader fork
	    final int vaderForkInSpeed = 1;
	    final int vaderForkOutSpeed = 1;
	    
	    
	    SerialPort serial_port;
	    IMUAdvanced imu;
	    boolean first_iteration;
	    
	   
	    public Robot() {
	    	frontRight = new CANTalon(1);
	    	frontLeft = new CANTalon(4);
    	 	rearRight = new CANTalon(2);
    	 	rearLeft = new CANTalon(3);
    	 	elevator1 = new CANTalon(5);
    	 	elevator2 = new CANTalon(6);
    	 	VaderFork1 = new CANTalon(7);
    	 	VaderFork2 = new CANTalon(8);
    	 	robotDrive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
    	 	
         
         
        
    	 	DriverStick2 = new Joystick(DriverStick1Channel);
    	 	DriverStick1 = new Joystick(DriverStick2Channel);
    	 	OperatorPad  = new Joystick(OperatorPadChannel );
    	 	
        try {
        serial_port = new SerialPort(57600,SerialPort.Port.kMXP);
                
                // You can add a second parameter to modify the 
                // update rate (in hz) from 4 to 100.  The default is 100.
                // If you need to minimize CPU load, you can set it to a
                // lower value, as shown here, depending upon your needs.
                
                // You can also use the IMUAdvanced class for advanced
                // features.
                
                byte update_rate_hz = 50;
                //imu = new IMU(serial_port,update_rate_hz);
                imu = new IMUAdvanced(serial_port,update_rate_hz);
        } catch( Exception ex ) {
                
        }
        if ( imu != null ) {
            LiveWindow.addSensor("IMU", "Gyro", imu);
        }
        first_iteration = true;
    }

	    	
	    	
	    
        
    

	    public void autonomous() {
       
    }


	    public void operatorControl() {
	    	
	    	
	    	
        robotDrive.setSafetyEnabled(false);
        
        while (isOperatorControl() && isEnabled()) {
        	
        	
        	
        	//gyro code>>>>
        	// When calibration has completed, zero the yaw
            // Calibration is complete approaximately 20 seconds
            // after the robot is powered on.  During calibration,
            // the robot should be still
            
            boolean is_calibrating = imu.isCalibrating();
            if ( first_iteration && !is_calibrating ) {
                Timer.delay( 0.3 );
                imu.zeroYaw();
                first_iteration = false;
            }
            
            // Update the dashboard with status and orientation
            // data from the nav6 IMU
            
            SmartDashboard.putBoolean(  "IMU_Connected",        imu.isConnected());
            SmartDashboard.putBoolean(  "IMU_IsCalibrating",    imu.isCalibrating());
            SmartDashboard.putNumber(   "IMU_Yaw",              imu.getYaw());
            SmartDashboard.putNumber(   "IMU_Pitch",            imu.getPitch());
            SmartDashboard.putNumber(   "IMU_Roll",             imu.getRoll());
            SmartDashboard.putNumber(   "IMU_CompassHeading",   imu.getCompassHeading());
            SmartDashboard.putNumber(   "IMU_Update_Count",     imu.getUpdateCount());
            SmartDashboard.putNumber(   "IMU_Byte_Count",       imu.getByteCount());

            // If you are using the IMUAdvanced class, you can also access the following
            // additional functions, at the expense of some extra processing
            // that occurs on the CRio processor
            
            SmartDashboard.putNumber(   "IMU_Accel_X",          imu.getWorldLinearAccelX());
            SmartDashboard.putNumber(   "IMU_Accel_Y",          imu.getWorldLinearAccelY());
            SmartDashboard.putBoolean(  "IMU_IsMoving",         imu.isMoving());
            SmartDashboard.putNumber(   "IMU_Temp_C",           imu.getTempC());
            
            Timer.delay(0.2);
            
            
        }

            robotDrive.mecanumDrive_Cartesian(DriverStick1.getRawAxis(3)*strafeSpeed-DriverStick2.getRawAxis(2)*
            		strafeSpeed,DriverStick1.getRawAxis(4)*speed, DriverStick2.getRawAxis(5)*speed, imu.getPitch());
            
//            vader going up control
            elevator1.set(OperatorPad.getRawAxis(vaderAxis)*vaderUpSpeed);
            elevator2.set(OperatorPad.getRawAxis(vaderAxis)*vaderUpSpeed);
//            vader going down contorl
            elevator2.set(OperatorPad.getRawAxis(vaderAxis)*vaderDownSpeed);
            elevator1.set(OperatorPad.getRawAxis(vaderAxis)*vaderDownSpeed);
            
            
            
            
//            vaderForks controlled going in 
            VaderFork1.set(OperatorPad.getRawAxis(vaderForkInAxis)*vaderForkInSpeed);
            VaderFork2.set(OperatorPad.getRawAxis(vaderForkInAxis)*vaderForkInSpeed);
//            vaderForks controlled going out
            VaderFork1.set(OperatorPad.getRawAxis(vaderForkOutAxis)*vaderForkOutSpeed);
            VaderFork2.set(OperatorPad.getRawAxis(vaderForkOutAxis)*vaderForkOutSpeed);
            
            
            
            
            
            Timer.delay(0.01);	// wait 5ms to avoid hogging CPU cycles
}
           
    

    /**
     * Runs during test mode
     */
    public void test() {
    	
    }


}
