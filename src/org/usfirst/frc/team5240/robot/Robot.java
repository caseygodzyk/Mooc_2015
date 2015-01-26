package org.usfirst.frc.team5240.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Ultrasonic;

import com.usfirst.team5240.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 



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
//change commit test
public class Robot extends IterativeRobot {
	
	 RobotDrive robotDrive;
	    Joystick stick;
	    CANTalon frontRight;
	    // Channels for the wheels
		CANTalon frontLeft;
		CANTalon rearLeft;
		CANTalon rearRight;
		Ultrasonic ultrasonic;
		
	    final int joystickChannel	= 0;
	    
	    //<Gyro code start
	    SerialPort serial_port;
	    //IMU imu;  // Alternatively, use IMUAdvanced for advanced features
	    IMUAdvanced imu;
	    boolean first_iteration;
	    //Gyro code stop

	    

    public void robotInit() {
    	 frontRight = new CANTalon(1);
    	 frontLeft = new CANTalon(4);
    	 rearRight = new CANTalon(2);
    	 rearLeft = new CANTalon(3);
         robotDrive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
         
         ultrasonic = new Ultrasonic(1,2);
        
        stick = new Joystick(joystickChannel);
        
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
    public double distance(double accelerationInitial, double accelerationFinal, double timeInitial, double timeFinal,double velocityInitial ) {
    	double avgAcc=(accelerationInitial+accelerationFinal)/2;
    	double changeInTime=timeFinal-timeInitial;
    	return velocityInitial+.5*avgAcc*changeInTime*changeInTime;
    }
    public double velocity(double initialVeloc,double accelerationInitial,double accelerationFinal,double timeInitial,double timeFinal){
    	double avgAcc=(accelerationInitial+accelerationFinal)/2;
    	double changeInTime=timeFinal-timeInitial;
    	return initialVeloc+avgAcc*changeInTime;
    }
        
    double initialTime =-1;
    double previousTime=-1;
    double previousAccelerationX=0;
    double previousAccelerationY=0;
    double autoDistanceX=0;
    double autoDistanceY=0;
    double initialVelocX=0;
	Timer drivetime = new Timer();
	
    public void autonomousInit(){
    	drivetime.start();
    	SmartDashboard.putBoolean("autoinit", true);
    	initialTime=-1;
    	initialVelocX=0;
    	previousTime=-1;
    	previousAccelerationX=0;
    	previousAccelerationY=0;
    	autoDistanceX=0;
    	autoDistanceY=0;
    }
    public void autonomousPeriodic() {
    	if(previousTime==-1) previousTime=drivetime.get();
    	if(initialTime==-1)initialTime=drivetime.get();
    	autoDistanceX+=distance(previousAccelerationX,imu.getWorldLinearAccelX(),previousTime,drivetime.get(),initialVelocX);
    	SmartDashboard.putNumber("distFunction", distance(previousAccelerationX,imu.getWorldLinearAccelX(),previousTime,drivetime.get(),initialVelocX));
    	SmartDashboard.putNumber("AccelerationAvg", (previousAccelerationX+imu.getWorldLinearAccelX())/2);
    	autoDistanceY+=distance(previousAccelerationY,imu.getWorldLinearAccelY(),previousTime,drivetime.get(),initialVelocX);
    	SmartDashboard.putNumber("LinearAccX", imu.getWorldLinearAccelX());
    	SmartDashboard.putNumber("LinearAccY", imu.getWorldLinearAccelY());
    	SmartDashboard.putNumber("Distance X", autoDistanceX);
    	SmartDashboard.putNumber("Distance Y", autoDistanceY);
    	SmartDashboard.putNumber("Time", drivetime.get());
    	SmartDashboard.putNumber("TotalTime",drivetime.get()-initialTime);
    	SmartDashboard.putNumber("CalculatedTime",drivetime.get()-previousTime);
    
    	//put in auto code
    	if(autoDistanceX>-200){//assume s
    		robotDrive.drive(-.2, 0);
    		SmartDashboard.putBoolean("driving", true);
    	}
    	else{
    		SmartDashboard.putBoolean("driving", false);
    		robotDrive.drive(0, 0);
    	}
    	//end auto code
    	initialVelocX=velocity(initialVelocX,previousAccelerationX,imu.getWorldLinearAccelX(),initialTime,drivetime.get());
    	previousAccelerationX=imu.getWorldLinearAccelX();
    	previousAccelerationY=imu.getWorldLinearAccelY();
    	previousTime=drivetime.get();
    }

    /**
     * Runs the motors with arcade steering.
     */
    public void teleopPeriodic() {
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
            SmartDashboard.putNumber( "Ultrasonic_Inches", ultrasonic.getRangeInches());
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
            
        
        
     
    
    /**
     * This function is called once each time the robot enters test mode.
     */
        	//frontRight.set(stick.getY());
        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        	// This sample does not use field-oriented drive, so the gyro input is set to zero.
            robotDrive.mecanumDrive_Cartesian(stick.getRawAxis(3)-stick.getRawAxis(2),stick.getRawAxis(4),stick.getRawAxis(5), imu.getPitch());
          
            Timer.delay(0.01);	// wait 5ms to avoid hoggingw cycles
        }
    }
           
    

    /**
     * Runs during test mode
     */
    public void testPeriodic() {
        robotDrive.setSafetyEnabled(false);
        robotDrive.mecanumDrive_Cartesian(stick.getRawAxis(3)-stick.getRawAxis(2),stick.getRawAxis(4), stick.getRawAxis(5), imu.getPitch());

    	
    	 SmartDashboard.putBoolean(  "IMU_Connected",        imu.isConnected());
         SmartDashboard.putBoolean(  "IMU_IsCalibrating",    imu.isCalibrating());
         SmartDashboard.putNumber(   "IMU_Yaw",              imu.getYaw());
         SmartDashboard.putNumber(   "IMU_Pitch",            imu.getPitch());
         SmartDashboard.putNumber(   "IMU_Roll",             imu.getRoll());
         SmartDashboard.putNumber(   "IMU_CompassHeading",   imu.getCompassHeading());
         SmartDashboard.putNumber(   "IMU_Update_Count",     imu.getUpdateCount());
         SmartDashboard.putNumber(   "IMU_Byte_Count",       imu.getByteCount());
         SmartDashboard.putNumber("number", 10);
         // If you are using the IMUAdvanced class, you can also access the following
         // additional functions, at the expense of some extra processing
         // that occurs on the CRio processor
         
         SmartDashboard.putNumber(   "IMU_Accel_X",          imu.getWorldLinearAccelX());
         SmartDashboard.putNumber(   "IMU_Accel_Y",          imu.getWorldLinearAccelY());
         SmartDashboard.putBoolean(  "IMU_IsMoving",         imu.isMoving());
         SmartDashboard.putNumber(   "IMU_Temp_C",           imu.getTempC());
    }


}
