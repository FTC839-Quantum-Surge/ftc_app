/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.motors.NeveRest40Gearmotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * This is the Hardware Configuration class for the 2018-2019 Rover Ruckus design.
 *
 * All motors, sensors or other robot configuration specific properties will be
 * defined here and shared with all op modes
 *
 */
public class Robot
{

    public enum ClawStateEnum
    {
        Unknown,
        Closed,
        Open
    }

    // ----------------------------------------------------------------------
    // Private Constants
    // ----------------------------------------------------------------------
    private static final String GYRO                        = "imu";

    private static final String LEFT_FRONT_DRIVE_MOTOR      = "LF_drive";
    private static final String LEFT_REAR_DRIVE_MOTOR       = "LR_drive";
    private static final String RIGHT_FRONT_DRIVE_MOTOR     = "RF_drive";
    private static final String RIGHT_REAR_DRIVE_MOTOR      = "RR_drive";

    private static final String INTAKE_MOTOR                = "intake";
    private static final String FOLD_MOTOR                  = "fold";
    private static final String LIFT_MOTOR                  = "lift";

    private static final String DUMP_SERVO_1                = "dump1";
    private static final String DUMP_SERVO_2                = "dump2";

    private static final String CLAW_SERVO                  = "claw";

    private static final String LIMIT_TOP                   = "limitTop";
    private static final String LIMIT_BOTTOM                = "limitBottom";

    private static final String LIMIT_FOLD                  = "limitFold";

    static final double     HEADING_THRESHOLD       = 0.01;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.05;      // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.05;     // Larger is more responsive, but also less stable


// Encoder Count Per rev
// NeverRest (7 ppr without gearbox)
//    40:1  = 280ppr
//    60:1  = 420ppr
//   104:1  = 728ppr
//
// Hex Core Motor (4 ppr without gearbox) 288 ppr at axle

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: NeverRest 40:1 (7 pulses at motor) Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    Orientation             m_LastAngles  = new Orientation();
    double                  m_GlobalAngle = 0;

    // ----------------------------------------------------------------------
    // Public Member Variables
    // ----------------------------------------------------------------------

    private OpMode       m_opMode    = null;
    private HardwareMap  m_hwMap     = null;
    private ElapsedTime  m_period    = new ElapsedTime();
    private ElapsedTime  m_runtime   = new ElapsedTime();

    private BNO055IMU m_imu                = null;

    private DcMotor  m_leftFrontDrive      = null;
    private DcMotor  m_leftRearDrive       = null;
    private DcMotor  m_rightFrontDrive     = null;
    private DcMotor  m_rightRearDrive      = null;
    public  DcMotor  m_intake              = null;
    private DcMotor  m_foldMotor           = null;
    private DcMotor  m_liftMotor           = null;

    public  Servo    m_dump1               = null;
    public  Servo    m_dump2               = null;

    private Servo    m_claw                = null;

    // Using Analog inputs for Limit Switches due to
    // issue with hub not powering on when digital input held low

    private AnalogInput m_limitTop         = null;
    private AnalogInput m_limitBottom      = null;

    private RevTouchSensor m_limitFold     = null;

    private ClawStateEnum   m_clawState    = ClawStateEnum.Unknown;

    // //////////////////////////////////////////////////////////////////////
    // Public Member Variables
    // //////////////////////////////////////////////////////////////////////

    public Lift    Lift = null;
    public Fold    Fold = null;

    // //////////////////////////////////////////////////////////////////////
    // Constructor
    // //////////////////////////////////////////////////////////////////////

    public Robot( OpMode opMode )
    {
        m_opMode = opMode;
    }

    // //////////////////////////////////////////////////////////////////////
    // Initialize standard Hardware interfaces
    // //////////////////////////////////////////////////////////////////////

    public void init( HardwareMap ahwMap, boolean bUseIMU )
    {
        // ------------------------------------------------------------------
        // Save reference to Hardware map
        // ------------------------------------------------------------------

        m_hwMap = ahwMap;

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.

        if ( bUseIMU )
        {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".

            m_imu = m_hwMap.get(BNO055IMU.class, GYRO);

            m_imu.initialize(parameters);

            LinearOpMode opMode = (LinearOpMode)m_opMode;

            m_runtime.reset();

            m_imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        }

        // ------------------------------------------------------------------
        // Define and Initialize Motors
        // ------------------------------------------------------------------

        m_leftFrontDrive  = m_hwMap.get( DcMotor.class, LEFT_FRONT_DRIVE_MOTOR    );
        m_leftRearDrive   = m_hwMap.get( DcMotor.class, LEFT_REAR_DRIVE_MOTOR     );
        m_rightFrontDrive = m_hwMap.get( DcMotor.class, RIGHT_FRONT_DRIVE_MOTOR   );
        m_rightRearDrive  = m_hwMap.get( DcMotor.class, RIGHT_REAR_DRIVE_MOTOR    );

        m_intake          = m_hwMap.get( DcMotor.class, INTAKE_MOTOR              );
        m_foldMotor       = m_hwMap.get( DcMotor.class, FOLD_MOTOR                );
        m_liftMotor       = m_hwMap.get( DcMotor.class, LIFT_MOTOR                );

        m_limitTop        = m_hwMap.get( AnalogInput.class   , LIMIT_TOP          );
        m_limitBottom     = m_hwMap.get( AnalogInput.class   , LIMIT_BOTTOM       );
        m_limitFold       = m_hwMap.get( RevTouchSensor.class, LIMIT_FOLD         );

        // ------------------------------------------------------------------
        // Set direction of drive motors (one side needs to be opposite)
        // ------------------------------------------------------------------
//        m_leftFrontDrive  .setDirection( DcMotor.Direction.REVERSE );
//        m_leftRearDrive   .setDirection( DcMotor.Direction.REVERSE );

        m_rightFrontDrive .setDirection( DcMotor.Direction.REVERSE );
        m_rightRearDrive  .setDirection( DcMotor.Direction.REVERSE );

        m_liftMotor       .setDirection( DcMotor.Direction.REVERSE );

        // ------------------------------------------------------------------
        // Set all motors to zero power
        // ------------------------------------------------------------------

        m_leftFrontDrive .setPower( 0 );
        m_leftRearDrive  .setPower( 0 );
        m_rightFrontDrive.setPower( 0 );
        m_rightRearDrive .setPower( 0 );
        m_intake         .setPower( 0 );
        m_foldMotor      .setPower( 0 );
        m_liftMotor      .setPower( 0 );

        // ------------------------------------------------------------------
        // Set all motors run Mode
        // ------------------------------------------------------------------

        m_leftFrontDrive  .setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        m_rightFrontDrive .setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        m_foldMotor      .setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        m_liftMotor      .setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        m_leftFrontDrive .setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        m_leftRearDrive  .setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        m_rightFrontDrive.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        m_rightRearDrive .setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

        m_intake         .setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER   );

        m_foldMotor      .setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER   );
        m_liftMotor      .setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER   );

        m_foldMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );

        // ------------------------------------------------------------------
        // Define and initialize ALL installed servos.
        // ------------------------------------------------------------------

        m_dump1   = m_hwMap.get( Servo.class, DUMP_SERVO_1  );
        m_dump2   = m_hwMap.get( Servo.class, DUMP_SERVO_2  );
        m_claw    = m_hwMap.get( Servo.class, CLAW_SERVO    );

        m_dump1.setDirection( Servo.Direction.REVERSE );

        Lift = new Lift( m_liftMotor, m_limitTop, m_limitBottom );
        Fold = new Fold( m_foldMotor, m_limitFold );

        SetDumpPosition( 0 );
        CloseClaw();
    }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    public void OpModeStarted()
    {
        // Start the logging of measured acceleration
//        if (m_imu != null)
//            m_imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

    }

    public void OpModeStopped()
    {
        if ( m_imu != null )
        {
            m_imu.stopAccelerationIntegration();
        }

    }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    void AddTelemtry(Telemetry telemetry)
    {
        Orientation angles;

        // telemetry.addData("Sorter Pos",  "%f", m_dump.getPosition());
//        telemetry.addData( "paddle Pos", "%f", m_paddle.getPosition());

        //telemetry.addData("Lift Pos",  "%d", Lift.CurrentPos());

        if (m_imu != null) {
            telemetry.addLine(m_imu.getCalibrationStatus().toString());
            telemetry.addLine(m_imu.getSystemStatus().toShortString());

            angles = m_imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("Angular", "%.4f, %.4f, %.4f", angles.firstAngle, angles.secondAngle, angles.thirdAngle);

            Position p = m_imu.getPosition();
            telemetry.addData("Pos", "%.4f, %.4f, %.4f", p.z, p.y, p.x);
        }

        telemetry.addData( "Fold Limit", "%b", m_limitFold.isPressed());
        telemetry.addData("Lift Pos",  "%d", Lift.CurrentPos());
        telemetry.addData("Fold Pos",  "%d", Fold.CurrentPos());
        telemetry.addData("L. W Pos",  "%d", GetLeftDrivePos());
        telemetry.addData("R. W Pos",  "%d", GetRightDrivePos());

//        telemetry.addData( "RR Pwrs", "%f", m_rightFrontDrive.getPower());
//        telemetry.addData( "LR Pwrs", "%f", m_leftFrontDrive.getPower());

//        telemetry.addData( "RF Pwrs", "%f", m_rightFrontDrive.getPower());
//        telemetry.addData( "LF Pwrs", "%f", m_leftFrontDrive.getPower());

    }

    // //////////////////////////////////////////////////////////////////////
    // //////////////////////////////////////////////////////////////////////
    //
    //  Helper methods to manipulate motors/servos
    //
    // //////////////////////////////////////////////////////////////////////
    // //////////////////////////////////////////////////////////////////////

    public int GetLeftDrivePos () { return m_leftFrontDrive.getCurrentPosition(); }
    public int GetRightDrivePos() { return m_rightFrontDrive.getCurrentPosition(); }

    // //////////////////////////////////////////////////////////////////////
    // Open & Close Claw Functions
    // //////////////////////////////////////////////////////////////////////

    public void OpenClaw()
    {
        m_claw.setPosition( 0 );
        m_clawState = ClawStateEnum.Open;
    }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    public void CloseClaw()
    {
        m_claw.setPosition( 1 );
        m_clawState = ClawStateEnum.Closed;
    }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    public boolean IsClawOpen()
    {
        return (( m_clawState == ClawStateEnum.Open) && m_claw.getPosition() <= 2);
    }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    public void SetDumpPosition( double dPos )
    {
        m_dump1.setPosition( dPos );
        m_dump2.setPosition( dPos );
    }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    public void SetIntakePower( double dPower )
    {
        m_intake.setPower( dPower );
    }

    // //////////////////////////////////////////////////////////////////////
    // Set manual left and right drive motor power (Tank Drive)
    // //////////////////////////////////////////////////////////////////////

    public void SetDrivePower( double dLeftPower, double dRightPower )
    {
        m_leftFrontDrive .setPower( dLeftPower );
        m_leftRearDrive  .setPower( dLeftPower );

        m_rightFrontDrive.setPower( dRightPower );
        m_rightRearDrive .setPower( dRightPower );
    }

    // //////////////////////////////////////////////////////////////////////
    // TODO: Use encoders to drive a specific distance.
    // //////////////////////////////////////////////////////////////////////

    public void DriveDistance( double speed, double leftInches, double rightInches, double timeoutS)
    {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active

        LinearOpMode opMode = (LinearOpMode)m_opMode;

        if (opMode == null)
            return;

        if ( opMode.opModeIsActive())
        {

            // Determine new target position, and pass to motor controller

            newLeftTarget  = m_leftFrontDrive .getCurrentPosition() + (int)(leftInches  * COUNTS_PER_INCH);
            newRightTarget = m_rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            m_leftFrontDrive .setTargetPosition(newLeftTarget);
            m_rightFrontDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            m_leftFrontDrive .setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m_rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            m_runtime.reset();

            //SetDrivePower( Math.abs(speed), Math.abs(speed));

            m_leftFrontDrive.setPower ( Math.abs(speed) );
            m_rightFrontDrive.setPower( Math.abs(speed) );

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opMode.opModeIsActive() &&
                   (m_runtime.seconds() < timeoutS) &&
                   (m_leftFrontDrive.isBusy() || m_rightFrontDrive.isBusy())) {

                // Display it for the driver.

                m_opMode.telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                m_opMode.telemetry.addData("Path2",  "Running at %7d :%7d",
                        m_leftFrontDrive.getCurrentPosition(),
                        m_rightFrontDrive.getCurrentPosition());
                m_opMode.telemetry.update();

                Lift.PeriodicCheck( 0 );
            }

            // Stop all motion;
            SetDrivePower( 0, 0);

            // Turn off RUN_TO_POSITION
            m_leftFrontDrive .setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
            m_rightFrontDrive.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

            //  sleep(250);   // optional pause after each move
        }
    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle)
    {
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        LinearOpMode opMode = (LinearOpMode)m_opMode;

        if ((opMode == null) || (m_imu == null))
            return;

        if (opMode.opModeIsActive())
        {
            // get current angle
            Orientation angles = m_imu.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double dTargetAngle = angle + angles.firstAngle;

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget  = m_leftFrontDrive.getCurrentPosition() + moveCounts;
            newRightTarget = m_rightFrontDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            m_leftFrontDrive.setTargetPosition(newLeftTarget);
            m_rightFrontDrive.setTargetPosition(newRightTarget);

            m_leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m_rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            m_leftFrontDrive.setPower(speed);
            m_rightFrontDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() &&
                    (m_leftFrontDrive.isBusy() && m_rightFrontDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(dTargetAngle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                m_leftFrontDrive.setPower(leftSpeed);
                m_rightFrontDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                opMode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                opMode.telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                opMode.telemetry.addData("Actual",  "%7d:%7d",      m_leftFrontDrive.getCurrentPosition(),
                        m_rightFrontDrive.getCurrentPosition());
                opMode.telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                opMode.telemetry.update();
            }

            // Stop all motion;
            m_leftFrontDrive.setPower(0);
            m_rightFrontDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            m_leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m_rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {
        LinearOpMode opMode = (LinearOpMode)m_opMode;

        if ((opMode == null) || (m_imu == null))
            return;

        // get current angle
        Orientation angles = m_imu.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double dTargetAngle = angle + angles.firstAngle;

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && !onHeading(speed, dTargetAngle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            opMode.telemetry.update();
        }
    }

    // //////////////////////////////////////////////////////////////////////
    // TODO: Use IMU (gyro) or Encoders to turn specified number of degrees
    // //////////////////////////////////////////////////////////////////////

    /*
    public void PivitTurn( double dSpeed, double dAngle )
    {
        // ------------------------------------------------------------------
        // If requested to turn counter clockwise, negate dPower;
        // ------------------------------------------------------------------

        //dPower *= (dDegrees < 0) ? -1 : 1;
        // keep looping while we are still active, and not on heading.

        while ( ((LinearOpMode)m_opMode).opModeIsActive() && !onHeading(dSpeed, dAngle, P_TURN_COEFF))
        {
            // Update telemetry & Allow time for other processes to run.
            m_opMode.telemetry.update();
        }


        // 1) get current Gyro heading;
        // 2) SetDrivePower( dPower, -dPower );
        // 3) wait for Gyro to meet or exceed requested value
        // SetDrivePower( 0, 0 );   // Stop Motots

    }
*/
    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff)
    {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double   leftSpeed;
        double   rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.

        SetDrivePower( leftSpeed, rightSpeed );

        // Display it for the driver.
        m_opMode.telemetry.addData("Target", "%5.2f", angle);
        m_opMode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        m_opMode.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        m_opMode.telemetry.update();
        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        double angle = getAngle();

        robotError = targetAngle - angle;     //gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
     public static double scaleInput( double dVal )
     {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0)
        {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16)
        {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;

        if (dVal < 0)
            dScale = -scaleArray[index];
        else
            dScale = scaleArray[index];

        // return scaled value.
        return dScale;
    }



    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        if (m_imu != null)
            m_LastAngles = m_imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        m_GlobalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        if (m_imu == null)
            return 0;

        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = m_imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - m_LastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        m_GlobalAngle += deltaAngle;

        m_LastAngles = angles;

        return m_GlobalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate( double degrees, double power)
    {
        double  leftPower, rightPower;

        LinearOpMode opMode = (LinearOpMode)m_opMode;

        if (opMode == null)
            return;

            // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        SetDrivePower( leftPower, rightPower );

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opMode.opModeIsActive() && getAngle() == 0) {}

            while (opMode.opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opMode.opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        SetDrivePower( 0, 0 );

        // wait for rotation to stop.

        opMode.sleep( 1000 );

        // reset angle tracking on new heading.
        resetAngle();
    }
}

