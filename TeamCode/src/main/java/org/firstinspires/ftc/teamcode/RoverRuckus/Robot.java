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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    private static final String LEFT_FRONT_DRIVE_MOTOR      = "LF_drive";
    private static final String LEFT_REAR_DRIVE_MOTOR       = "LR_drive";
    private static final String RIGHT_FRONT_DRIVE_MOTOR     = "RF_drive";
    private static final String RIGHT_REAR_DRIVE_MOTOR      = "RR_drive";

    private static final String INTAKE_MOTOR                = "intake";
    private static final String FOLD_MOTOR                  = "fold";
    private static final String LIFT_MOTOR                  = "lift";
    private static final String ARM_MOTOR                   = "arm";

    private static final String DUMP_SERVO                  = "dump";
    private static final String CLAW_SERVO                  = "claw";
    private static final String PADDLE_SERVO                = "paddle";
    private static final String TILT_SERVO                  = "tilt";

    private static final String LIMIT_TOP                   = "limitTop";
    private static final String LIMIT_BOTTOM                = "limitBottom";

// Encoder Count Per rev
// NeverRest (7 ppr without gearbox)
//    40:1  = 280ppr
//    60:1  = 420ppr
//   104:1  = 728ppr
//
// Hex Core Motor (4 ppr without gearbox) 288 ppr at axle

    static final double     COUNTS_PER_MOTOR_REV    = 280 ;    // eg: NeverRest 40:1 (7 pulses at motor) Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = -(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // ----------------------------------------------------------------------
    // Public Member Variables
    // ----------------------------------------------------------------------

    private OpMode       m_opMode    = null;
    private HardwareMap  m_hwMap     = null;
    private ElapsedTime  m_period    = new ElapsedTime();
    private ElapsedTime  m_runtime   = new ElapsedTime();


    private DcMotor  m_leftFrontDrive      = null;
    private DcMotor  m_leftRearDrive       = null;
    private DcMotor  m_rightFrontDrive     = null;
    private DcMotor  m_rightRearDrive      = null;
    public DcMotor  m_intake              = null;
    private DcMotor  m_foldMotor           = null;
    private DcMotor  m_liftMotor           = null;
    private DcMotor  m_armMotor            = null;

    public  Servo    m_dump                = null;
    private Servo    m_claw                = null;
    public  Servo    m_paddle              = null;
    public  Servo    m_tilt                = null;

    // Using Analog inputs for Limit Switches due to
    // issue with hub not powering on when digital input held low

    private AnalogInput m_limitTop         = null;
    private AnalogInput m_limitBottom      = null;

    private ClawStateEnum   m_clawState    = ClawStateEnum.Unknown;

    // //////////////////////////////////////////////////////////////////////
    // Public Member Variables
    // //////////////////////////////////////////////////////////////////////

    public Lift    Lift = null;
    public Arm     Arm  = null;
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

    public void init( HardwareMap ahwMap )
    {
        // ------------------------------------------------------------------
        // Save reference to Hardware map
        // ------------------------------------------------------------------

        m_hwMap = ahwMap;

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
        m_armMotor        = m_hwMap.get( DcMotor.class, ARM_MOTOR                 );

        m_limitTop        = m_hwMap.get( AnalogInput.class, LIMIT_TOP          );
        m_limitBottom     = m_hwMap.get( AnalogInput.class, LIMIT_BOTTOM       );

        // ------------------------------------------------------------------
        //
        // ------------------------------------------------------------------

        //m_limitTop   .setMode( DigitalChannel.Mode.INPUT );
        //m_limitBottom.setMode( DigitalChannel.Mode.INPUT );

        // ------------------------------------------------------------------
        // Set direction of drive motors (one side needs to be opposite)
        // ------------------------------------------------------------------

        m_leftFrontDrive  .setDirection( DcMotor.Direction.FORWARD );
        m_leftRearDrive   .setDirection( DcMotor.Direction.FORWARD );

        m_rightFrontDrive .setDirection( DcMotor.Direction.REVERSE );
        m_rightRearDrive  .setDirection( DcMotor.Direction.REVERSE );

        m_liftMotor       .setDirection( DcMotor.Direction.REVERSE );
        m_armMotor        .setDirection( DcMotor.Direction.REVERSE );

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
        m_armMotor       .setPower( 0 );

        // ------------------------------------------------------------------
        // Set all motors run Mode
        // ------------------------------------------------------------------

        m_leftRearDrive  .setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        m_rightRearDrive .setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        m_foldMotor      .setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        m_liftMotor      .setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        m_armMotor       .setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        m_leftFrontDrive .setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        m_leftRearDrive  .setMode( DcMotor.RunMode.RUN_USING_ENCODER   );
        m_rightFrontDrive.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        m_rightRearDrive .setMode( DcMotor.RunMode.RUN_USING_ENCODER   );
        m_intake         .setMode( DcMotor.RunMode.RUN_USING_ENCODER   );

        m_foldMotor      .setMode( DcMotor.RunMode.RUN_USING_ENCODER   );
        m_liftMotor      .setMode( DcMotor.RunMode.RUN_USING_ENCODER   );
        m_armMotor       .setMode( DcMotor.RunMode.RUN_USING_ENCODER   );


        // ------------------------------------------------------------------
        // Define and initialize ALL installed servos.
        // ------------------------------------------------------------------

        m_dump   = m_hwMap.get( Servo.class, DUMP_SERVO  );
        m_claw   = m_hwMap.get( Servo.class, CLAW_SERVO  );
        m_paddle = m_hwMap.get( Servo.class, PADDLE_SERVO);
        m_tilt   = m_hwMap.get( Servo.class, TILT_SERVO  );

        m_dump  .setPosition( 1.0 );
        m_tilt  .setPosition( 0 );

        Lift = new Lift( m_liftMotor, m_limitTop, m_limitBottom );
        Arm  = new Arm ( m_armMotor, m_dump );
        Fold = new Fold( m_foldMotor );

        CloseClaw();
        CaptureElements();
    }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    public void AddTelemtry(Telemetry telemetry)
    {
        // telemetry.addData("Sorter Pos",  "%f", m_dump.getPosition());
        telemetry.addData( "paddle Pos", "%f", m_paddle.getPosition());

        telemetry.addData("Lift Pos",  "%d", Lift.CurrentPos());
        telemetry.addData("Arm  Pos",  "%d", Arm.CurrentPos());
        telemetry.addData("Fold Pos",  "%d", Fold.CurrentPos());
        telemetry.addData("L. W Pos",  "%d", GetLeftDrivePos());
        telemetry.addData("R. W Pos",  "%d", GetRightDrivePos());
    }

    // //////////////////////////////////////////////////////////////////////
    // //////////////////////////////////////////////////////////////////////
    //
    //  Helper methods to manipulate motors/servos
    //
    // //////////////////////////////////////////////////////////////////////
    // //////////////////////////////////////////////////////////////////////

    public int GetLeftDrivePos () { return m_leftRearDrive.getCurrentPosition(); }
    public int GetRightDrivePos() { return m_rightRearDrive.getCurrentPosition(); }

    // //////////////////////////////////////////////////////////////////////
    // Paddle
    // //////////////////////////////////////////////////////////////////////

    public void CaptureElements()
    {
        m_paddle.setPosition( 0.9 );
    }

    public void ReleaseElements()
    {
        m_paddle.setPosition( 0.5 );
    }

    public void SetPaddlePos(double dPos)
    {
        m_paddle.setPosition( dPos );
    }

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
        return ( m_clawState == ClawStateEnum.Open);
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

        if (opMode == null) {
            return;
        }

        if ( opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newLeftTarget  = m_leftRearDrive .getCurrentPosition() + (int)(leftInches  * COUNTS_PER_INCH);
            newRightTarget = m_rightRearDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            m_leftRearDrive .setTargetPosition(newLeftTarget);
            m_rightRearDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            m_leftRearDrive .setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m_rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            m_runtime.reset();

            SetDrivePower( Math.abs(speed), Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opMode.opModeIsActive() &&
                    (m_runtime.seconds() < timeoutS) &&
                    (m_leftRearDrive.isBusy() && m_rightRearDrive.isBusy())) {

                // Display it for the driver.

                m_opMode.telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                m_opMode.telemetry.addData("Path2",  "Running at %7d :%7d",
                        m_leftRearDrive.getCurrentPosition(),
                        m_rightRearDrive.getCurrentPosition());
                m_opMode.telemetry.update();

                Lift.PeriodicCheck( 0 );
            }

            // Stop all motion;
            SetDrivePower( 0, 0);

            // Turn off RUN_TO_POSITION
            m_leftRearDrive .setMode( DcMotor.RunMode.RUN_USING_ENCODER );
            m_rightRearDrive.setMode( DcMotor.RunMode.RUN_USING_ENCODER );

            //  sleep(250);   // optional pause after each move
        }
    }

    // //////////////////////////////////////////////////////////////////////
    // TODO: Use encoders AND IMU (gyro) to drive a specific distance in a
    //       straight line.
    // //////////////////////////////////////////////////////////////////////

    public void DriveStraigt( double dPower, int nInches )
    {
        // TODO
    }

    // //////////////////////////////////////////////////////////////////////
    // TODO: Use IMU (gyro) or Encoders to turn specified number of degrees
    // //////////////////////////////////////////////////////////////////////

    public void PivitTurn( double dPower, double dDegrees )
    {
        // ------------------------------------------------------------------
        // If requested to turn counter clockwise, negate dPower;
        // ------------------------------------------------------------------

        dPower *= (dDegrees < 0) ? -1 : 1;

        // 1) get current Gyro heading;
        // 2) SetDrivePower( dPower, -dPower );
        // 3) wait for Gyro to meet or exceed requested value
        // SetDrivePower( 0, 0 );   // Stop Motots

    }
}

