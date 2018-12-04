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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is the Hardware Configuration class for the 2018-2019 Rover Ruckus design.
 *
 * All motors, sensors or other robot configuration specific properties will be
 * defined here and shared with all op modes
 *
 */
public class Robot
{
    // ----------------------------------------------------------------------
    // Private Member Variables
    // ----------------------------------------------------------------------

    private HardwareMap m_hwMap     =  null;
    private ElapsedTime m_period    = new ElapsedTime();

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

    private static final String DUMP_SERVO                  = "dump";
    private static final String LATCH_SERVO                 = "latch";

    // ----------------------------------------------------------------------
    // Public Member Variables
    // ----------------------------------------------------------------------

    private DcMotor  leftFrontDrive      = null;
    private DcMotor  leftRearDrive       = null;
    private DcMotor  rightFrontDrive     = null;
    private DcMotor  rightRearDrive      = null;
    private DcMotor  intake              = null;
    private DcMotor  fold                = null;
    private DcMotor  lift                = null;

    private Servo    dump                = null;
    private Servo    latch               = null;

    // //////////////////////////////////////////////////////////////////////
    // Constructor
    // //////////////////////////////////////////////////////////////////////

    public Robot()
    {
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

        leftFrontDrive  = m_hwMap.get( DcMotor.class, LEFT_FRONT_DRIVE_MOTOR    );
        leftRearDrive   = m_hwMap.get( DcMotor.class, LEFT_REAR_DRIVE_MOTOR     );
        rightFrontDrive = m_hwMap.get( DcMotor.class, RIGHT_FRONT_DRIVE_MOTOR   );
        rightRearDrive  = m_hwMap.get( DcMotor.class, RIGHT_REAR_DRIVE_MOTOR    );
        intake          = m_hwMap.get( DcMotor.class, INTAKE_MOTOR              );
        fold            = m_hwMap.get( DcMotor.class, FOLD_MOTOR                );
        lift            = m_hwMap.get( DcMotor.class, LIFT_MOTOR                );

        // ------------------------------------------------------------------
        // Set direction of drive motors (one side needs to be opposite)
        // ------------------------------------------------------------------

        leftFrontDrive  .setDirection( DcMotor.Direction.FORWARD );
        leftRearDrive   .setDirection( DcMotor.Direction.FORWARD );

        rightFrontDrive .setDirection( DcMotor.Direction.REVERSE );
        rightRearDrive  .setDirection( DcMotor.Direction.REVERSE );

        // ------------------------------------------------------------------
        // Set all motors to zero power
        // ------------------------------------------------------------------

        leftFrontDrive .setPower( 0 );
        leftRearDrive  .setPower( 0 );
        rightFrontDrive.setPower( 0 );
        rightRearDrive .setPower( 0 );
        intake         .setPower( 0 );
        fold           .setPower( 0 );
        lift           .setPower( 0 );

        // ------------------------------------------------------------------
        // Set all motors run Mode
        // ------------------------------------------------------------------

        leftFrontDrive .setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        leftRearDrive  .setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        rightFrontDrive.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        rightRearDrive .setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        intake         .setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

        fold           .setMode( DcMotor.RunMode.RUN_TO_POSITION     );
        lift           .setMode( DcMotor.RunMode.RUN_TO_POSITION     );

        // ------------------------------------------------------------------
        // Define and initialize ALL installed servos.
        // ------------------------------------------------------------------

        dump  = m_hwMap.get( Servo.class, DUMP_SERVO  );
        latch = m_hwMap.get( Servo.class, LATCH_SERVO );

        dump.setPosition ( 0.5 );
        latch.setPosition( 0.5 );
    }

    // //////////////////////////////////////////////////////////////////////
    // //////////////////////////////////////////////////////////////////////
    //
    //  Helper methods to manipulate motors/servos
    //
    // //////////////////////////////////////////////////////////////////////
    // //////////////////////////////////////////////////////////////////////

    // //////////////////////////////////////////////////////////////////////
    // Set manual left and right drive motor power (Tank Drive)
    // //////////////////////////////////////////////////////////////////////

    public void SetDrivePower( double dLeftPower, double dRightPower )
    {
        leftFrontDrive .setPower( dLeftPower );
        leftRearDrive  .setPower( dLeftPower );

        rightFrontDrive.setPower( dRightPower );
        rightRearDrive .setPower( dRightPower );
    }

    // //////////////////////////////////////////////////////////////////////
    // TODO: Use encoders to drive a specific distance.
    // //////////////////////////////////////////////////////////////////////

    public void DriveDistance( double dLeftPower, double dRightPower, int nInches )
    {
        // NOTES: Since each side can be different power levels (slight turn)
        //        which encoder should be used to determine distance (slower or faster motor)?
        //        If turn is not needed, remove one of the power parameters.

        // 1) Read current encoder values
        // 2) SetDrivePower( dLeftPower, dRightPower );
        // 3) Wait until encoder meets or exceeds requested value
        // 4) SetDrivePower( 0, 0 );    // Stop Motors
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

    public void SetLiftMotorPower(double dPower )
    {
        lift.setPower(dPower);
    }


 }

