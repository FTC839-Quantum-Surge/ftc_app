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

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    public enum ClawStateEnum
    {
        Unknown,
        Closed,
        Open
    }

    public enum LiftPosEnum
    {
        None,
        Top,
        Hook,
        Transfer,
        Bottom
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

    private static final String DUMP_SERVO                  = "dump";
    private static final String CLAW_SERVO                  = "claw";

    private static final String LIMIT_TOP                   = "limitTop";
    private static final String LIMIT_BOTTOM                = "limitBottom";

    private static final int   LIFT_TOP                     = 22200;
    private static final int   LIFT_LATCH                   = 9250;

    // ----------------------------------------------------------------------
    // Public Member Variables
    // ----------------------------------------------------------------------

    private HardwareMap m_hwMap     = null;
    private ElapsedTime m_period    = new ElapsedTime();

    private DcMotor  m_leftFrontDrive      = null;
    private DcMotor  m_leftRearDrive       = null;
    private DcMotor  m_rightFrontDrive     = null;
    private DcMotor  m_rightRearDrive      = null;
    private DcMotor  m_intake              = null;
    private DcMotor  m_fold                = null;
    private DcMotor  m_lift                = null;

    private Servo    m_dump                = null;
    private Servo    m_claw                = null;

    // Using Analog inputs for Limit Switches due to
    // issue with hub not powering on when digital input held low

    private AnalogInput m_limitTop         = null;
    private AnalogInput m_limitBottom      = null;

    private int      m_nLiftEncHomePos     = 0;

    private ClawStateEnum   m_clawState    = ClawStateEnum.Unknown;
    private LiftPosEnum     m_targetPos    = LiftPosEnum.None;

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

        m_leftFrontDrive  = m_hwMap.get( DcMotor.class, LEFT_FRONT_DRIVE_MOTOR    );
        m_leftRearDrive   = m_hwMap.get( DcMotor.class, LEFT_REAR_DRIVE_MOTOR     );
        m_rightFrontDrive = m_hwMap.get( DcMotor.class, RIGHT_FRONT_DRIVE_MOTOR   );
        m_rightRearDrive  = m_hwMap.get( DcMotor.class, RIGHT_REAR_DRIVE_MOTOR    );
        m_intake          = m_hwMap.get( DcMotor.class, INTAKE_MOTOR              );
        m_fold            = m_hwMap.get( DcMotor.class, FOLD_MOTOR                );
        m_lift            = m_hwMap.get( DcMotor.class, LIFT_MOTOR                );

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

        m_lift            .setDirection( DcMotor.Direction.REVERSE );

        // ------------------------------------------------------------------
        // Set all motors to zero power
        // ------------------------------------------------------------------

        m_leftFrontDrive .setPower( 0 );
        m_leftRearDrive  .setPower( 0 );
        m_rightFrontDrive.setPower( 0 );
        m_rightRearDrive .setPower( 0 );
        m_intake         .setPower( 0 );
        m_fold           .setPower( 0 );
        m_lift           .setPower( 0 );

        // ------------------------------------------------------------------
        // Set all motors run Mode
        // ------------------------------------------------------------------

        m_lift            .setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        m_leftFrontDrive .setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        m_leftRearDrive  .setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        m_rightFrontDrive.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        m_rightRearDrive .setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        m_intake         .setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

        m_fold           .setMode( DcMotor.RunMode.RUN_TO_POSITION     );
        m_lift           .setMode( DcMotor.RunMode.RUN_USING_ENCODER   );

        // ------------------------------------------------------------------
        // Define and initialize ALL installed servos.
        // ------------------------------------------------------------------

        m_dump  = m_hwMap.get( Servo.class, DUMP_SERVO  );
        m_claw  = m_hwMap.get( Servo.class, CLAW_SERVO  );

        m_dump.setPosition( 0.5 );

        m_nLiftEncHomePos =  m_lift.getCurrentPosition();

        CloseClaw();
    }

    public boolean  GetLimitTop   () { return m_limitTop   .getVoltage() < 0.5; } //.getState(); }
    public boolean  GetLimitBottom() { return m_limitBottom.getVoltage() < 0.5; } //getState(); }

    public double  GetLimitTopVal   () { return m_limitTop   .getVoltage(); } //.getState(); }
    public double  GetLimitBottomVal() { return m_limitBottom.getVoltage(); } //getState(); }

    // //////////////////////////////////////////////////////////////////////
    // //////////////////////////////////////////////////////////////////////
    //
    //  Helper methods to manipulate motors/servos
    //
    // //////////////////////////////////////////////////////////////////////
    // //////////////////////////////////////////////////////////////////////

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
        m_lift.setPower(dPower);

    }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

   // public void SetLiftTarget( LiftPosEnum eTarget )
   // {
   //     targetPos = eTarget;
   // }

    public void SetLiftTarget( LiftPosEnum eTarget ) {

        if ( m_targetPos != LiftPosEnum.None )
            StopLift();

        int nTarget = 0;
        m_targetPos = eTarget;

        switch (m_targetPos)
        {
            case Top:       nTarget = m_nLiftEncHomePos + LIFT_TOP;     break;
            case Hook:      nTarget = m_nLiftEncHomePos + LIFT_LATCH;   break;
            case Bottom:    nTarget = m_nLiftEncHomePos;                break;

            case None:
            default:
                return;
        }

        m_lift.setTargetPosition( nTarget);

        // Turn On RUN_TO_POSITION
        m_lift.setMode( DcMotor.RunMode.RUN_TO_POSITION );

        m_lift.setPower( 1 );
    }

    public void StopLift()
    {
        m_lift.setPower( 0 );
        m_lift.setMode( DcMotor.RunMode.RUN_USING_ENCODER );

        m_targetPos = LiftPosEnum.None;
    }

    public int LiftPos()
    {
        return  m_lift.getCurrentPosition();
    }

    public void LiftPeriodicCheck( double dUserLiftPower )
    {
        double  dLiftPower   = m_lift.getPower();
        boolean bLimitTop    = GetLimitTop();
        boolean bLimitBottom = GetLimitBottom();

        // Is lift targeting a position?  If so, check to see if it reached the position.

        if ( m_targetPos != LiftPosEnum.None )
        {
            if (m_lift.isBusy() == false)
                StopLift();
        }

        // Is the user trying to manually move the stick, 0 power is okay if not targeting pos

        if ((dUserLiftPower != 0) || (m_targetPos == LiftPosEnum.None ))
        {
            // if Manual input, stop targeting

            if (m_targetPos != LiftPosEnum.None)
                StopLift();

            dLiftPower = dUserLiftPower;
        }

        // Check limit switches

        // We don't know the direction of the motor when targeting, don't check limit switch for now

        if (m_targetPos == LiftPosEnum.None )
        {
            if ((bLimitTop == false) && (dLiftPower > 0)) {
                StopLift();
                return;
            }
        }

        if ((bLimitBottom == false) && (dLiftPower < 0))
        {
            StopLift();
            return;
        }

        m_lift.setPower( dLiftPower );

    }
}

