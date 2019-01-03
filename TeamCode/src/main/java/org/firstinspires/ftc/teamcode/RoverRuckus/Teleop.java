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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop", group="Quantum")
//@Disabled
public class Teleop extends OpMode {

    /* Declare OpMode members. */
    private Robot    m_robot         = new Robot( this );
    private boolean  m_bStartPressed = false;

    // //////////////////////////////////////////////////////////////////////
    // Code to run ONCE when the driver hits INIT
    // //////////////////////////////////////////////////////////////////////

    @Override
    public void init()
    {
        m_robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
    }

    // //////////////////////////////////////////////////////////////////////
    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    // //////////////////////////////////////////////////////////////////////

    @Override
    public void init_loop()
    {
    }

    // //////////////////////////////////////////////////////////////////////
    // Code to run ONCE when the driver hits PLAY
    // //////////////////////////////////////////////////////////////////////

    @Override
    public void start()
    {
    }

    // //////////////////////////////////////////////////////////////////////
    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    // //////////////////////////////////////////////////////////////////////

    @Override
    public void loop()
    {
       double left;
       double right;

        // ------------------------------------------------------------------
        // Drive Wheels - Run in Arcade mode
        // (note: The joystick goes negative when pushed forwards, so negate it)
        // ------------------------------------------------------------------

        double drive = -gamepad1.right_stick_y;
        double turn  =  gamepad1.right_stick_x;
        left    = Range.clip(drive + turn, -1.0, 1.0) ;
        right   = Range.clip(drive - turn, -1.0, 1.0) ;

        m_robot.SetDrivePower( left, right);

        // ------------------------------------------------------------------
        // Claw
        // ------------------------------------------------------------------

        if (gamepad1.start)
            m_bStartPressed = true;

        if (!gamepad1.start && m_bStartPressed)
        {
            m_bStartPressed = false;

            if (m_robot.IsClawOpen())
                m_robot.CloseClaw();
            else
                m_robot.OpenClaw();
        }

        // ------------------------------------------------------------------
        // Lift
        // ------------------------------------------------------------------

        if (gamepad1.y)
            m_robot.Lift.SetTarget( Lift.PosEnum.Top, 1 );

        if (gamepad1.a)
            m_robot.Lift.SetTarget( Lift.PosEnum.Bottom, 1 );

        if (gamepad1.b)
            m_robot.Lift.SetTarget( Lift.PosEnum.Hook, 1 );

        m_robot.Lift.PeriodicCheck( gamepad1.right_trigger - gamepad1.left_trigger);

        // ------------------------------------------------------------------
        // Arm
        // ------------------------------------------------------------------

        // TODO:

        // Test Only
//        if (gamepad1.dpad_up)
//            m_robot.Arm.SetTarget( Arm.PosEnum.Top, 0.3, true );
//        else if (gamepad1.dpad_down)
//            m_robot.Arm.SetTarget( Arm.PosEnum.Bottom, 0.3, false );
//        else
//            m_robot.Arm.Stop();

        m_robot.Arm.PeriodicCheck( gamepad1.left_stick_y );

        double dSorterPos = Math.max( 1-(1/(Arm.ARM_TOP-Arm.ARM_BOTTOM)) * (double)m_robot.Arm.CurrentPos(), 0);

        m_robot.m_dump.setPosition( dSorterPos );

        // ------------------------------------------------------------------
        // Fold
        // ------------------------------------------------------------------

        // TODO:

        // Test Only
//        if (gamepad1.dpad_right)
//            m_robot.Fold.SetTarget( Fold.PosEnum.Floor, 0.1, false );
//
//        if (gamepad1.dpad_left)
//            m_robot.Fold.SetTarget( Fold.PosEnum.Folded, 0.1, false );
//
//        if (gamepad1.back)
//            m_robot.Fold.SetTarget( Fold.PosEnum.Vertical, 0.1, true );

        m_robot.Fold.PeriodicCheck( 0 );

        // ------------------------------------------------------------------
        // Send telemetry message to signify robot running;
        // ------------------------------------------------------------------

        telemetry.addData("Sorter Pos",  "%f", dSorterPos);
        telemetry.addData("Lift Pos",  "%d", m_robot.Lift.CurrentPos());
        telemetry.addData("Arm  Pos",  "%d", m_robot.Arm.CurrentPos());
        telemetry.addData("Fold Pos",  "%d", m_robot.Fold.CurrentPos());
        telemetry.addData("L. W Pos",  "%d", m_robot.GetLeftDrivePos());
        telemetry.addData("R. W Pos",  "%d", m_robot.GetRightDrivePos());
//        telemetry.addData("intake Pos",  "%d", m_robot.m_intake.getCurrentPosition());
        telemetry.addData( "left_y", "%f", gamepad1.left_stick_y );

        telemetry.addData("LeftTrigger", gamepad1.left_trigger );
        telemetry.addData( "RightTrigger", gamepad1.right_trigger );

    }

    // //////////////////////////////////////////////////////////////////////
    // Code to run ONCE after the driver hits STOP
    // //////////////////////////////////////////////////////////////////////

    @Override
    public void stop() {
    }
}
