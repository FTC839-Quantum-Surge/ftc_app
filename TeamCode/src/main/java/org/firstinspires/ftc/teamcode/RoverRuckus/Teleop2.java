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
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name="Teleop2", group="Quantum")
@Disabled
public class Teleop2 extends OpMode {

    /* Declare OpMode members. */
    private Robot    m_robot         = new Robot( this );
    private boolean  m_bStartPressed = false;

    // //////////////////////////////////////////////////////////////////////
    // Code to run ONCE when the driver hits INIT
    // //////////////////////////////////////////////////////////////////////

    @Override
    public void init()
    {
        m_robot.init(hardwareMap, true);

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

    double dPos = 0;

    @Override
    public void loop()
    {
        double left;
        double right;

        // //////////////////////////////////////////////////////////////////
        // //////////////////////////////////////////////////////////////////
        //
        // Driver Controller Actions (gamepad1)
        //
        // //////////////////////////////////////////////////////////////////
        // //////////////////////////////////////////////////////////////////

        // ------------------------------------------------------------------
        // Drive Wheels - Run in Arcade mode
        // (note: The joystick goes negative when pushed forwards, so negate it)
        // ------------------------------------------------------------------

        double drive = -gamepad1.left_stick_y;
        double turn  = -gamepad1.left_stick_x;
        left    = Range.clip(drive + turn, -1.0, 1.0) ;
        right   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Adjust for dead zone...
        if (Math.abs( left ) < .1)
            left = 0;

        if (Math.abs( right ) < .1)
            left = 0;

        // Override is pivot turn

        if (gamepad1.right_bumper)
        {
            right = -1;
            left  = 1;
        }

        if (gamepad1.left_bumper)
        {
            right = 1;
            left  = -1;
        }

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


        // //////////////////////////////////////////////////////////////////
        // //////////////////////////////////////////////////////////////////
        //
        // Accessory Controller Actions  (gamepad2)
        //
        // //////////////////////////////////////////////////////////////////
        // //////////////////////////////////////////////////////////////////


        // ------------------------------------------------------------------
        // Fold
        // ------------------------------------------------------------------

        if (gamepad2.dpad_up)
            m_robot.Fold.SetTarget( Fold.PosEnum.Folded, 1, false );

        if (gamepad2.dpad_down)
            m_robot.Fold.SetTarget( Fold.PosEnum.Floor, 1, false );

        if (gamepad2.dpad_right)
            m_robot.Fold.SetTarget( Fold.PosEnum.Vertical, 1, true );

        m_robot.Fold.PeriodicCheck(  -(gamepad2.right_stick_y * 1) );

        // ------------------------------------------------------------------
        // intake
        // ------------------------------------------------------------------

        m_robot.SetIntakePower( gamepad2.right_trigger - gamepad2.left_trigger);



        // ------------------------------------------------------------------
        // Lift
        // ------------------------------------------------------------------

        if (gamepad2.y)
            m_robot.Lift.SetTarget( Lift.PosEnum.Top, 1 );

        if (gamepad2.a)
            m_robot.Lift.SetTarget( Lift.PosEnum.Bottom, 1 );

        if (gamepad2.b)
            m_robot.Lift.SetTarget( Lift.PosEnum.Hook, 1 );


        m_robot.Lift.PeriodicCheck(  -(gamepad2.left_stick_y * 1 ) );

        // ------------------------------------------------------------------
        // Arm
        // ------------------------------------------------------------------

       // if (gamepad2.dpad_up)
       //     m_robot.Arm.SetTarget( Arm.PosEnum.Top, 0.20, false );

       // if (gamepad2.dpad_down)
        // .Arm.SetTarget( Arm.PosEnum.Bottom, 0.15, false );

       // m_robot.Arm.PeriodicCheck( gamepad2.left_stick_y * 0.55    );

        // ------------------------------------------------------------------
        // Release / capture Elements in sorter
        // ------------------------------------------------------------------

        // if (gamepad2.right_bumper)
        //   m_robot.ReleaseElements();

        //if (gamepad2.left_bumper)
        //  m_robot.CaptureElements();

        // ------------------------------------------------------------------
        // Raise / lower Tilt
        // ------------------------------------------------------------------

//        if (gamepad2.x)
//            m_robot.TiltUp();
//        else
//            m_robot.TiltDown();

        // ------------------------------------------------------------------
        // Send telemetry message to signify robot running;
        // ------------------------------------------------------------------

        m_robot.AddTelemtry( telemetry );

        telemetry.addData( "right1_y", "%f", gamepad2.right_stick_y );
        telemetry.addData( "right2_y", "%f", gamepad2.left_stick_y );

    //    telemetry.addData("LeftTrigger", gamepad1.left_trigger );
    //    telemetry.addData( "RightTrigger", gamepad1.right_trigger );

        telemetry.update();

    }

    // //////////////////////////////////////////////////////////////////////
    // Code to run ONCE after the driver hits STOP
    // //////////////////////////////////////////////////////////////////////

    @Override
    public void stop() {
    }
}
