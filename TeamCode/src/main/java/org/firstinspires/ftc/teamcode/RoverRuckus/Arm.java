package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends ManualTargeting< Arm.PosEnum >
{
    public static final int   ARM_TOP                      = 132;
    public static final int   ARM_BOTTOM                   = 0;

    private Servo m_sorter;

    public enum PosEnum
    {
        None,
        Top,
        Bottom
    }
    // //////////////////////////////////////////////////////////////////////
    // override to implement limit switches
    // //////////////////////////////////////////////////////////////////////

    public Arm( DcMotor motor, Servo sorter )
    {
        super( motor );

        m_eTargetPos = PosEnum.None;

        m_sorter = sorter;
    }

    @Override public int    DeadZone() { return 5; }

    @Override public double ScalePower( double dPower, int nAbsAmtToGo )
    {
        if (nAbsAmtToGo < 50)
            return dPower * .50;

        if (nAbsAmtToGo < 25)
            return dPower * .25;

        return dPower;
    }

    // //////////////////////////////////////////////////////////////////////
    // Override stop method and make it maintain current location if stopped.
    // //////////////////////////////////////////////////////////////////////

    /*
    @Override
    public void Stop()
    {
        m_motor.setTargetPosition( m_motor.getCurrentPosition() );
//        m_motor.setPower( 0 );
//        m_motor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
//
//        m_eTargetPos = GetNotTargetingValue();
    }
*/

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    @Override protected Arm.PosEnum GetNotTargetingValue() { return PosEnum.None; }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    @Override
    protected int GetEncoderTargetValue( Arm.PosEnum eTarget)
    {
        switch ( eTarget)
        {
            case Top:       return ARM_TOP;
            case Bottom:    return ARM_BOTTOM;
        }

        return 0;
    }

    @Override
    public boolean PeriodicCheck( double dOverridePower )
    {
        boolean bResult = super.PeriodicCheck( dOverridePower );

        // Rotate sorter based on arm position.

        double dSorterPos = Math.max( 1.0f-(1.0f/(Arm.ARM_TOP-Arm.ARM_BOTTOM)) * (double)CurrentPos(), 0.0f);

        m_sorter.setPosition( dSorterPos );

        return bResult;
    }

}
