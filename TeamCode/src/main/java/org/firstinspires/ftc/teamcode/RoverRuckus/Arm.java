package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends Targetable< Arm.PosEnum >
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
/*
    @Override
    public boolean PeriodicCheck( double dOverridePower )
    {
        boolean bResult = super.PeriodicCheck( dOverridePower );

        // Rotate sorter based on arm position.

        double dSorterPos = (1/(ARM_TOP-ARM_BOTTOM)) * CurrentPos();

        m_sorter.setPosition( dSorterPos );

        return bResult;
    }
*/
}
