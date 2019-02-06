package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class Lift  extends Targetable< Lift.PosEnum >
{
    private static final int   LIFT_TOP                     = 21300;
    private static final int   LIFT_SAFE_TO_DUMP            = 17500;
    private static final int   LIFT_LATCH                   = 10541; //11000=not working  //11500   //12000;    //10000;

    private AnalogInput m_limitTop;
    private AnalogInput m_limitBottom;

    public enum PosEnum
    {
        None,
        Top,
        SafeToDump,
        Hook,
        Bottom
    }
    // //////////////////////////////////////////////////////////////////////
    // override to implement limit switches
    // //////////////////////////////////////////////////////////////////////

    public Lift( DcMotor motor, AnalogInput limitTop, AnalogInput limitBottom )
    {
        super( motor );

        m_eTargetPos = PosEnum.None;

        m_limitBottom = limitBottom;
        m_limitTop    = limitTop;
    }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    @Override protected Lift.PosEnum GetNotTargetingValue() { return PosEnum.None; }
    @Override protected Lift.PosEnum GetTopTarget()         { return PosEnum.Top;  }

    @Override public boolean  GetLimitTop   () { return m_limitTop   .getVoltage() > 0.5; }
    @Override public boolean  GetLimitBottom() { return m_limitBottom.getVoltage() > 0.5; }

    public double  GetLimitTopVal   () { return m_limitTop   .getVoltage(); }
    public double  GetLimitBottomVal() { return m_limitBottom.getVoltage(); }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    @Override
    public int GetEncoderTargetValue( Lift.PosEnum eTarget)
    {
        switch ( eTarget)
        {
            case Top:        return LIFT_TOP          + m_nHomeEncPos;
            case SafeToDump: return LIFT_SAFE_TO_DUMP + m_nHomeEncPos;
            case Hook:       return LIFT_LATCH        + m_nHomeEncPos;
            case Bottom:     return m_nHomeEncPos;
        }

        return 0;
    }
}
