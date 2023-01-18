package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import java.lang.Math;



public class MPU6050 {
    private final I2C m_i2c;
    public MPU6050(int addr)
    {
        m_i2c = new I2C(Port.kOnboard, addr);
        m_i2c.write(0x6B, 0x00);
        m_i2c.write(0x1C, 0);
        m_i2c.write(0x19,0x07);
        m_i2c.write(0x1B,0x0);
        m_i2c.write(0x1C,0x0);
      //  m_i2c.notifyAll();
       
        // System.out.print(m_i2c.verifySensor(addr, 1, null));
    }

    public MPU6050()
    {
        this(0x68);
    }

    public double readXAxisGyro()
    {
        byte[] xBuffer = new byte[2];
        xBuffer[0] = xBuffer[1] = 0;
        int xGyro = 0;
        m_i2c.read(0x43, 1, xBuffer);
        xGyro =  (byte)xBuffer[0]<<8 | (byte)xBuffer[1];
        return xGyro/131.0; 
    }

    public double readYAxisGyro()
    {
        byte[] yBuffer = new byte[2];
        yBuffer[0] = yBuffer[1] = 0;
        int yGyro = 0;
        m_i2c.read(0x45, 1, yBuffer);
        yGyro =  (byte)yBuffer[0]<<8 | (byte)yBuffer[1];
        return yGyro/131.0; 
    }
    
    public double readZAxisGyro()
    {
        byte[] zBuffer = new byte[2];
        zBuffer[0] = zBuffer[1] = 0;
        int zGyro = 0;
        m_i2c.read(0x47, 1, zBuffer);
        zGyro =  (byte)zBuffer[0]<<8 | (byte)zBuffer[1];
        return zGyro/131.0; 
    }

    public double readXAxisAccel()
    {
        byte[] xBuffer = new byte[2];
        xBuffer[0] = xBuffer[1] = 0;
        int xAccel = 0;
        m_i2c.read(0x3B, 1, xBuffer);
        xAccel =  (byte)xBuffer[0]<<8 | (byte)xBuffer[1];
        return xAccel/13824.0; 
    }

    public double readYAxisAccel()
    {
        byte[] yBuffer = new byte[2];
        yBuffer[0] = yBuffer[1] = 0;
        int yAccel = 0;
        m_i2c.read(0x3D, 1, yBuffer);
        yAccel =  (byte)yBuffer[0]<<8 | (byte)yBuffer[1];
        return yAccel/13824.0; 
    }

    public double readZAxisAccel()
    {
        byte[] zBuffer = new byte[2];
        zBuffer[0] = zBuffer[1] = 0;
        int zAccel = 0;
        m_i2c.read(0x3F, 1, zBuffer);
        zAccel =  (byte)zBuffer[0]<<8 | (byte)zBuffer[1];
        return zAccel/13824.0; 
    }

    public double readXAxisAngle()
    {
        double xAngle;

        xAngle = Math.toDegrees(Math.atan(-this.readYAxisAccel()/-this.readZAxisAccel())*Math.PI/2.0);
        return xAngle;
    }

    public double readYAxisAngle()
    {
        double yAngle;

        yAngle = Math.toDegrees(Math.atan(-this.readXAxisAccel()/-this.readZAxisAccel())*Math.PI/2.0);
        return yAngle;
    }

    public double readZAxisAngle()
    {
        double zAngle;

        zAngle = Math.toDegrees(Math.atan(-this.readYAxisAccel()/-this.readXAxisAccel())*Math.PI/2.0);
        return zAngle;
    }
}

