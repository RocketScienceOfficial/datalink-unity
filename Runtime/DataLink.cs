using System.Runtime.InteropServices;

public enum DataLinkMessageType : byte
{
    DATALINK_MESSAGE_TELEMETRY_DATA_OBC,
    DATALINK_MESSAGE_TELEMETRY_DATA_OBC_WITH_RESPONSE,
    DATALINK_MESSAGE_TELEMETRY_DATA_GCS,
    DATALINK_MESSAGE_TELEMETRY_RESPONSE,
    DATALINK_MESSAGE_OBC_APP_EKF_TRANSMIT_START,
    DATALINK_MESSAGE_OBC_APP_EKF_TRANSMIT_END,
    DATALINK_MESSAGE_OBC_APP_EKF_DATA,
    DATALINK_MESSAGE_DATA_SAVED_CHUNK,
    DATALINK_MESSAGE_DATA_SAVED_SIZE,
    DATALINK_MESSAGE_DATA_REQUEST_READ,
    DATALINK_MESSAGE_DATA_FINISH_READ,
    DATALINK_MESSAGE_DATA_REQUEST_CLEAR,
    DATALINK_MESSAGE_DATA_PROGRESS_CLEAR,
    DATALINK_MESSAGE_DATA_FINISH_CLEAR,
    DATALINK_MESSAGE_DATA_REQUEST_RECOVERY,
    DATALINK_MESSAGE_DATA_FINISH_RECOVERY,
    DATALINK_MESSAGE_IGN_REQUEST_TEST,
    DATALINK_MESSAGE_IGN_FINISH_TEST,
    DATALINK_MESSAGE_CONFIG_GET,
    DATALINK_MESSAGE_CONFIG_GET_ACK,
    DATALINK_MESSAGE_CONFIG_SET,
    DATALINK_MESSAGE_CONFIG_SET_ACK,
    DATALINK_MESSAGE_RADIO_MODULE_TX_DONE,
    DATALINK_MESSAGE_NONE,
}

public enum DataLinkOptionsSMState : byte
{
    DATALINK_OPTIONS_SM_STATE_STANDING = 0,
    DATALINK_OPTIONS_SM_STATE_ACCELERATING = 1,
    DATALINK_OPTIONS_SM_STATE_FREE_FLIGHT = 2,
    DATALINK_OPTIONS_SM_STATE_FREE_FALL = 3,
    DATALINK_OPTIONS_SM_STATE_LANDED = 4,
}

public enum DataLinkFlagsTelemetryDataControlFlags : byte
{
    DATALINK_FLAGS_TELEMETRY_DATA_CONTROL_ARM_ENABLED = 1 << 0,
    DATALINK_FLAGS_TELEMETRY_DATA_CONTROL_3V3_ENABLED = 1 << 1,
    DATALINK_FLAGS_TELEMETRY_DATA_CONTROL_5V_ENABLED = 1 << 2,
    DATALINK_FLAGS_TELEMETRY_DATA_CONTROL_VBAT_ENABLED = 1 << 3,
    DATALINK_FLAGS_TELEMETRY_DATA_CONTROL_IGN_1 = 1 << 4,
    DATALINK_FLAGS_TELEMETRY_DATA_CONTROL_IGN_2 = 1 << 5,
    DATALINK_FLAGS_TELEMETRY_DATA_CONTROL_IGN_3 = 1 << 6,
    DATALINK_FLAGS_TELEMETRY_DATA_CONTROL_IGN_4 = 1 << 7,
}

public enum DataLinkFlagsTelemetryResponseControlFlags : byte
{
    DATALINK_FLAGS_TELEMETRY_RESPONSE_CONTROL_ARM_ENABLED = 1 << 0,
    DATALINK_FLAGS_TELEMETRY_RESPONSE_CONTROL_3V3_ENABLED = 1 << 1,
    DATALINK_FLAGS_TELEMETRY_RESPONSE_CONTROL_5V_ENABLED = 1 << 2,
    DATALINK_FLAGS_TELEMETRY_RESPONSE_CONTROL_VBAT_ENABLED = 1 << 3,
}

public enum DataLinkFlagsIGN : byte
{
    DATALINK_FLAGS_IGN_1_CONT = 1 << 0,
    DATALINK_FLAGS_IGN_2_CONT = 1 << 1,
    DATALINK_FLAGS_IGN_3_CONT = 1 << 2,
    DATALINK_FLAGS_IGN_4_CONT = 1 << 3,
    DATALINK_FLAGS_IGN_1_STATE = 1 << 4,
    DATALINK_FLAGS_IGN_2_STATE = 1 << 5,
    DATALINK_FLAGS_IGN_3_STATE = 1 << 6,
    DATALINK_FLAGS_IGN_4_STATE = 1 << 7,
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct DataLinkFrameTelemetryDataOBC
{
    public float qw;
    public float qx;
    public float qy;
    public float qz;
    public ushort velocity;
    public byte batteryVoltage10;
    public byte batteryPercentage;
    public double lat;
    public double lon;
    public ushort alt;
    public byte state;
    public byte controlFlags;
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct DataLinkFrameTelemetryDataGCS
{
    public float qw;
    public float qx;
    public float qy;
    public float qz;
    public ushort velocity;
    public byte batteryVoltage10;
    public byte batteryPercentage;
    public double lat;
    public double lon;
    public ushort alt;
    public byte state;
    public byte controlFlags;
    public byte signalStrengthNeg;
    public byte packetLossPercentage;
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct DataLinkFrameTelemetryResponse
{
    public byte controlFlags;
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct DataLinkFrameOBCAppEKFData
{
    public float qw;
    public float qx;
    public float qy;
    public float qz;
    public float positionN;
    public float positionE;
    public float positionD;
    public float velocityN;
    public float velocityE;
    public float velocityD;
    public float accelerationN;
    public float accelerationE;
    public float accelerationD;
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct DataLinkFrameDataSavedChunk
{
    public ushort dt;
    public float accX;
    public float accY;
    public float accZ;
    public float velN;
    public float velE;
    public float velD;
    public float posN;
    public float posE;
    public float posD;
    public float qw;
    public float qx;
    public float qy;
    public float qz;
    public double lat;
    public double lon;
    public double alt;
    public byte smState;
    public byte batVolts10;
    public byte ignFlags;
    public byte gpsData;
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct DataLinkFrameDataSavedSize
{
    public uint size;
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct DataLinkFrameDataProgressClear
{
    public byte percentage;
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct DataLinkFrameIGNRequestTest
{
    public byte ignNum;
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct DataLinkFrameConfigGet
{
    public ushort mainHeight;
    public byte emergencySpeed;
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct DataLinkFrameConfigSet
{
    public ushort mainHeight;
    public byte emergencySpeed;
}

public class DataLinkFrame
{
    public byte magic_serial;
    public DataLinkMessageType msgId;
    public byte len;
    public byte[] payload;
    public ushort crc;
}

public static class DataLink
{
    public const byte DATALINK_MAGIC_SERIAL = 0x7E;

    private static readonly byte[] SerializationData = new byte[512];

    public static byte[] Serialize(DataLinkFrame frame)
    {
        int offset = 0;

        SerializationData[0] = DATALINK_MAGIC_SERIAL;
        SerializationData[1] = (byte)frame.msgId;

        offset += 3;

        if (frame.payload != null)
        {
            SerializationData[2] = (byte)frame.payload.Length;

            for (var i = 0; i < frame.payload.Length; i++)
            {
                SerializationData[offset + i] = frame.payload[i];
            }

            offset += frame.payload.Length;
        }
        else
        {
            SerializationData[2] = 0x00;
        }

        ushort crc = CalculateCRC16(SerializationData, offset);

        SerializationData[offset + 0] = (byte)(crc & 0xff);
        SerializationData[offset + 1] = (byte)(crc >> 8);

        offset += 2;

        byte[] returnData = new byte[offset];

        for (var i = 0; i < returnData.Length; i++)
        {
            returnData[i] = SerializationData[i];
        }

        return returnData;
    }

    public static DataLinkFrame Deserialize(byte[] data)
    {
        if (data.Length < 5)
        {
            return null;
        }

        var frame = new DataLinkFrame();

        frame.magic_serial = data[0];

        if (frame.magic_serial != DATALINK_MAGIC_SERIAL)
        {
            return null;
        }

        frame.crc |= data[data.Length - 2];
        frame.crc |= (ushort)(data[data.Length - 1] << 8);

        ushort crc = CalculateCRC16(data, data.Length - 2);

        if (crc != frame.crc)
        {
            return null;
        }

        frame.msgId = (DataLinkMessageType)data[1];

        if ((byte)frame.msgId >= (byte)DataLinkMessageType.DATALINK_MESSAGE_NONE)
        {
            return null;
        }

        frame.len = data[2];

        if (frame.len != data.Length - 5)
        {
            return null;
        }

        if (frame.len > 0)
        {
            frame.payload = new byte[frame.len];

            for (var i = 0; i < frame.len; i++)
            {
                frame.payload[i] = data[3 + i];
            }
        }
        else
        {
            frame.payload = null;
        }

        return frame;
    }

    private static ushort CalculateCRC16(byte[] data, int len)
    {
        if (data.Length < len)
        {
            return 0x0000;
        }

        ushort crc = 0xffff;
        byte t, L;

        for (int i = 0; i < len; i++)
        {
            crc ^= data[i];
            L = (byte)(crc ^ (crc << 4));
            t = (byte)((L << 3) | (L >> 5));
            L ^= (byte)(t & 0x07);
            t = (byte)((t & 0xF8) ^ (((t << 1) | (t >> 7)) & 0x0F) ^ (byte)(crc >> 8));
            crc = (ushort)((L << 8) | t);
        }

        return crc;
    }
}