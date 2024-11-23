using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class COBS
{
    public static byte[] Encode(byte[] data)
    {
        var list = new List<byte>();
        var code = (byte)0x01;
        var lastIndex = 0;

        list.Add(0x00);

        for (var i = 0; i < data.Length; i++)
        {
            var b = data[i];

            if (b > 0x00)
            {
                list.Add(b);
                code++;
            }

            if (b == 0x00 || code == 0xff)
            {
                list[lastIndex] = code;
                code = 1;

                list.Add(0x00);
                lastIndex = list.Count - 1;
            }
        }

        list[lastIndex] = code;
        list.Add(0x00);

        return list.ToArray();
    }

    public static byte[] Decode(byte[] data, int len)
    {
        var list = new List<byte>();
        var code = (byte)0xff;
        var block = 0;

        for (var i = 0; i < len; i++)
        {
            var b = data[i];

            if (block > 0)
            {
                list.Add(b);
            }
            else
            {
                if (code != 0xff)
                {
                    list.Add(0x00);
                }

                code = b;
                block = code;

                if (code == 0x00)
                {
                    break;
                }
            }

            block--;
        }

        list.RemoveAt(list.Count - 1);

        return list.ToArray();
    }
}