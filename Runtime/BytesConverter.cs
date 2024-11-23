using System.Runtime.InteropServices;
using UnityEngine;

public static class BytesConverter
{
    public static byte[] GetBytes<T>(T str)
    {
        int size = Marshal.SizeOf(str);
        byte[] arr = new byte[size];

        GCHandle h = default;

        try
        {
            h = GCHandle.Alloc(arr, GCHandleType.Pinned);

            Marshal.StructureToPtr(str, h.AddrOfPinnedObject(), false);
        }
        finally
        {
            if (h.IsAllocated)
            {
                h.Free();
            }
        }

        return arr;
    }

    public static T FromBytes<T>(byte[] arr) where T : struct
    {
        T str = default;
        GCHandle h = default;

        if (arr.Length != Marshal.SizeOf<T>())
        {
            Debug.LogError("Invalid size of structure!");

            return str;
        }

        try
        {
            h = GCHandle.Alloc(arr, GCHandleType.Pinned);

            str = Marshal.PtrToStructure<T>(h.AddrOfPinnedObject());
        }
        finally
        {
            if (h.IsAllocated)
            {
                h.Free();
            }
        }

        return str;
    }
}