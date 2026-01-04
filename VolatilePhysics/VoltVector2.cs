/*
 *  VolatilePhysics - A 2D Physics Library for Networked Games
 *  Copyright (c) 2015-2016 - Alexander Shoulson - http://ashoulson.com
 *
 *  This software is provided 'as-is', without any express or implied
 *  warranty. In no event will the authors be held liable for any damages
 *  arising from the use of this software.
 *  Permission is granted to anyone to use this software for any purpose,
 *  including commercial applications, and to alter it and redistribute it
 *  freely, subject to the following restrictions:
 *  
 *  1. The origin of this software must not be misrepresented; you must not
 *     claim that you wrote the original software. If you use this software
 *     in a product, an acknowledgment in the product documentation would be
 *     appreciated but is not required.
 *  2. Altered source versions must be plainly marked as such, and must not be
 *     misrepresented as being the original software.
 *  3. This notice may not be removed or altered from any source distribution.
*/

using System;
using FixMath.NET;

namespace Volatile
{
    public struct VoltVector2 : IEquatable<VoltVector2>
    {
        public Fix64 x;
        public Fix64 y;

        private static readonly VoltVector2 zeroVector = new VoltVector2(Fix64.Zero);
        private static readonly VoltVector2 unitVector = new VoltVector2(Fix64.One, Fix64.One);
        private static readonly VoltVector2 unitXVector = new VoltVector2(Fix64.One, Fix64.Zero);
        private static readonly VoltVector2 unitYVector = new VoltVector2(Fix64.Zero, Fix64.One);

        public static VoltVector2 zero => zeroVector;
        public static VoltVector2 one => unitVector;
        public static VoltVector2 unitX => unitXVector;
        public static VoltVector2 unitY => unitYVector;

        public VoltVector2(Fix64 x, Fix64 y)
        {
            this.x = x;
            this.y = y;
        }

        public VoltVector2(Fix64 value)
        {
            this.x = value;
            this.y = value;
        }

        public Fix64 sqrMagnitude
        {
            get
            {
                return (this.x * this.x) + (this.y * this.y);
            }
        }

        public Fix64 magnitude
        {
            get
            {
                return VoltMath.Sqrt(this.sqrMagnitude);
            }
        }

        public VoltVector2 normalized
        {
            get
            {
                Fix64 magnitude = this.magnitude;
                return new VoltVector2(this.x / magnitude, this.y / magnitude);
            }
        }

        public static VoltVector2 operator *(VoltVector2 a, Fix64 b)
        {
            return new VoltVector2(a.x * b, a.y * b);
        }

        public static VoltVector2 operator *(Fix64 a, VoltVector2 b)
        {
            return new VoltVector2(b.x * a, b.y * a);
        }

        public static VoltVector2 operator /(VoltVector2 value1, VoltVector2 value2)
        {
            value1.x /= value2.x;
            value1.y /= value2.y;
            return value1;
        }

        public static VoltVector2 operator /(VoltVector2 value1, Fix64 divider)
        {
            Fix64 num = Fix64.One / divider;
            value1.x *= num;
            value1.y *= num;
            return value1;
        }

        public static VoltVector2 operator +(VoltVector2 a, VoltVector2 b)
        {
            return new VoltVector2(a.x + b.x, a.y + b.y);
        }

        public static VoltVector2 operator -(VoltVector2 a, VoltVector2 b)
        {
            return new VoltVector2(a.x - b.x, a.y - b.y);
        }

        public static VoltVector2 operator -(VoltVector2 a)
        {
            return new VoltVector2(-a.x, -a.y);
        }

        public static bool operator ==(VoltVector2 a, VoltVector2 b)
        {
            return a.x == b.x && a.y == b.y;
        }

        public static bool operator !=(VoltVector2 a, VoltVector2 b)
        {
            return !(a == b);
        }

        public static VoltVector2 Add(VoltVector2 value1, VoltVector2 value2)
        {
            value1.x += value2.x;
            value1.y += value2.y;
            return value1;
        }

        public static void Add(ref VoltVector2 value1, ref VoltVector2 value2, out VoltVector2 result)
        {
            result.x = value1.x + value2.x;
            result.y = value1.y + value2.y;
        }

        public static VoltVector2 Subtract(VoltVector2 value1, VoltVector2 value2)
        {
            value1.x -= value2.x;
            value1.y -= value2.y;
            return value1;
        }
        
        public static void Subtract(ref VoltVector2 value1, ref VoltVector2 value2, out VoltVector2 result)
        {
            result.x = value1.x - value2.x;
            result.y = value1.y - value2.y;
        }

        public static VoltVector2 Multiply(VoltVector2 value1, VoltVector2 value2)
        {
            value1.x *= value2.x;
            value1.y *= value2.y;
            return value1;
        }
        
        public static void Multiply(ref VoltVector2 value1, ref VoltVector2 value2, out VoltVector2 result)
        {
            result.x = value1.x * value2.x;
            result.y = value1.y * value2.y;
        }
        
        public static VoltVector2 Multiply(VoltVector2 value1, Fix64 scaleFactor)
        {
            value1.x *= scaleFactor;
            value1.y *= scaleFactor;
            return value1;
        }
        
        public static void Multiply(ref VoltVector2 value1, Fix64 scaleFactor, out VoltVector2 result)
        {
            result.x = value1.x * scaleFactor;
            result.y = value1.y * scaleFactor;
        }
        
        public static VoltVector2 Divide(VoltVector2 value1, VoltVector2 value2)
        {
            value1.x /= value2.x;
            value1.y /= value2.y;
            return value1;
        }
        
        public static void Divide(ref VoltVector2 value1, ref VoltVector2 value2, out VoltVector2 result)
        {
            result.x = value1.x / value2.x;
            result.y = value1.y / value2.y;
        }
        
        public static VoltVector2 Divide(VoltVector2 value1, Fix64 divider)
        {
            Fix64 num = Fix64.One / divider;
            value1.x *= num;
            value1.y *= num;
            return value1;
        }

        public static void Divide(ref VoltVector2 value1, Fix64 divider, out VoltVector2 result)
        {
            Fix64 num = Fix64.One / divider;
            result.x = value1.x * num;
            result.y = value1.y * num;
        }

        public static Fix64 Dot(VoltVector2 value1, VoltVector2 value2)
        {
            return value1.x * value2.x + value1.y * value2.y;
        }
        
        public static void Dot(ref VoltVector2 value1, ref VoltVector2 value2, out Fix64 result)
        {
            result = value1.x * value2.x + value1.y * value2.y;
        }

        public static VoltVector2 Negate(VoltVector2 value)
        {
            value.x = Fix64.Zero - value.x;
            value.y = Fix64.Zero - value.y;
            return value;
        }
        
        public static void Negate(ref VoltVector2 value, out VoltVector2 result)
        {
            result.x = Fix64.Zero - value.x;
            result.y = Fix64.Zero - value.y;
        }
        
        public void Normalize()
        {
            Fix64 num = Fix64.One / Fix64.Sqrt(x * x + y * y);
            x *= num;
            y *= num;
        }
        
        public static VoltVector2 Normalize(VoltVector2 value)
        {
            Fix64 num = Fix64.One / Fix64.Sqrt(value.x * value.x + value.y * value.y);
            value.x *= num;
            value.y *= num;
            return value;
        }
        
        public static void Normalize(ref VoltVector2 value, out VoltVector2 result)
        {
            Fix64 num = Fix64.One / Fix64.Sqrt(value.x * value.x + value.y * value.y);
            result.x = value.x * num;
            result.y = value.y * num;
        }

        public static VoltVector2 Reflect(VoltVector2 vector, VoltVector2 normal)
        {
            Fix64 num = ((Fix64)2f) * (vector.x * normal.x + vector.y * normal.y);
            VoltVector2 result = default(VoltVector2);
            result.x = vector.x - normal.x * num;
            result.y = vector.y - normal.y * num;
            return result;
        }
        
        public static void Reflect(ref VoltVector2 vector, ref VoltVector2 normal, out VoltVector2 result)
        {
            Fix64 num = ((Fix64)2) * (vector.x * normal.x + vector.y * normal.y);
            result.x = vector.x - normal.x * num;
            result.y = vector.y - normal.y * num;
        }

        public void Round()
        {
            x = Fix64.Round(x);
            y = Fix64.Round(y);
        }
        
        public static VoltVector2 Round(VoltVector2 value)
        {
            value.x = Fix64.Round(value.x);
            value.y = Fix64.Round(value.y);
            return value;
        }

        public static void Round(ref VoltVector2 value, out VoltVector2 result)
        {
            result.x = Fix64.Round(value.x);
            result.y = Fix64.Round(value.y);
        }

        public void Ceiling()
        {
            x = Fix64.Ceiling(x);
            y = Fix64.Ceiling(y);
        }

        public static VoltVector2 Ceiling(VoltVector2 value)
        {
            value.x = Fix64.Ceiling(value.x);
            value.y = Fix64.Ceiling(value.y);
            return value;
        }

        public static void Ceiling(ref VoltVector2 value, out VoltVector2 result)
        {
            result.x = Fix64.Ceiling(value.x);
            result.y = Fix64.Ceiling(value.y);
        }

        public void Floor()
        {
            x = Fix64.Floor(x);
            y = Fix64.Floor(y);
        }
        
        public static VoltVector2 Floor(VoltVector2 value)
        {
            value.x = Fix64.Floor(value.x);
            value.y = Fix64.Floor(value.y);
            return value;
        }
        
        public static void Floor(ref VoltVector2 value, out VoltVector2 result)
        {
            result.x = Fix64.Floor(value.x);
            result.y = Fix64.Floor(value.y);
        }

        public static VoltVector2 Clamp(VoltVector2 value1, VoltVector2 min, VoltVector2 max)
        {
            return new VoltVector2(VoltMath.Clamp(value1.x, min.x, max.x), VoltMath.Clamp(value1.y, min.y, max.y));
        }
        
        public static void Clamp(ref VoltVector2 value1, ref VoltVector2 min, ref VoltVector2 max, out VoltVector2 result)
        {
            result.x = VoltMath.Clamp(value1.x, min.x, max.x);
            result.y = VoltMath.Clamp(value1.y, min.y, max.y);
        }

        public static Fix64 Distance(VoltVector2 value1, VoltVector2 value2)
        {
            Fix64 num = value1.x - value2.x;
            Fix64 num2 = value1.y - value2.y;
            return VoltMath.Sqrt(num * num + num2 * num2);
        }
        
        public static void Distance(ref VoltVector2 value1, ref VoltVector2 value2, out Fix64 result)
        {
            Fix64 num = value1.x - value2.x;
            Fix64 num2 = value1.y - value2.y;
            result = VoltMath.Sqrt(num * num + num2 * num2);
        }
        
        public static Fix64 DistanceSquared(VoltVector2 value1, VoltVector2 value2)
        {
            Fix64 num = value1.x - value2.x;
            Fix64 num2 = value1.y - value2.y;
            return num * num + num2 * num2;
        }
        
        public static void DistanceSquared(ref VoltVector2 value1, ref VoltVector2 value2, out Fix64 result)
        {
            Fix64 num = value1.x - value2.x;
            Fix64 num2 = value1.y - value2.y;
            result = num * num + num2 * num2;
        }

        public override bool Equals(object obj) => Equals(obj as VoltVector2?);

        public override string ToString()
        {
            return "{X:" + x + " Y:" + y + "}";
        }

        public bool Equals(VoltVector2 other)
        {
            return this == other;
        }

        public override int GetHashCode()
        {
            return (x.GetHashCode() * 397) ^ y.GetHashCode();
        }

        public Fix64 Length()
        {
            return Fix64.Sqrt(x * x + y * y);
        }

        public Fix64 LengthSquared()
        {
            return x * x + y * y;
        }
        
        public static VoltVector2 Lerp(VoltVector2 value1, VoltVector2 value2, Fix64 amount)
        {
            return new VoltVector2(VoltMath.Lerp(value1.x, value2.x, amount), VoltMath.Lerp(value1.y, value2.y, amount));
        }
        
        public static void Lerp(ref VoltVector2 value1, ref VoltVector2 value2, Fix64 amount, out VoltVector2 result)
        {
            result.x = VoltMath.Lerp(value1.x, value2.x, amount);
            result.y = VoltMath.Lerp(value1.y, value2.y, amount);
        }

        public static VoltVector2 LerpPrecise(VoltVector2 value1, VoltVector2 value2, Fix64 amount)
        {
            return new VoltVector2(VoltMath.LerpPrecise(value1.x, value2.x, amount), VoltMath.LerpPrecise(value1.y, value2.y, amount));
        }
        
        public static void LerpPrecise(ref VoltVector2 value1, ref VoltVector2 value2, Fix64 amount, out VoltVector2 result)
        {
            result.x = VoltMath.LerpPrecise(value1.x, value2.x, amount);
            result.y = VoltMath.LerpPrecise(value1.y, value2.y, amount);
        }

        public static VoltVector2 Rotate(VoltVector2 value, Fix64 radians)
        {
            Fix64 num = Fix64.Cos(radians);
            Fix64 num2 = Fix64.Sin(radians);
            return new VoltVector2(value.x * num - value.y * num2, value.x * num2 + value.y * num);
        }

        public void Rotate(Fix64 radians)
        {
            Fix64 num = Fix64.Cos(radians);
            Fix64 num2 = Fix64.Sin(radians);
            Fix64 _x = x;
            x = x * num - y * num2;
            y = _x * num2 + y * num;
        }
        
        public static VoltVector2 RotateAround(VoltVector2 value, VoltVector2 origin, Fix64 radians)
        {
            return Rotate(value - origin, radians) + origin;
        }
        
        public void RotateAround(VoltVector2 origin, Fix64 radians)
        {
            this -= origin;
            Rotate(radians);
            this += origin;
        }
        
        public static VoltVector2 Max(VoltVector2 value1, VoltVector2 value2)
        {
            return new VoltVector2(
                (value1.x > value2.x) ? value1.x : value2.x,
                (value1.y > value2.y) ? value1.y : value2.y
            );
        }
        
        public static void Max(ref VoltVector2 value1, ref VoltVector2 value2, out VoltVector2 result)
        {
            result.x = (value1.x > value2.x) ? value1.x : value2.x;
            result.y = (value1.y > value2.y) ? value1.y : value2.y;
        }
        
        public static VoltVector2 Min(VoltVector2 value1, VoltVector2 value2)
        {
            return new VoltVector2(
                (value1.x < value2.x) ? value1.x : value2.x,
                (value1.y < value2.y) ? value1.y : value2.y
            );
        }
        
        public static void Min(ref VoltVector2 value1, ref VoltVector2 value2, out VoltVector2 result)
        {
            result.x = (value1.x < value2.x) ? value1.x : value2.x;
            result.y = (value1.y < value2.y) ? value1.y : value2.y;
        }
    }
}