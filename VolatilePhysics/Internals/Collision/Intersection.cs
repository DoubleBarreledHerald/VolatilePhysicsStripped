using System;
using System.Collections.Generic;
using FixMath.NET;

namespace Volatile
{
  public class Intersection 
    : IVoltPoolable<Intersection>
  {
    #region Interface
    IVoltPool<Intersection> IVoltPoolable<Intersection>.Pool { get; set; }
    void IVoltPoolable<Intersection>.Reset() { this.Reset(); }
    #endregion

    public VoltShape voltShapeA { get; private set; }
    public VoltShape voltShapeB { get; private set; }
    public List<VoltVector2> worldPoints { get; private set; }
    public VoltVector2 normal { get; private set; }
    public Fix64 penetration { get; private set; }

    public Intersection()
    {
      this.Reset();
    }

    internal Intersection Assign(
      VoltShape self,
      VoltShape other,
      List<VoltVector2> worldPoints,
      VoltVector2 normal,
      Fix64 penetration)
    {
      this.Reset();

      this.voltShapeA = self;
      this.voltShapeB = other;
      this.worldPoints = worldPoints;
      this.normal = normal;
      this.penetration = penetration;

      return this;
    }

    #region Internals
    private void Reset()
    {
      this.worldPoints = new List<VoltVector2>();
      this.normal = VoltVector2.zero;
      this.penetration = Fix64.Zero;

      this.voltShapeA = null;
      this.voltShapeB = null;
    }
    #endregion
  }
}