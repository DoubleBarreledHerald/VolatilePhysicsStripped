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

using FixMath.NET;
using System;
using System.Collections.Generic;

#if UNITY
using UnityEngine;
#endif

namespace Volatile
{
  public enum VoltBodyType
  {
    Static,
    Dynamic,
    Invalid,
  }

  public delegate bool VoltBodyFilter(VoltBody body);
  public delegate bool VoltCollisionFilter(VoltBody bodyA, VoltBody bodyB);

  public class VoltBody
    : IVoltPoolable<VoltBody>
    , IIndexedValue
  {
    #region Interface
    IVoltPool<VoltBody> IVoltPoolable<VoltBody>.Pool { get; set; }
    void IVoltPoolable<VoltBody>.Reset() { this.Reset(); }
    int IIndexedValue.Index { get; set; }
    #endregion

    /// <summary>
    /// A predefined filter that disallows collisions between dynamic bodies.
    /// </summary>
    public static bool DisallowDynamic(VoltBody a, VoltBody b)
    {
      return
        (a != null) &&
        (b != null) &&
        (a.IsStatic || b.IsStatic);
    }

    public static bool Filter(VoltBody body, VoltBodyFilter filter)
    {
      return ((filter == null) || (filter.Invoke(body) == true));
    }

    /// <summary>
    /// Static objects are considered to have infinite mass and cannot move.
    /// Setting will reset all active forces on the body.
    /// </summary>
    public bool IsStatic
    {
      get {
        if (this.BodyType == VoltBodyType.Invalid)
          throw new InvalidOperationException();
        return this.BodyType == VoltBodyType.Static;
      }
      
      //Leaves the body in the incorrect Broadphase
      /* set
      {
        if (this.BodyType == VoltBodyType.Invalid)
          throw new InvalidOperationException();
        if (value)
        {
          //static
          SetStatic();
        }
        else
        {
          //dynamic
          ComputeDynamics();
        }
      } */
    }

    public bool isAwake { get; set; } = true;

    public bool CanSleep { get; set; } = true;

    public Fix64 SleepEpsilon = (Fix64)20;

    public Fix64 SleepTimerSeconds = (Fix64)0.25;

    public Fix64 IdleTime { get; set; }

    internal bool willWakeUp = false;

    public bool SleepGravityScaling = true;

    //REF: https://github.com/schteppe/p2.js/blob/2beb2750f42d29014e289cb803b7269d5b0edaad/src/world/World.js#L920
    private bool CheckSleepy(){
      var speedSquared = this.InternalLinearVelocity.LengthSquared() + Fix64.Pow(Fix64.Abs(this.AngularVelocity), (Fix64)2);
      var speedLimitSquared = Fix64.Pow(this.SleepEpsilon, (Fix64)2);

      // Add to idle time
      if(speedSquared >= speedLimitSquared){
          this.IdleTime = Fix64.Zero;
          isAwake = true;
          return false;
      }

      if (this.IdleTime > this.SleepTimerSeconds){
          return true;
      }
      
      this.IdleTime += World.DeltaTime;
      return false;
    }

    public void CheckWakeUp()
    {
      if (!this.CanSleep || this.willWakeUp || this.isAwake) return;

      if (CheckSleepy()) return;

      this.willWakeUp = true;
    }

    public void CallSleep()
    {
      if (!this.CanSleep) return;

      if (!this.isAwake) return;

      if (!CheckSleepy()) return;

      this.isAwake = false;
      ClearForces();
      ClearVelocities();
    }

    internal void CallWakeUp()
    {
      if (!this.CanSleep) return;

      if (this.isAwake) {
        this.willWakeUp = false;
        return;
      }

      if (!this.willWakeUp) return;

      WakeUp();
    }

    public void WakeUp()
    {
      this.willWakeUp = false;
      this.IdleTime = (Fix64)0;
      this.isAwake = true;
    }

    /// <summary>
    /// Awakens nearby bodies if they pass this body's collision filter.
    /// </summary>
    public void WakeUpNeighbors()
    {
      VoltBuffer<VoltBody> neighbors = World.QueryOverlapBody(AABB, CheckCollisionFilter);

      foreach (VoltBody neighbor in neighbors)
      {
          neighbor.WakeUp();
      }
    }

    public bool IsEnabled { get; set; } = true;

    public bool IsTrigger { get; set; } = false;

    public bool RaycastMove { get; set; } = false;
    
    public bool IgnoreRaycasts { get; set; } = false;

    public bool IgnoreTriggers { get; set; } = false;

    public bool IsInWorld { get { return this.World != null; } }

    public VoltVector2 Position {
      get { return InternalPosition * World.WorldScale; }
      set { InternalPosition = value * World.InvWorldScale; }
    }
    internal VoltVector2 InternalPosition { get; private set; }

    public VoltVector2 Facing { get; private set; }

    public VoltAABB AABB { get; private set; }

#if DEBUG
    internal bool IsInitialized { get; set; }
#endif

    /// <summary>
    /// For attaching arbitrary data to this body.
    /// </summary>
    public object UserData { get; set; }

    public VoltWorld World { get; private set; }
    public void SetID(int ID)
    {
      this.ID = ID;
      World.RequireDynamicSort = true;
    }
    public int ID { get; private set; }
    public VoltBodyType BodyType { get; private set; }
    public VoltCollisionFilter CollisionFilter { private get; set; }

    /// <summary>
    /// Current angle in radians.
    /// </summary>
    public Fix64 Angle { get; private set; }

    public bool IsFixedAngle { get; set; } = false;

    public bool IsFixedPosition { get; set; } = false;

    public bool IsFixed { get { return IsFixedAngle && IsFixedPosition; } }

    
    public VoltVector2 LinearVelocity {
      get { return InternalLinearVelocity * World.WorldScale; }
      set { InternalLinearVelocity = value * World.InvWorldScale; }
    }
    internal VoltVector2 InternalLinearVelocity { get; set; }
    public Fix64 AngularVelocity { get; set; }

    /// <summary>
    /// The local linear damping.
    /// </summary>
    public VoltVector2 LinearDamping { get; set; } = VoltVector2.one;
    /// <summary>
    /// The local angular damping.
    /// </summary>
    public Fix64 AngularDamping { get; set; } = (Fix64)1;

    public Fix64 BiasStrength = (Fix64)1;

    public VoltVector2 Force { get; private set; }
    public Fix64 Torque { get; private set; }
    
    /// <summary>
    /// Sets whether or not the body will be affected by the World's gravity. 
    /// </summary>
    public bool IsAffectedByWorldGravity { get; set; } = true;
    /// <summary>
    /// The local gravity of the body.
    /// </summary>
    public VoltVector2 Gravity { get; set; }

    public delegate void CollisionEventHandler(VoltBody bodyA, VoltBody bodyB);
    public event CollisionEventHandler OnCollision;

    public delegate void TriggerEventHandler(VoltBody bodyA, VoltBody bodyB);
    public event TriggerEventHandler OnTrigger;

    HashSet<VoltBody> collisions = new HashSet<VoltBody>();
    HashSet<VoltBody> triggers = new HashSet<VoltBody>();

    internal void AddCollision(VoltBody collision)
    {
      collisions.Add(collision);
    }

    internal void AddTrigger(VoltBody trigger)
    {
      triggers.Add(trigger);
    }

    internal void HandleCollisions()
    {
      foreach (VoltBody collision in collisions)
      {
        OnCollision?.Invoke(this, collision);
      }
      collisions.Clear();

      foreach (VoltBody trigger in triggers)
      {
        OnTrigger?.Invoke(this, trigger);
      }
      triggers.Clear();
    }

    public Delegate[] GetCollisionDelegates()
    {
      return OnCollision.GetInvocationList();
    }

    public void ClearOnCollisionEvent()
    {
      OnCollision = null;
    }

    public void ClearShapesOnCollisionEvent()
    {
      foreach (VoltShape shape in shapes)
      {
        shape.ClearOnCollisionEvent();
      }
    }

    public Delegate[] GetTriggerDelegates()
    {
      return OnTrigger.GetInvocationList();
    }

    public void ClearOnTriggerEvent()
    {
      OnTrigger = null;
    }

    /// <summary>
    /// The collective mass of each of the shapes that make up the body.
    /// If set, overrides.
    /// </summary>
    public Fix64 Mass
    {
      get
      {
        if (BodyType == VoltBodyType.Static) return Fix64.Zero;
        if (IsFixedPosition) return Fix64.MaxValue;
        //mass is overridden.
        if (_mass != null)
          return _mass.GetValueOrDefault();
        //mass is calculated.
        return collMass;
      }
      set
      {
        if (BodyType == VoltBodyType.Static) return;

        if (value == Fix64.Zero)
        {
          _mass = null;
        } else {
          _mass = value;
        }
        //recalculate
        ComputeDynamics();
      }
    }
    /// <summary>
    /// Overrides the Mass value;
    /// </summary>
    private Fix64? _mass { get; set; } = null;
    /// <summary>
    /// The collective mass of each shape that makes up the body.
    /// </summary>
    private Fix64 collMass { get; set; }
    public Fix64 Inertia { get; set; }
    public Fix64 InvMass { get; private set; }
    public Fix64 InvInertia { get; private set; }

    public VoltVector2 BiasVelocity { get; private set; }
    public Fix64 BiasRotation { get; private set; }

    // Used for broadphase structures
    public int ProxyId { get; internal set; }

    public VoltShape[] shapes { get; private set; }
    internal int shapeCount;
    /// <summary>
    /// The collective area of each shape that makes up the body.
    /// </summary>
    public Fix64 Area { get; private set; }

    #region Manipulation
    public void AddTorque(Fix64 torque)
    {
      if (IsEnabled == false) return;
      this.AngularVelocity -= torque * World.DeltaTime * InvInertia;
      CheckWakeUp();
    }

    public void AddForce(VoltVector2 force)
    {
      if (IsEnabled == false) return;
      this.LinearVelocity += force * World.DeltaTime * InvMass;
      CheckWakeUp();
    }

    public void AddForce(VoltVector2 force, VoltVector2 point)
    {
      if (IsEnabled == false) return;
      this.LinearVelocity += force * World.DeltaTime * InvMass;
      this.AngularVelocity -= VoltMath.Cross(this.InternalPosition - point, force) * World.DeltaTime * InvMass;
      CheckWakeUp();
    }

    public void Set(VoltVector2 position, Fix64 radians)
    {
      this.InternalPosition = position;
      this.Angle = radians;
      this.Facing = VoltMath.Polar(radians);
      this.OnPositionUpdated();
      CheckWakeUp();
    }

    //TODO REMOVE
    public void SetForce(VoltVector2 force, Fix64 torque, VoltVector2 biasVelocity, Fix64 biasRotation)
    {
      CheckWakeUp();
    }

    #endregion

    #region Tests
    /// <summary>
    /// Checks if an AABB overlaps with our AABB.
    /// </summary>
    internal bool QueryAABBOnly(
      VoltAABB worldBounds)
    {
      if (IsEnabled == false) return false;
      return AABB.Intersect(worldBounds);
    }

    internal bool QueryAABB(
      VoltAABB worldBounds)
    {
      if (!QueryAABBOnly(worldBounds)) return false;
      
      // Actual query on shapes done in body space
      for (int i = 0; i < this.shapeCount; i++)
        if (this.shapes[i].QueryAABB(worldBounds))
          return true;
      return false;
    }

    /// <summary>
    /// Checks if a point is contained in this body. 
    /// Begins with AABB checks unless bypassed.
    /// </summary>
    internal bool QueryPoint(
      VoltVector2 point,
      bool bypassAABB = false)
    {
      if (IsEnabled == false) return false;
      // AABB check done in world space (because it keeps changing)
      if (bypassAABB == false)
        if (AABB.QueryPoint(point) == false)
          return false;

      // Actual query on shapes done in body space
      VoltVector2 bodySpacePoint = WorldToBodyPoint(point);
      for (int i = 0; i < this.shapeCount; i++)
        if (this.shapes[i].QueryPoint(bodySpacePoint))
          return true;
      return false;
    }

    /// <summary>
    /// Checks if a circle overlaps with this body. 
    /// Begins with AABB checks.
    /// </summary>
    internal bool QueryCircle(
      VoltVector2 origin,
      Fix64 radius,
      bool bypassAABB = false)
    {
      if (IsEnabled == false) return false;
      // AABB check done in world space (because it keeps changing)
      if (bypassAABB == false)
        if (AABB.QueryCircleApprox(origin, radius) == false)
          return false;

      // Actual query on shapes done in body space
      VoltVector2 bodySpaceOrigin = WorldToBodyPoint(origin);
      for (int i = 0; i < this.shapeCount; i++)
        if (this.shapes[i].QueryCircle(bodySpaceOrigin, origin, radius))
          return true;
      return false;
    }

    /// <summary>
    /// Performs a ray cast check on this body. 
    /// Begins with AABB checks.
    /// </summary>
    internal bool RayCast(
      ref VoltRayCast ray,
      ref VoltRayResult result,
      bool bypassAABB = false)
    {
      if (IsEnabled == false) return false;

      if (IgnoreRaycasts == true) return false;
      
      // AABB check done in world space (because it keeps changing)
      if (bypassAABB == false)
        if (AABB.RayCast(ref ray) == false)
          return false;

      // Actual tests on shapes done in body space
      VoltRayCast bodySpaceRay = WorldToBodyRay(ref ray);
      for (int i = 0; i < this.shapeCount; i++)
        if (this.shapes[i].RayCast(ref bodySpaceRay, ref result))
          if (result.IsContained)
            return true;

      // We need to convert the results back to world space to be any use
      // (Doesn't matter if we were contained since there will be no normal)
      if (result.Body == this)
        result.normal = BodyToWorldDirection(result.normal);
      return result.IsValid;
    }

    /// <summary>
    /// Performs a circle cast check on this body. 
    /// Begins with AABB checks.
    /// </summary>
    internal bool CircleCast(
      ref VoltRayCast ray,
      Fix64 radius,
      ref VoltRayResult result,
      bool bypassAABB = false)
    {
      if (IsEnabled == false) return false;
      // AABB check done in world space (because it keeps changing)
      if (bypassAABB == false)
        if (AABB.CircleCastApprox(ref ray, radius) == false)
          return false;

      // Actual tests on shapes done in body space
      VoltRayCast bodySpaceRay = WorldToBodyRay(ref ray);
      for (int i = 0; i < this.shapeCount; i++)
        if (this.shapes[i].CircleCast(ref bodySpaceRay, radius, ref result))
          if (result.IsContained)
            return true;

      // We need to convert the results back to world space to be any use
      // (Doesn't matter if we were contained since there will be no normal)
      if (result.Body == this)
        result.normal = BodyToWorldDirection(result.normal);
      return result.IsValid;
    }
    #endregion

    public VoltBody()
    {
      this.Reset();
      this.ProxyId = -1;
    }

    internal void InitializeDynamic(
      VoltVector2 position,
      Fix64 radians,
      VoltShape[] shapesToAdd)
    {
      this.Initialize(position, radians, shapesToAdd);
      this.OnPositionUpdated();
      this.ComputeDynamics();
    }

    internal void InitializeStatic(
      VoltVector2 position,
      Fix64 radians,
      VoltShape[] shapesToAdd)
    {
      this.Initialize(position, radians, shapesToAdd);
      this.OnPositionUpdated();
      this.SetStatic();
    }

    private void Initialize(
      VoltVector2 position,
      Fix64 radians,
      VoltShape[] shapesToAdd)
    {
      this.InternalPosition = position;
      this.Angle = radians;
      this.Facing = VoltMath.Polar(radians);

#if DEBUG
      for (int i = 0; i < shapesToAdd.Length; i++)
        VoltDebug.Assert(shapesToAdd[i].IsInitialized);
#endif

      if ((this.shapes == null) || (this.shapes.Length < shapesToAdd.Length))
        this.shapes = new VoltShape[shapesToAdd.Length];
      Array.Copy(shapesToAdd, this.shapes, shapesToAdd.Length);
      this.shapeCount = shapesToAdd.Length;
      this.Area = Fix64.Zero;
      for (int i = 0; i < this.shapeCount; i++)
      {
        VoltShape shape = this.shapes[i];
        shape.AssignBody(this, VoltWorld.AssignShape(shape));
        this.Area += shape.Area;
      }

#if DEBUG
      this.IsInitialized = true;
#endif
    }

    internal void AssignWorld(VoltWorld world, int id)
    {
      this.World = world;
      this.ID = id;
    }

    internal void FreeShapes()
    {
      if (this.World != null)
      {
        for (int i = 0; i < this.shapeCount; i++)
          this.World.FreeShape(this.shapes[i]);
        for (int i = 0; i < this.shapes.Length; i++)
          this.shapes[i] = null;
      }
      this.shapeCount = 0;
    }

    /// <summary>
    /// Used for saving the body as part of another structure. The body
    /// will retain all geometry data and associated metrics, but its
    /// position, velocity, forces, and all related history will be cleared.
    /// </summary>
    internal void PartialReset()
    {
      InternalPosition = VoltVector2.zero;
      Facing = VoltVector2.zero;
      AABB = new VoltAABB();

      this.InternalLinearVelocity = VoltVector2.zero;
      this.AngularVelocity = Fix64.Zero;

      this.BiasVelocity = VoltVector2.zero;
      this.BiasRotation = Fix64.Zero;
    }

    /// <summary>
    /// Full reset. Clears out all data for pooling. Call FreeShapes() first.
    /// </summary>
    private void Reset()
    {
      VoltDebug.Assert(this.shapeCount == 0);

#if DEBUG
      this.IsInitialized = false;
#endif

      this.UserData = null;
      this.World = null;
      this.BodyType = VoltBodyType.Invalid;
      this.CollisionFilter = null;

      this.Angle = Fix64.Zero;
      this.InternalLinearVelocity = VoltVector2.zero;
      this.AngularVelocity = Fix64.Zero;

      this._mass = null;
      this.collMass = Fix64.Zero;
      this.Inertia = Fix64.Zero;
      this.InvMass = Fix64.Zero;
      this.InvInertia = Fix64.Zero;

      this.BiasVelocity = VoltVector2.zero;
      this.BiasRotation = Fix64.Zero;

      InternalPosition = VoltVector2.zero;
      Facing = VoltVector2.zero;
      AABB = new VoltAABB();
    }

    #region Collision
    internal bool CanCollide(VoltBody other)
    {
      if (IsEnabled == false) return false;
      // Ignore self 
      if (this == other)
        return false;

      //Trigger bypass
      if (this.IsTrigger || other.IsTrigger)
      {
        if (other.IsTrigger && IgnoreTriggers) return false;
        return CheckCollisionFilter(other);
      }

      //Ignore static-fixed-asleep collisions
      if ((this.IsStatic || this.IsFixed || !this.isAwake) && (other.IsStatic || other.IsFixed || !other.isAwake))
        return false;


      return CheckCollisionFilter(other);
    }

    public bool CheckCollisionFilter(VoltBody other)
    {
      if (this.CollisionFilter != null)
        return this.CollisionFilter.Invoke(this, other);
      return true;
    }

    internal bool CanCollideRay(VoltBody other)
    {
      if (IsEnabled == false) return false;
      // Ignore self 
      if (this == other)
        return false;

      if (other.IsTrigger)
        return false;
      
      return CanCollide(other);
    }

    
    internal void ApplyImpulse(VoltVector2 impulse, VoltVector2 worldPoint) {
      VoltVector2 r = worldPoint - (this.InternalPosition);

      this.InternalLinearVelocity = this.InternalLinearVelocity + (impulse * InvMass);
      this.AngularVelocity -= this.InvInertia * VoltMath.Cross(impulse, r);
    }

    internal void ApplyBias(VoltVector2 j, VoltVector2 r)
    {
      if (IsEnabled == false) return;
      if (!IsFixedPosition)
        this.BiasVelocity += j * this.InvMass * BiasStrength;
      if (!IsFixedAngle)
        this.BiasRotation -= this.InvInertia * VoltMath.Cross(j, r) * BiasStrength;
    }
    #endregion

    #region Transformation Shortcuts
    internal VoltVector2 WorldToBodyPointCurrent(VoltVector2 vector)
    {
      return WorldToBodyPoint(vector);
    }

    internal VoltVector2 BodyToWorldPointCurrent(VoltVector2 vector)
    {
      return BodyToWorldPoint(vector);
    }

    internal Axis BodyToWorldAxisCurrent(Axis axis)
    {
      return BodyToWorldAxis(axis);
    }
    #endregion

    #region Helpers
    /// <summary>
    /// Applies the current position and angle to shapes and the AABB.
    /// </summary>
    private void OnPositionUpdated()
    {
      for (int i = 0; i < this.shapeCount; i++)
        this.shapes[i].OnBodyPositionUpdated();
      this.UpdateAABB();
    }

    /// <summary>
    /// Builds the AABB by combining all the shape AABBs.
    /// </summary>
    private void UpdateAABB()
    {
      Fix64 top = Fix64.MinValue;
      Fix64 right = Fix64.MinValue;
      Fix64 bottom = Fix64.MaxValue;
      Fix64 left = Fix64.MaxValue;

      for (int i = 0; i < this.shapeCount; i++)
      {
        VoltAABB aabb = this.shapes[i].AABB;

        top = VoltMath.Max(top, aabb.Top);
        right = VoltMath.Max(right, aabb.Right);
        bottom = VoltMath.Min(bottom, aabb.Bottom);
        left = VoltMath.Min(left, aabb.Left);
      }

      this.AABB = new VoltAABB(top, bottom, left, right);
    }

    internal void ApplyDamping()
    {
      // Apply damping
      if (!IsFixedPosition)
      {
        Fix64 xVelocity = this.InternalLinearVelocity.x * this.LinearDamping.x * this.World.LinearDamping.x;
        Fix64 yVelocity = this.InternalLinearVelocity.y * this.LinearDamping.y * this.World.LinearDamping.y;
        this.InternalLinearVelocity = new VoltVector2(xVelocity, yVelocity);
      }
      if (!IsFixedAngle)
        this.AngularVelocity *= this.World.AngularDamping * this.AngularDamping;
    }

    internal void ApplyGravity(Fix64 mult)
    {
      if (IsEnabled == false) return;
      if (!this.isAwake) return;

      Fix64 SleepDelta = Fix64.One;

      if (SleepGravityScaling)
        SleepDelta = -(VoltMath.Max(IdleTime / SleepTimerSeconds, Fix64.Zero) - Fix64.One);

      //Apply global gravity
      if (this.IsAffectedByWorldGravity)
        this.InternalLinearVelocity += this.World.Gravity * this.World.DeltaTime * SleepDelta * mult;

      //Apply personal gravity
      this.InternalLinearVelocity += Gravity * this.World.DeltaTime * SleepDelta * mult;
    }

    internal void IntegrateVelocity()
    {
      if (IsEnabled == false) return;

      if (!isAwake) return;
      
      IntegratePosition();
      IntegrateRotation();

      OnPositionUpdated();
    }

    private void IntegratePosition()
    {
      if (IsFixedPosition)
        return;

      if (!isAwake) return;

      VoltVector2 targetPosition =
        this.InternalPosition + this.World.DeltaTime * this.InternalLinearVelocity;

      if (RaycastMove)
      {
        IntegrateRaycastMove(ref targetPosition);
      }

      this.InternalPosition = targetPosition;
    }

    private void IntegrateRotation()
    {
      if (IsFixedAngle)
        return;

      if (!isAwake) return;

      this.Angle +=
        this.World.DeltaTime * this.AngularVelocity;
      this.Facing = VoltMath.Polar(this.Angle);
    }

    internal void IntegrateBias()
    {
      if (!isAwake) return;

      //Position
      if (!IsFixedPosition)
        this.InternalPosition += this.BiasVelocity;

      //Rotation
      if (!IsFixedAngle)
        this.Angle += this.BiasRotation;
      this.Facing = VoltMath.Polar(this.Angle);
      
      this.BiasVelocity = VoltVector2.zero;
      this.BiasRotation = Fix64.Zero;
      OnPositionUpdated();
    }

    public VoltVector2 rayMoveOrigin;
    public VoltVector2 rayMoveTarget;

    private void IntegrateRaycastMove(ref VoltVector2 targetPosition)
    {
      if ((InternalPosition - targetPosition).Length() == Fix64.Zero) 
        return;

      if (World.QueryPoint(InternalPosition, CanCollideRay).Count > 0) 
        return;

      //Raycast from current position to target position
      var ray = new VoltRayCast(InternalPosition, targetPosition);
      rayMoveOrigin = InternalPosition;
      rayMoveTarget = targetPosition;
      var result = new VoltRayResult();

      //on collide
      if (World.RayCast(ref ray, ref result, CanCollideRay))
      {
        //move to collision point
        targetPosition = result.ComputePoint(ref ray);
        return;
      }
    }

    public void ClearForces()
    {
      this.BiasVelocity = VoltVector2.zero;
      this.BiasRotation = Fix64.Zero;
    }

    public void ClearVelocities()
    {
      this.InternalLinearVelocity = VoltVector2.zero;
      this.AngularVelocity = Fix64.Zero;
    }

    private void ComputeDynamics()
    {
      this.collMass = Fix64.Zero;
      this.Inertia = Fix64.Zero;

      for (int i = 0; i < this.shapeCount; i++)
      {
        VoltShape shape = this.shapes[i];
        Fix64 curMass;
        //Mass override
        if (this._mass != null)
        {
          //Divide the body's override mass to each shape based on their area.
          curMass = _mass.GetValueOrDefault() * (shape.Area / this.Area);
        } else {
          if (shape.Density == Fix64.Zero)
            continue;
          curMass = shape.Mass;
        }

        Fix64 curInertia = shape.Inertia;

        this.collMass += curMass;
        this.Inertia += curMass * curInertia;
      }

      if (this.collMass < VoltConfig.MINIMUM_DYNAMIC_MASS)
      {
        throw new InvalidOperationException("Mass of dynamic too small");
      }

      this.InvMass = Fix64.One / this.Mass;
      this.InvInertia = Fix64.One / this.Inertia;

      this.BodyType = VoltBodyType.Dynamic;
    }

    private void SetStatic()
    {
      this.collMass = Fix64.Zero;
      //this._mass = Fix64.Zero;
      this.Inertia = Fix64.Zero;
      this.InvMass = Fix64.Zero;
      this.InvInertia = Fix64.Zero;

      this.BodyType = VoltBodyType.Static;
    }
    #endregion

#region World-Space to Body-Space Transformations
    internal VoltVector2 WorldToBodyPoint(VoltVector2 vector)
    {
      return VoltMath.WorldToBodyPoint(this.InternalPosition, this.Facing, vector);
    }

    internal VoltVector2 WorldToBodyDirection(VoltVector2 vector)
    {
      return VoltMath.WorldToBodyDirection(this.Facing, vector);
    }

    internal VoltRayCast WorldToBodyRay(ref VoltRayCast rayCast)
    {
      return new VoltRayCast(
        this.WorldToBodyPoint(rayCast.origin),
        this.WorldToBodyDirection(rayCast.direction),
        rayCast.distance);
    }
    #endregion

    #region Body-Space to World-Space Transformations
    internal VoltVector2 BodyToWorldPoint(VoltVector2 vector)
    {
      return VoltMath.BodyToWorldPoint(this.InternalPosition, this.Facing, vector);
    }

    internal VoltVector2 BodyToWorldDirection(VoltVector2 vector)
    {
      return VoltMath.BodyToWorldDirection(this.Facing, vector);
    }

    internal Axis BodyToWorldAxis(Axis axis)
    {
      VoltVector2 normal = axis.Normal.Rotate(this.Facing);
      Fix64 width = VoltVector2.Dot(normal, this.InternalPosition) + axis.Width;
      return new Axis(normal, width);
    }
    #endregion

    #region Debug
#if UNITY && DEBUG
    public void GizmoDraw(
      Color edgeColor,
      Color normalColor,
      Color bodyOriginColor,
      Color shapeOriginColor,
      Color bodyAabbColor,
      Color shapeAabbColor,
      Fix64 normalLength)
    {
      Color current = Gizmos.color;

      // Draw origin
      Gizmos.color = bodyOriginColor;
      Gizmos.DrawWireSphere(this.Position, 0.1f);

      // Draw facing
      Gizmos.color = normalColor;
      Gizmos.DrawLine(
        this.Position,
        this.Position + this.Facing * normalLength);

      this.AABB.GizmoDraw(bodyAabbColor);

      for (int i = 0; i < this.shapeCount; i++)
        this.shapes[i].GizmoDraw(
          edgeColor,
          normalColor,
          shapeOriginColor,
          shapeAabbColor,
          normalLength);

      Gizmos.color = current;
    }
#endif
    #endregion
  }
}