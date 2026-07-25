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

#if UNITY
using UnityEngine;
#endif

using System;
using FixMath.NET;

namespace Volatile
{
  internal sealed class Contact 
    : IVoltPoolable<Contact>
  {
    #region Interface
    IVoltPool<Contact> IVoltPoolable<Contact>.Pool { get; set; }
    void IVoltPoolable<Contact>.Reset() { this.Reset(); }
    #endregion

    #region Static Methods
    private static Fix64 BiasDist(Fix64 dist)
    {
      return VoltConfig.ResolveRate * VoltMath.Min(Fix64.Zero, dist + VoltConfig.ResolveSlop);
    }
    #endregion

    private VoltVector2 worldPoint;
    private VoltVector2 normal;
    private Fix64 penetration;

    private VoltVector2 localA;
    private VoltVector2 localB;
    private VoltVector2 worldA;
    private VoltVector2 worldB;
    private VoltVector2 rA;
    private VoltVector2 rB;
    private VoltVector2 tangent;
    private Fix64 relativeVelocity;

    private Fix64 accumulatedNormalLambda;
    private Fix64 accumulatedFrictionLambda;

    private Fix64 invMassA;
    private Fix64 invMassB;
    private Fix64 invIA;
    private Fix64 invIB;


    public Contact()
    {
      this.Reset();
    }

    internal Contact Assign(
      VoltVector2 worldPoint,
      VoltVector2 normal,
      Fix64 penetration)
    {
      this.Reset();

      this.worldPoint = worldPoint;
      this.normal = normal;
      this.penetration = penetration;

      return this;
    }

    internal void PreStep(Manifold manifold)
    {
      VoltBody bodyA = manifold.ShapeA.Body;
      VoltBody bodyB = manifold.ShapeB.Body;

      if (bodyA.IsTrigger || bodyB.IsTrigger || manifold.ShapeA.IsTrigger || manifold.ShapeB.IsTrigger) return;

      localA = worldPoint - bodyA.InternalPosition; localA.Rotate(-bodyA.Angle);
      localB = worldPoint - bodyB.InternalPosition; localB.Rotate(-bodyB.Angle);
      worldA = localA; worldA.Rotate(bodyA.Angle); worldA += bodyA.InternalPosition;
      worldB = localB; worldB.Rotate(bodyB.Angle); worldB += bodyA.InternalPosition;
      rA = worldA - bodyA.InternalPosition;
      rB = worldB - bodyB.InternalPosition;
      tangent = new VoltVector2(normal.y, -normal.x);

      invMassA = bodyA.InvMass;
      invMassB = bodyB.InvMass;
      invIA = bodyA.InvInertia;
      invIB = bodyB.InvInertia;

		  // Store relative velocity BEFORE warm starting for restitution
      VoltVector2 velA = bodyA.InternalLinearVelocity + VoltMath.CrossSV(rA, bodyA.AngularVelocity);
      VoltVector2 velB = bodyB.InternalLinearVelocity + VoltMath.CrossSV(rB, bodyB.AngularVelocity);
      VoltVector2 relVel = velB - velA;
      relativeVelocity = VoltVector2.Dot(normal, relVel);
    }

    internal void Solve(Manifold manifold)
    {
      VoltBody bodyA = manifold.ShapeA.Body;
      VoltBody bodyB = manifold.ShapeB.Body;

      SolveContact(manifold);

      SolveFriction(manifold);

      bodyA.CheckWakeUp();
      bodyB.CheckWakeUp();
    }

    internal void SolveContact(Manifold manifold)
    {
      VoltBody bodyA = manifold.ShapeA.Body;
      VoltBody bodyB = manifold.ShapeB.Body;

      //Contact
      VoltVector2 velA = bodyA.InternalLinearVelocity + VoltMath.CrossSV(rA, bodyA.AngularVelocity);
      VoltVector2 velB = bodyB.InternalLinearVelocity + VoltMath.CrossSV(rB, bodyB.AngularVelocity);
      VoltVector2 relVel = velB - velA;
      Fix64 Cdot = VoltVector2.Dot(normal, relVel);

      Fix64 rnA = VoltMath.Cross(rA, normal);
      Fix64 rnB = VoltMath.Cross(rB, normal);
      Fix64 effectiveMass =
        this.invMassA + this.invMassB + rnA * rnA * this.invIA + rnB * rnB * this.invIB;

      if (effectiveMass < VoltConfig.MINIMUM_DYNAMIC_MASS) return;

      Fix64 seperation = VoltMath.Min((Fix64)0, -Fix64.Abs(penetration) + VoltConfig.ResolveSlop);
      Fix64 velocityBias = (VoltConfig.ResolveRate / bodyA.World.DeltaTime) * seperation;

      Fix64 lambda = -(Cdot + velocityBias) / effectiveMass;
      
      // Clamp the accumulated impulse
      Fix64 oldAccum = this.accumulatedNormalLambda;
      this.accumulatedNormalLambda = VoltMath.Max(oldAccum + lambda, Fix64.Zero);
      lambda = this.accumulatedNormalLambda - oldAccum;

      if (lambda == (Fix64)0) return;

      VoltVector2 impulse = normal * lambda;
      bodyA.ApplyImpulse(-impulse, worldA);
      bodyB.ApplyImpulse(impulse, worldB);
    }

    internal void SolveFriction(Manifold manifold)
    {
      if (manifold.Friction <= Fix64.Zero) return;

      Fix64 rtA = VoltMath.Cross(rA, tangent);
      Fix64 rtB = VoltMath.Cross(rB, tangent);
      Fix64 effectiveMassTangent = invMassA + invMassB + rtA * rtA * invIA + rtB * rtB * invIB;

      if (effectiveMassTangent < VoltConfig.MINIMUM_DYNAMIC_MASS) return;
      
      VoltBody bodyA = manifold.ShapeA.Body;
      VoltBody bodyB = manifold.ShapeB.Body;

      VoltVector2 velA = bodyA.InternalLinearVelocity + VoltMath.CrossSV(rA, bodyA.AngularVelocity);
      VoltVector2 velB = bodyB.InternalLinearVelocity + VoltMath.CrossSV(rB, bodyB.AngularVelocity);
      VoltVector2 relVel = velB - velA;
      Fix64 CTDot = VoltVector2.Dot(tangent, relVel);
      Fix64 lambda = -CTDot / effectiveMassTangent;

      // Compute the maximum friction impulse according to Coulomb's model
      Fix64 maxFriction = manifold.Friction * this.accumulatedNormalLambda;
      
      // Clamp force between -maxFriction and maxFriction
      Fix64 oldAccum = this.accumulatedFrictionLambda;
      this.accumulatedFrictionLambda = VoltMath.Max(-maxFriction, VoltMath.Min(oldAccum + lambda, maxFriction));
      lambda = this.accumulatedFrictionLambda - oldAccum;

      VoltVector2 frictionImpulse = tangent * lambda;
      bodyA.ApplyImpulse(-frictionImpulse, worldA);
      bodyB.ApplyImpulse(frictionImpulse, worldB);
    }

    internal void SolveRestitution(Manifold manifold)
    {
      VoltBody bodyA = manifold.ShapeA.Body;
      VoltBody bodyB = manifold.ShapeB.Body;
      // Only apply restitution if:
      // 1. There's a restitution coefficient > 0
      // 2. The contact point is new this step (not persisted)
      // 3. The initial relative velocity was approaching fast enough
      Fix64 restitutionThreshold = (Fix64)1.0; // Increased threshold
      
      if (manifold.Restitution == (Fix64)0) {
          return;
      }
      
      if (this.relativeVelocity > -restitutionThreshold) {
          return;
      }

      Fix64 rnA = VoltMath.Cross(rA, normal);
      Fix64 rnB = VoltMath.Cross(rB, normal);
      Fix64 effectiveMass =
        this.invMassA + this.invMassB + rnA * rnA * this.invIA + rnB * rnB * this.invIB;
      if (effectiveMass < VoltConfig.MINIMUM_DYNAMIC_MASS) return;

      // Calculate current velocities
      VoltVector2 velA = bodyA.InternalLinearVelocity + VoltMath.CrossSV(rA, bodyA.AngularVelocity);
      VoltVector2 velB = bodyB.InternalLinearVelocity + VoltMath.CrossSV(rB, bodyB.AngularVelocity);
      VoltVector2 relVel = velB - velA;
      Fix64 vn = VoltVector2.Dot(normal, relVel);

      // Compute restitution impulse
      // We want the final velocity to be -e * initial velocity
      // So we need to change from current vn to -e * relativeVelocity
      // velocity change = -e * relativeVelocity - vn
      // impulse = mass * velocity change
      Fix64 impulse = -(vn + manifold.Restitution * this.relativeVelocity) / effectiveMass;

      // Only apply positive impulses (separating)
      if (impulse > (Fix64)0) {
          VoltVector2 restitutionImpulse = this.normal * (impulse);
          bodyA.ApplyImpulse(-restitutionImpulse, this.worldA);
          bodyB.ApplyImpulse(restitutionImpulse, this.worldB);
      }
    }

    #region Internals
    private void Reset()
    {
      this.worldPoint = VoltVector2.zero;
      this.normal = VoltVector2.zero;
      this.penetration = Fix64.Zero;
      localA = VoltVector2.zero;
      localB = VoltVector2.zero;
      worldA = VoltVector2.zero;
      worldB = VoltVector2.zero;
      rA = VoltVector2.zero;
      rB = VoltVector2.zero;
      tangent = VoltVector2.zero;
      relativeVelocity = Fix64.Zero;
      invMassA = Fix64.Zero;
      invMassB = Fix64.Zero;
      invIA = Fix64.Zero;
      invIB = Fix64.Zero;
      accumulatedNormalLambda = Fix64.Zero;
      accumulatedFrictionLambda = Fix64.Zero;
    }
    #endregion
  }
}