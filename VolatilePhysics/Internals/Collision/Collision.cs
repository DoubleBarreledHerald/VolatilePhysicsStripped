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

//REF: https://github.com/majikayogames/physics-tutorial

#if UNITY
using UnityEngine;
#endif

using System;
using System.Collections.Generic;
using System.Linq;
using FixMath.NET;

namespace Volatile
{
  internal static class Collision
  {
    #region Dispatch
    private delegate Manifold Test(
      VoltWorld world,
      VoltShape sa, 
      VoltShape sb);

    private readonly static Test[,] tests = new Test[,]
      {
        { __Circle_Circle, __Circle_Polygon },
        { __Polygon_Circle, __Polygon_Polygon}
      };

    internal static Manifold Dispatch(
      VoltWorld world,
      VoltShape sa, 
      VoltShape sb)
    {
      if (!sa.IsEnabled || !sb.IsEnabled) return null;
      Test test = Collision.tests[(int)sa.Type, (int)sb.Type];
      return test(world, sa, sb);
    }

    private static Manifold __Circle_Circle(
      VoltWorld world,
      VoltShape sa, 
      VoltShape sb)
    {
      return Circle_Circle(world, (VoltCircle)sa, (VoltCircle)sb);
    }

    private static Manifold __Circle_Polygon(
      VoltWorld world,
      VoltShape sa, 
      VoltShape sb)
    {
      return Circle_Polygon(world, (VoltCircle)sa, (VoltPolygon)sb);
    }

    private static Manifold __Polygon_Circle(
      VoltWorld world,
      VoltShape sa, 
      VoltShape sb)
    {
      return Circle_Polygon(world, (VoltCircle)sb, (VoltPolygon)sa);
    }

    private static Manifold __Polygon_Polygon(
      VoltWorld world,
      VoltShape sa, 
      VoltShape sb)
    {
      return Polygon_Polygon(world, (VoltPolygon)sa, (VoltPolygon)sb);
    }
    #endregion

    #region Collision Tests
    private static Manifold Circle_Circle(
      VoltWorld world,
      VoltCircle circA,
      VoltCircle circB)
    {
      return 
        TestCircles(
          world,
          circA, 
          circB,
          circB.worldSpaceOrigin, 
          circB.radius);
    }

    private static Manifold Circle_Polygon(
      VoltWorld world,
      VoltCircle circ,
      VoltPolygon poly)
    {
      // Get the axis on the polygon closest to the circle's origin
      Fix64 penetration;
      int index =
        Collision.FindAxisMaxPenetration(
          circ.worldSpaceOrigin,
          circ.radius,
          poly,
          out penetration);

      if (index < 0)
        return null;

      VoltVector2 a, b;
      poly.GetEdge(index, out a, out b);
      Axis axis = poly.GetWorldAxis(index);

      // If the circle is past one of the two vertices, check it like
      // a circle-circle intersection where the vertex has radius 0
      Fix64 d = VoltMath.Cross(axis.Normal, circ.worldSpaceOrigin);
      if (d > VoltMath.Cross(axis.Normal, a))
        return Collision.TestCircles(world, circ, poly, a, Fix64.Zero);
      if (d < VoltMath.Cross(axis.Normal, b))
        return Collision.TestCircles(world, circ, poly, b, Fix64.Zero);

      // Build the collision Manifold
      Manifold manifold = world.AllocateManifold().Assign(world, circ, poly);
      VoltVector2 pos =
        circ.worldSpaceOrigin - (circ.radius + penetration / (Fix64)2) * axis.Normal;
      manifold.AddContact(pos, -axis.Normal, penetration);
      return manifold;
    }

    private static Manifold Polygon_Polygon(
      VoltWorld world,
      VoltPolygon polyA,
      VoltPolygon polyB)
    {
      switch (VoltConfig.SOLVER_TYPE)
      {
        case VoltConfig.SolverType.Fast:
          return FAST_Polygon_Polygon(world, polyA, polyB);
        case VoltConfig.SolverType.SAT:
          return SAT_Polygon_Polygon(world, polyA, polyB);
        default:
          return FAST_Polygon_Polygon(world, polyA, polyB);
      }
    }

    private static Manifold FAST_Polygon_Polygon(
      VoltWorld world,
      VoltPolygon polyA,
      VoltPolygon polyB)
    {
      Axis a1, a2;
      if (Collision.FindMinSepAxis(polyA, polyB, out a1) == false)
        return null;
      if (Collision.FindMinSepAxis(polyB, polyA, out a2) == false)
        return null;

      // We will use poly1's axis, so we may need to swap
      if (a2.Width > a1.Width)
      {
        VoltUtil.Swap(ref polyA, ref polyB);
        VoltUtil.Swap(ref a1, ref a2);
      }

      // Build the collision Manifold
      Manifold manifold = 
        world.AllocateManifold().Assign(world, polyA, polyB);
      Collision.FindVerts(polyA, polyB, a1.Normal, a1.Width, manifold);
      return manifold;
    }

    private static Manifold SAT_Polygon_Polygon(VoltWorld world,
      VoltPolygon polyA,
      VoltPolygon polyB)
    {
      Manifold manifold = 
        world.AllocateManifold().Assign(world, polyA, polyB);

      if (PolyToPolySAT(polyA.worldVertices, polyB.worldVertices,
      out VoltVector2 normal, out Fix64 penetration,
      out bool referenceIsA, out int referenceEdgeIndex)) {

        VoltVector2 centerA = FindArithmeticMean(polyA.worldVertices);
        VoltVector2 centerB = FindArithmeticMean(polyB.worldVertices);
				// Ensure normal is always pointing A->B
				// CollisionConstraint expects this because it applies impulse from A->B in normal direction
				// If the normal does not point A->B, it will pull them towards each other instead of acting repulsive
				if (VoltVector2.Dot(normal, centerB - centerA) < Fix64.Zero) {
					normal = -normal;
				}

				// Determine clipped contact points
        List<VoltVector2> finalPoints;
        if (referenceIsA)
        {
          ClipPolyToPoly(polyA.Body, polyA, polyB.Body, polyB, referenceEdgeIndex, out finalPoints);
        } else
        {
          ClipPolyToPoly(polyB.Body, polyB, polyA.Body, polyA, referenceEdgeIndex, out finalPoints);
        }

        foreach (VoltVector2 voltVector2 in finalPoints)
        {
          manifold.AddContact(voltVector2, normal, penetration);
        }
      }

      return manifold;
    }
    #endregion

    #region Common Tests and Queries
    /// <summary>
    /// Simple check for point-circle containment.
    /// </summary>
    internal static bool TestPointCircleSimple(
      VoltVector2 point,
      VoltVector2 origin,
      Fix64 radius)
    {
      VoltVector2 delta = origin - point;
      return delta.sqrMagnitude <= (radius * radius);
    }

    /// <summary>
    /// Simple check for two overlapping circles.
    /// </summary>
    internal static bool TestCircleCircleSimple(
      VoltVector2 originA,
      VoltVector2 originB,
      Fix64 radiusA,
      Fix64 radiusB)
    {
      Fix64 radiusTotal = radiusA + radiusB;
      return (originA - originB).sqrMagnitude <= (radiusTotal * radiusTotal);
    }

    /// <summary>
    /// Checks a ray against a circle with a given origin and square radius.
    /// </summary>
    internal static bool CircleRayCast(
      VoltShape shape,
      VoltVector2 shapeOrigin,
      Fix64 sqrRadius,
      ref VoltRayCast ray,
      ref VoltRayResult result)
    {
      VoltVector2 toOrigin = shapeOrigin - ray.origin;

      if (toOrigin.sqrMagnitude < sqrRadius)
      {
        result.SetContained(shape);
        return true;
      }

      Fix64 slope = VoltVector2.Dot(toOrigin, ray.direction);
      if (slope < Fix64.Zero)
        return false;

      Fix64 sqrSlope = slope * slope;
      Fix64 d = sqrRadius + sqrSlope - VoltVector2.Dot(toOrigin, toOrigin);
      if (d < Fix64.Zero)
        return false;

      Fix64 dist = slope - VoltMath.Sqrt(d);
      if (dist < Fix64.Zero || dist > ray.distance)
        return false;

      // N.B.: For historical raycasts this normal will be wrong!
      // Must be either transformed back to world or invalidated later.
      VoltVector2 normal = (dist * ray.direction - toOrigin).normalized;
      result.Set(shape, dist, normal);
      return true;
    }


    /// <summary>
    /// Returns the index of the nearest axis on the poly to a point.
    /// Outputs the minimum distance between the axis and the point.
    /// </summary>
    internal static int FindAxisShortestDistance(
      VoltVector2 point,
      Axis[] axes,
      out Fix64 minDistance)
    {
      int ix = 0;
      minDistance = Fix64.MaxValue;
      bool inside = true;

      for (int i = 0; i < axes.Length; i++)
      {
        Fix64 dot = VoltVector2.Dot(axes[i].Normal, point);
        Fix64 dist = axes[i].Width - dot;

        if (dist < Fix64.Zero)
          inside = false;

        if (dist < minDistance)
        {
          minDistance = dist;
          ix = i;
        }
      }

      if (inside == true)
      {
        minDistance = Fix64.Zero;
        ix = -1;
      }

      return ix;
    }

    /// <summary>
    /// Returns the index of the axis with the max circle penetration depth.
    /// Breaks out if a separating axis is found between the two shapes.
    /// Outputs the penetration depth of the circle in the axis (if any).
    /// </summary>
    internal static int FindAxisMaxPenetration(
      VoltVector2 origin,
      Fix64 radius,
      VoltPolygon poly,
      out Fix64 penetration)
    {
      int index = 0;
      int found = 0;
      penetration = Fix64.MinValue;

      for (int i = 0; i < poly.countWorld; i++)
      {
        Axis axis = poly.worldAxes[i];
        Fix64 dot = VoltVector2.Dot(axis.Normal, origin);
        Fix64 dist = dot - axis.Width - radius;

        if (dist > Fix64.Zero)
          return -1;

        if (dist > penetration)
        {
          penetration = dist;
          found = index;
        }

        index++;
      }

      return found;
    }

    // REF: https://stackoverflow.com/questions/1119451/how-to-tell-if-a-line-intersects-a-polygon-in-c
    /// <summary>
    /// Simple check for the intersection of two lines.
    /// </summary>
    internal static bool TestLineLineSimple(VoltVector2 start1, VoltVector2 end1, VoltVector2 start2, VoltVector2 end2, out VoltVector2 intersection)
    {
        Fix64 denom = ((end1.x - start1.x) * (end2.y - start2.y)) - ((end1.y - start1.y) * (end2.x - start2.x));

        intersection = VoltVector2.zero;

        //  AB & CD are parallel 
        if (denom == Fix64.Zero)
            return false;

        Fix64 numer = ((start1.y - start2.y) * (end2.x - start2.x)) - ((start1.x - start2.x) * (end2.y - start2.y));

        Fix64 r = numer / denom;

        Fix64 numer2 = ((start1.y - start2.y) * (end1.x - start1.x)) - ((start1.x - start2.x) * (end1.y - start1.y));

        Fix64 s = numer2 / denom;

        if (r < Fix64.Zero || r > Fix64.One || s < Fix64.Zero || s > Fix64.One)
            return false;

        // Find intersection point
        intersection.x = start1.x + (r * (end1.x - start1.x));
        intersection.y = start1.y + (r * (end1.y - start1.y));

        return true;
    }

    /// <summary>
    /// Simple check for Line AABB intersection.
    /// </summary>
    internal static bool TestLineAABBSimple(VoltVector2 start, VoltVector2 end, VoltAABB AABB){
      if(AABB.TopRight.x < VoltMath.Min(start.x, end.x)){
          return false;
      }
      if(AABB.BottomLeft.x > VoltMath.Max(start.x, end.x)){
          return false;
      }
      if(AABB.TopRight.y < VoltMath.Min(start.y, end.y)){
          return false;
      }
      if(AABB.BottomLeft.y > VoltMath.Max(start.y, end.y)){
          return false;
      }

      return true;
    }

    internal static bool TestLinePolygonSimple(VoltVector2 start, VoltVector2 end, VoltPolygon polygon)
    {
      if (!TestLineAABBSimple(start, end, polygon.AABB)) return false;

      //int closestIndex = FindAxisShortestDistance(start, polygon.worldAxes, out _);

      for (int i = 0; i < polygon.worldVertices.Count(); i++)
      {
        polygon.GetEdge(i, out VoltVector2 start2, out VoltVector2 end2);
        if (TestLineLineSimple(start, end, start2, end2, out _)) return true;
      }

      return false;
    }
    #endregion


    internal static bool TestPolygonPolygonSimple(
      VoltPolygon polyA,
      VoltPolygon polyB)
    {
      for (int i = 0; i < polyA.countWorld; i++)
      {
        VoltVector2 vertex = polyA.worldVertices[i];
        if (polyB.ContainsPoint(vertex) == true)
          return true;
      }

      for (int i = 0; i < polyB.countWorld; i++)
      {
        VoltVector2 vertex = polyB.worldVertices[i];
        if (polyA.ContainsPoint(vertex) == true)
        {
          return true;
        }
      }

      // Star of david check

      Axis a1, a2;
      if (Collision.FindMinSepAxis(polyA, polyB, out a1) == false)
        return false;
      if (Collision.FindMinSepAxis(polyB, polyA, out a2) == false)
        return false;

      // We will use poly1's axis, so we may need to swap
      if (a2.Width > a1.Width)
      {
        VoltUtil.Swap(ref polyA, ref polyB);
        VoltUtil.Swap(ref a1, ref a2);
      }
      for (int i = 0; i < polyA.countWorld; i++)
      {
        VoltVector2 vertex = polyA.worldVertices[i];
        if (polyB.ContainsPointPartial(vertex, a1.Normal) == true)
        {
          return true;
        }
      }

      for (int i = 0; i < polyB.countWorld; i++)
      {
        VoltVector2 vertex = polyB.worldVertices[i];
        if (polyA.ContainsPointPartial(vertex, -a1.Normal) == true)
        {
          return true;
        }
      }

      return false;
    }

    //REF: https://www.csharphelper.com/howtos/howto_line_circle_intersection.html
    // Find the points of intersection.
    internal static bool TestCircleLineSimple(
        VoltVector2 circleOrigin, Fix64 radius,
        VoltVector2 point1, VoltVector2 point2)
    {
        Fix64 dx, dy, A, B, C, det;

        dx = point2.x - point1.x;
        dy = point2.y - point1.y;

        A = dx * dx + dy * dy;
        B = (Fix64)2 * (dx * (point1.x - circleOrigin.x) + dy * (point1.y - circleOrigin.y));
        C = (point1.x - circleOrigin.x) * (point1.x - circleOrigin.x) +
            ((point1.y - circleOrigin.x) * (point1.y - circleOrigin.x)) -
            radius * radius;

        det = B * B - (Fix64)4 * A * C;
        if ((A <= (Fix64)0.0000001) || (det < Fix64.Zero))
        {
            // No real solutions.
            return false;
        }

        return true;
    }

    #region Helpers
    /// <summary>
    /// Workhorse for circle-circle collisions, compares origin distance
    /// to the sum of the two circles' radii, returns a Manifold.
    /// </summary>
    /// 
    private static Manifold TestCircles(
      VoltWorld world,
      VoltCircle shapeA,
      VoltShape shapeB,
      VoltVector2 overrideBCenter, // For testing vertices in circles
      Fix64 overrideBRadius)
    {
      VoltVector2 r = overrideBCenter - shapeA.worldSpaceOrigin;
      Fix64 min = shapeA.radius + overrideBRadius;
      Fix64 distSq = r.sqrMagnitude;

      if (distSq >= min * min)
        return null;

      Fix64 dist = VoltMath.Sqrt(distSq);

      // 최소값을 지정하여 divide by zero 방지
      Fix64 distInv = Fix64.One / VoltMath.Max(dist, min / (Fix64)10);

      VoltVector2 pos =
        shapeA.worldSpaceOrigin +
        (Fix64.One / (Fix64)2 + distInv * (shapeA.radius - min / (Fix64)2)) * r;

      // Build the collision Manifold
      Manifold manifold = 
        world.AllocateManifold().Assign(world, shapeA, shapeB);
      manifold.AddContact(pos, distInv * r, dist - min);
      return manifold;
    }

    private static bool FindMinSepAxis(
      VoltPolygon poly1,
      VoltPolygon poly2,
      out Axis axis)
    {
      axis = new Axis(VoltVector2.zero, Fix64.MinValue);

      for (int i = 0; i < poly1.countWorld; i++)
      {
        Axis a = poly1.worldAxes[i];
        Fix64 min = Fix64.MaxValue;
        for (int j = 0; j < poly2.countWorld; j++)
        {
          VoltVector2 v = poly2.worldVertices[j];
          min = VoltMath.Min(min, VoltVector2.Dot(a.Normal, v));
        }
        min -= a.Width;

        if (min > Fix64.Zero)
          return false;
        if (min > axis.Width)
          axis = new Axis(a.Normal, min);
      }

      return true;
    }

    /// <summary>
    /// Add contacts for penetrating vertices. Note that this does not handle
    /// cases where an overlap was detected, but no vertices fall inside the
    /// opposing polygon (like a Star of David). For this we have a fallback.
    /// 
    /// See http://chipmunk-physics.googlecode.com/svn/trunk/src/cpCollision.c
    /// </summary>
    private static void FindVerts(
      VoltPolygon poly1,
      VoltPolygon poly2,
      VoltVector2 normal,
      Fix64 penetration,
      Manifold manifold)
    {
      bool found = false;

      for (int i = 0; i < poly1.countWorld; i++)
      {
        VoltVector2 vertex = poly1.worldVertices[i];
        if (poly2.ContainsPoint(vertex) == true)
        {
          if (manifold.AddContact(vertex, normal, penetration) == false)
            return;
          found = true;
          break;
        }
      }

      for (int i = 0; i < poly2.countWorld; i++)
      {
        VoltVector2 vertex = poly2.worldVertices[i];
        if (poly1.ContainsPoint(vertex) == true)
        {
          if (manifold.AddContact(vertex, normal, penetration) == false)
            return;
          found = true;
          break;
        }
      }

      // Fallback to check the degenerate "Star of David" case
      if (found == false)
        FindVertsFallback(poly1, poly2, normal, penetration, manifold);
    }

    /// <summary>
    /// A fallback for handling degenerate "Star of David" cases.
    /// </summary>
    private static void FindVertsFallback(
      VoltPolygon poly1,
      VoltPolygon poly2,
      VoltVector2 normal,
      Fix64 penetration,
      Manifold manifold)
    {
      for (int i = 0; i < poly1.countWorld; i++)
      {
        VoltVector2 vertex = poly1.worldVertices[i];
        if (poly2.ContainsPointPartial(vertex, normal) == true)
        {
          if (manifold.AddContact(vertex, normal, penetration) == false)
            return;
          break;
        }
      }

      for (int i = 0; i < poly2.countWorld; i++)
      {
        VoltVector2 vertex = poly2.worldVertices[i];
        if (poly1.ContainsPointPartial(vertex, -normal) == true)
        {
          if (manifold.AddContact(vertex, normal, penetration) == false)
            return;
          break;
        }
      }
    }

    public static bool PolyToPolySAT(VoltVector2[] verticesA, VoltVector2[] verticesB, out VoltVector2 normal, out Fix64 penetration, out bool referenceIsA, out int referenceEdgeIndex)
    {
      normal = VoltVector2.zero;
      penetration = Fix64.MaxValue;
      referenceIsA = true;
      referenceEdgeIndex = 0;

      //Get Polygon Edge Normals
      VoltVector2[] normalsA = GetNormals(verticesA);
      VoltVector2[] normalsB = GetNormals(verticesB);

      Fix64 minSepA = Fix64.MinValue;
      int minEdgeA = 0;
      Fix64 minSepB = Fix64.MinValue;
      int minEdgeB = 0;

      for(int i = 0; i < verticesA.Length; i++)
      {
        VoltVector2 va = verticesA[i];
        VoltVector2 vb = verticesA[(i + 1) % verticesA.Length];

        ProjectVertices(verticesA, normalsA[i], out Fix64 minA, out Fix64 maxA);
        ProjectVertices(verticesB, normalsA[i], out Fix64 minB, out Fix64 maxB);

        //No collision
        if(minA >= maxB || minB >= maxA)
          return false;
        
        Fix64 seperation = minB - maxA;
        if (seperation > minSepA) {
          minSepA = seperation;
          minEdgeA = i;
        }
      }

      for (int i = 0; i < verticesB.Length; i++)
      {
        VoltVector2 va = verticesB[i];
        VoltVector2 vb = verticesB[(i + 1) % verticesB.Length];

        ProjectVertices(verticesA, normalsB[i], out Fix64 minA, out Fix64 maxA);
        ProjectVertices(verticesB, normalsB[i], out Fix64 minB, out Fix64 maxB);

        //No collision
        if (minA >= maxB || minB >= maxA)
          return false;

        Fix64 seperation = minA - maxB;
        if (seperation > minSepB) {
          minSepB = seperation;
          minEdgeB = i;
        }
      }

      referenceEdgeIndex = minEdgeA;
      normal = normalsA[referenceEdgeIndex];

      if (minSepB > minSepA)
      {
        referenceIsA = false;
        referenceEdgeIndex = minEdgeB;
        normal = -normalsB[referenceEdgeIndex];
      }

      penetration = referenceIsA ? -minSepA : -minSepB;

      return true;
    }

    private static VoltVector2 FindArithmeticMean(VoltVector2[] vertices)
    {
      Fix64 sumX = Fix64.Zero;
      Fix64 sumY = Fix64.Zero;

      for(int i = 0; i < vertices.Length; i++)
      {
        VoltVector2 v = vertices[i];
        sumX += v.x;
        sumY += v.y;
      }

      return new VoltVector2(sumX / (Fix64)vertices.Length, sumY / (Fix64)vertices.Length);
    }

    private static void ProjectVertices(VoltVector2[] vertices, VoltVector2 axis, out Fix64 min, out Fix64 max)
    {
      min = Fix64.MaxValue;
      max = Fix64.MinValue;

      for(int i = 0; i < vertices.Length; i++)
      {
        VoltVector2 v = vertices[i];
        Fix64 proj = VoltVector2.Dot(v, axis);

        if(proj < min) { min = proj; }
        if(proj > max) { max = proj; }
      }
    }

    private static void ClipPolyToPoly(
      VoltBody referenceBody, VoltPolygon referencePoly,
      VoltBody incidentObject, VoltPolygon incidentPoly,
      int referenceEdgeIndex,
      out List<VoltVector2> finalPoints
      )
    {
      finalPoints = new List<VoltVector2>();
      // Generate contact points with reference object/shape and incident object/shape
      VoltVector2[] referenceVerts = referencePoly.worldVertices;
      VoltVector2[] incidentVerts = incidentPoly.worldVertices;
      //Normals
      VoltVector2 a1 = referenceVerts[referenceEdgeIndex];
      VoltVector2 a2 = referenceVerts[(referenceEdgeIndex + 1) % referenceVerts.Count()];
      VoltVector2 edge = a2 - a1;
      VoltVector2 axis = new VoltVector2(-edge.y, edge.x);
      VoltVector2 referenceNormal = axis.normalized;
      VoltVector2[] incidentNormals = GetNormals(incidentVerts);

      VoltVector2 n = referenceNormal;

      // Incident edge selection: edge with normal pointing most opposite to n
      Fix64 lowestDot = Fix64.MaxValue;
      int incidentIndex = 0;
      for (int i = 0; i < incidentNormals.Length; i++)
      {
        Fix64 dot = VoltVector2.Dot(n, incidentNormals[i]);
        if (dot < lowestDot) {
          lowestDot = dot;
          incidentIndex = i;
        }
      }
      VoltVector2 b2 = incidentVerts[incidentIndex];
      VoltVector2 b1 = incidentVerts[(incidentIndex + 1) % incidentVerts.Length];

      // Clip to start and end faces. Tangents on ends of reference edge. |-----|
      VoltVector2 refTangent = (a2 - a1).normalized;

      List<VoltVector2> clippedPoints =
        ClipLineSegmentToLine(b1, b2, -refTangent, a1);
      
      if (clippedPoints.Count == 0) return;

      clippedPoints =
        ClipLineSegmentToLine(clippedPoints[0], clippedPoints[1],
          refTangent, a2);

      // Keep points that are behind the reference face, plus speculative slop like Box2D
      finalPoints = clippedPoints.FindAll(
        v => VoltVector2.Dot(n, v - a1) <= VoltConfig.ResolveSlop);

      //TODO Finish after contact constraints
		  /* // Box2D style feature IDs: combine object/shape IDs with vertex indices
      int i11 = referenceEdgeIndex; // ref edge start vertex
      int i12 = (i11 + 1) % referenceVerts.Length; // ref edge end vertex
      int i21 = (incidentIndex + 1) % incidentVerts.Length; // incident edge start vertex (b1)
      int i22 = incidentIndex;
      var prefix =
        ((referenceBody.ID & 0xFF) << 24) |
        ((incidentObject.ID & 0xFF) << 16) |
        ((referenceBody.ID)) */
    }

    private static List<VoltVector2> ClipLineSegmentToLine(VoltVector2 p1, VoltVector2 p2, VoltVector2 normal, VoltVector2 offset) {
      List<VoltVector2> clippedPoints = new List<VoltVector2>();
      Fix64 distance0 = VoltVector2.Dot(p1 - offset, normal);
      Fix64 distance1 = VoltVector2.Dot(p2 - offset, normal);
      // If the points are behind the plane, don't clip
      if (distance0 <= Fix64.Zero) clippedPoints.Add(p1);
      if (distance1 <= Fix64.Zero) clippedPoints.Add(p2);
      // If one is in front of the plane, have to clip it to the intersection point
      // clippedPoints.length < 2 for edge case where 1 point is exactly on the plane
      if (Fix64.Sign(distance0) != Fix64.Sign(distance1) &&
          clippedPoints.Count < 2) {
        Fix64 pctAcross = distance1 / (distance1 - distance0);
        VoltVector2 intersectionPt = p2 + ((p1 - p2) * pctAcross);
        clippedPoints.Add(intersectionPt);
      }
      return clippedPoints; // Returns 2 or 0 points
    }

    private static VoltVector2[] GetNormals(VoltVector2[] vertices)
    {
      int bLength = vertices.Length;
      VoltVector2[] normals = new VoltVector2[bLength];
      for(int i = 0; i < bLength; i++)
      {
        VoltVector2 va = vertices[i];
        VoltVector2 vb = vertices[(i + 1) % bLength];
        
        VoltVector2 edge = vb - va;
        VoltVector2 axis = new VoltVector2(-edge.y, edge.x);
        axis = axis.normalized;
        normals[i] = axis;
      }
      return normals;
    }
    #endregion
  }
}
