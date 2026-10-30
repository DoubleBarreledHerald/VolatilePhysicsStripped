using System;
using System.Collections.Generic;
using System.Linq;
using FixMath.NET;

namespace Volatile
{
    internal static class Query
    {
        #region Dispatch
        private delegate Intersection Test(
          VoltWorld world,
          VoltShape sa,
          VoltShape sb);

        private readonly static Test[,] tests = new Test[,]
          {
        { __Circle_Circle, __Circle_Polygon },
        { __Polygon_Circle, __Polygon_Polygon}
          };

        internal static Intersection Dispatch(
          VoltWorld world,
          VoltShape sa,
          VoltShape sb)
        {
            if (!sa.IsEnabled || !sb.IsEnabled) return null;
            Test test = Query.tests[(int)sa.Type, (int)sb.Type];
            return test(world, sa, sb);
        }

        private static Intersection __Circle_Circle(
          VoltWorld world,
          VoltShape sa,
          VoltShape sb)
        {
            return Circle_Circle(world, (VoltCircle)sa, (VoltCircle)sb);
        }

        private static Intersection __Circle_Polygon(
          VoltWorld world,
          VoltShape sa,
          VoltShape sb)
        {
            return Circle_Polygon(world, (VoltCircle)sa, (VoltPolygon)sb);
        }

        private static Intersection __Polygon_Circle(
          VoltWorld world,
          VoltShape sa,
          VoltShape sb)
        {
            return Polygon_Circle(world, (VoltPolygon)sb, (VoltCircle)sa);
        }

        private static Intersection __Polygon_Polygon(
          VoltWorld world,
          VoltShape sa,
          VoltShape sb)
        {
            return Polygon_Polygon(world, (VoltPolygon)sa, (VoltPolygon)sb);
        }
        #endregion

        #region Collision Tests
        private static Intersection Circle_Circle(
          VoltWorld world,
          VoltCircle circA,
          VoltCircle circB)
        {
            if (!Collision.CircleToCircle(
                circA, circB,
                out List<VoltVector2> positions,
                out VoltVector2 normal,
                out Fix64 penetration)
            )
                return null;

            return world.AllocateIntersection().Assign(circA, circB, positions, normal, penetration);
        }

        private static Intersection Circle_Polygon(
          VoltWorld world,
          VoltCircle circ,
          VoltPolygon poly)
        {
            List<VoltVector2> contacts;
            VoltVector2 normal;
            Fix64 penetration;

            switch (VoltConfig.SOLVER_TYPE)
            {
                case VoltConfig.SolverType.Fast:
                    if (!Collision.CircleToPolyFast(circ, poly, out contacts, out normal, out penetration))
                        return null;
                    break;
                case VoltConfig.SolverType.SAT:
                    if (!Collision.CircleToPolySAT(circ, poly, out contacts, out normal, out penetration))
                        return null;
                    break;
                default:
                    if (!Collision.CircleToPolyFast(circ, poly, out contacts, out normal, out penetration))
                        return null;
                    break;
            }

            return world.AllocateIntersection().Assign(circ, poly, contacts, normal, penetration);
        }

        private static Intersection Polygon_Circle(
          VoltWorld world,
          VoltPolygon poly,
          VoltCircle circ)
        {
            List<VoltVector2> contacts;
            VoltVector2 normal;
            Fix64 penetration;

            switch (VoltConfig.SOLVER_TYPE)
            {
                case VoltConfig.SolverType.Fast:
                    if (!Collision.CircleToPolyFast(circ, poly, out contacts, out normal, out penetration))
                        return null;
                    break;
                case VoltConfig.SolverType.SAT:
                    if (!Collision.CircleToPolySAT(circ, poly, out contacts, out normal, out penetration))
                        return null;
                    break;
                default:
                    if (!Collision.CircleToPolyFast(circ, poly, out contacts, out normal, out penetration))
                        return null;
                    break;
            }

            return world.AllocateIntersection().Assign(poly, circ, contacts, normal, penetration);
        }

        private static Intersection Polygon_Polygon(
          VoltWorld world,
          VoltPolygon polyA,
          VoltPolygon polyB)
        {
            List<VoltVector2> contacts;
            VoltVector2 normal;
            Fix64 penetration;

            switch (VoltConfig.SOLVER_TYPE)
            {
                case VoltConfig.SolverType.Fast:
                    if (!Collision.PolyToPolyFast(polyA, polyB, out contacts, out normal, out penetration))
                        return null;
                    break;
                case VoltConfig.SolverType.SAT:
                    if (!Collision.PolyToPolySAT(polyA, polyB, out contacts, out normal, out penetration))
                        return null;
                    break;
                default:
                    if (!Collision.PolyToPolyFast(polyA, polyB, out contacts, out normal, out penetration))
                        return null;
                    break;
            }
            return world.AllocateIntersection().Assign(polyA, polyB, contacts, normal, penetration);
        }
        #endregion
    }
}