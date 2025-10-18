using System;
using System.Collections.Generic;

using UnityEngine;

using Volatile;

public abstract class VolatileShape : MonoBehaviour
{
  public abstract VoltShape PrepareShape(VoltWorld world);

  public abstract void DrawShapeInEditor();
  public abstract Vector2 ComputeTrueCenterOfMass();
}
