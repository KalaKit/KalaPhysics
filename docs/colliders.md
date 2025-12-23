# KalaPhysics — Collider Shapes, Requirements & Meaning

## BSP (Bounding Sphere)

**Shape**  
A perfect sphere in world space.

**Requirements**
- **center (vec3)**  
  World-space position of the sphere’s center. This is the sole spatial reference.
- **radius (float)**  
  Distance from the center to the surface in all directions. Fully defines the size.

**Rationale**  
A sphere is mathematically complete with just center and radius. This yields the fastest possible distance, overlap, and ray tests and makes BSP ideal for broadphase filtering.

---

## AABB (Axis-Aligned Bounding Box)

**Shape**  
A rectangular box aligned with the world axes.

**Requirements**
- **min (vec3)**  
  World-space corner with the smallest X, Y, and Z values.
- **max (vec3)**  
  World-space corner with the largest X, Y, and Z values.

**Rationale**  
Min and max directly encode the spatial bounds of the box. This representation enables extremely cheap overlap tests and is the natural form for broadphase and spatial partitioning.

---

## OBB (Oriented Bounding Box)

**Shape**  
A box with fixed dimensions that can rotate freely in space.

**Requirements**
- **position (vec3 reference)**  
  World-space center of the box. Moving this moves the OBB.
- **rotation (quat reference)**  
  World-space orientation of the box, defining its local axes.
- **halfExtents (vec3)**  
  Half-size of the box along each local axis. Defines the intrinsic shape.

**Rationale**  
This cleanly separates shape (half-extents) from motion (position and rotation). The box geometry is implicit and reconstructed procedurally without vertex data.

---

## BCP (Bounding Capsule, upright)

**Shape**  
A vertical capsule aligned to the world up-axis.

**Requirements**
- **position (vec3 reference)**  
  World-space center of the capsule along its vertical axis.
- **height (float)**  
  Distance between the centers of the two hemispherical ends.
- **radius (float)**  
  Radius of the cylindrical body and hemispherical caps.

**Rationale**  
An upright capsule is rotationally invariant and ideal for characters and agents. These values fully define its collision behavior while ensuring stability and smooth contact.

---

## KDOP (Discrete Oriented Polytope)

**Shape**  
A convex volume defined by projecting geometry onto a fixed set of oriented planes.

**Requirements**
- **vertices (span of vec3 references)**  
  Source points projected onto the plane set to define the volume.
- **position (vec3 reference)**  
  World-space location of the geometry.
- **rotation (quat reference)**  
  Orientation used when projecting vertices onto the planes.

**Rationale**  
KDOPs are geometry-derived by nature. The vertex set defines the shape, while position and rotation determine how it occupies world space.

---

## BCH (Bounding Convex Hull)

**Shape**  
The exact convex hull enclosing a set of points.

**Requirements**
- **vertices (span of vec3 references)**  
  Points from which the convex hull is constructed.
- **position (vec3 reference)**  
  World-space placement of the hull.
- **rotation (quat reference)**  
  Orientation of the hull in world space.

**Rationale**  
A convex hull cannot be reconstructed procedurally. The vertex set is the shape; position and rotation place it correctly in the simulation.
