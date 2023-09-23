# Zeta Resources

* ### [About](https://github.com/Salamence064/Zeta-Resources#about-1)
* ### [The Zeta Codebase](https://github.com/Salamence064/Zeta-Resources#the-zeta-codebase-1)
* ### [Math for Zeta](https://github.com/Salamence064/Zeta-Resources#math-for-zeta-1)
* ### [Physics for Zeta](https://github.com/Salamence064/Zeta-Resources#physics-for-zeta-1)
* ### [Getting Started with Git and Github](https://github.com/Salamence064/Zeta-Resources#getting-started-with-git-and-github-1)

___

## About

___

## The Zeta Codebase

* ##### [General Structure](https://github.com/Salamence064/Zeta-Resources#general-structure-1)
* ##### [ZMath](https://github.com/Salamence064/Zeta-Resources#zmath-1)
* ##### [Primitives](https://github.com/Salamence064/Zeta-Resources#primitives-1)
* ##### [Rigid and Static Bodies](https://github.com/Salamence064/Zeta-Resources#rigid-and-static-bodies-1)
* ##### [The Physics Handler](https://github.com/Salamence064/Zeta-Resources#the-physics-handler-1)
* ##### [Important Notes](https://github.com/Salamence064/Zeta-Resources#important-notes-1)
* ##### [2D Engine to Reference](https://github.com/Salamence064/Zeta-Resources#2d-engine-to-reference-1)

___

#### General Structure
* **6 Main Sections:**
  1. zmath.h
  2. primitives.h
  3. bodies.h
  4. intersections.h
  5. collisions.h
  6. physicshandler.h
<br>

* **Two Namespaces:**
  1. ZMath - contains all the math library code
  2. Zeta - contains all the physics related code
<br>

* **Coordinate System Used**
  * The origin is in the bottom left of the screen by default for Zeta
  * The position of primitives and rigid and static bodies is considered to be the centerpoint instead of the top left corner
  * Zeta will still work for any arbitrary coordinate system if the user provides a custom gravity value

Our engine works by accepting rigid and static bodies from the user in a physics handler. This physics handler then updates the positions, velocities, and net forces of those objects each frame for the user as depicted below. A codeblock example of a base implementation is also shown.

<br>

![](CodeStructure.jpg)

<br>

```c++
#include <ZETA/physicshandler.h>

int main() {
    // Create a physics handler with the default settings
    Zeta::Handler handler;

    // Create the colliders
    Zeta::Sphere* s1 = new Zeta::Sphere(ZMath::Vec3D(100.0f, 120.0f, 100.0f), 50.0f);
    Zeta::Sphere* s2 = new Zeta::Sphere(ZMath::Vec3D(350.0f, 500.0f, -340.0f), 200.0f);

    // Create some rigid bodies
    Zeta::RigidBody3D rb1(
        s1->c,                       // centerpoint
        50.0f,                       // mass
        0.9f,                        // coefficient of restitution
        0.975f,                      // linear damping
        Zeta::RIGID_SPHERE_COLLIDER, // collider type
        s1                           // collider
    );

    Zeta::RigidBody3D rb2(
        s2->c,                       // centerpoint
        100.0f,                      // mass
        0.95f,                       // coefficient of restitution
        0.8f,                        // linear damping
        Zeta::RIGID_SPHERE_COLLIDER, // collider type
        s2                           // collider
    );

    // Add the rigid bodies to the handler
    handler.addRigidBody(&rb1);
    handler.addRigidBody(&rb2);

    // Program's dt loop
    float dt = 0.0f;

    // Note: windowShouldNotClose would be replaced with the exit window condition in the user's graphics library
    while (windowShouldNotClose) {
        /* Rendering/Drawing code would go here */

        handler.update(dt); // The handler subtracts from dt for you
        // Note: getEllapsedTime() would be replaced with the equivalent in the user's graphics library
        dt += getEllapsedTime();
    }

    return 0;
};
```
___

#### ZMath

___

#### Primitives

___

#### Rigid and Static Bodies

___

#### The Physics Handler

___

#### Important Notes

___

#### 2D Engine to Reference

* As a reference, check out [Zeta2D](https://github.com/Salamence064/Zeta2D)
* Zeta2D's a 2D version of our engine that I made
* 2D examples can help for visualizing or converting it to 3D
* Zeta2D does not have rotational kinematics yet
___

## Math for Zeta

___

## Physics for Zeta

___

## Getting Started with Git and Github


