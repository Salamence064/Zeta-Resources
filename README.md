# <span style="color:fuchsia">Zeta Resources</span>

* ### <span style="color:thistle">About</span>
* ### <span style="color:thistle">The Zeta Codebase</span>
* ### <span style="color:thistle">Math for Zeta</span>
* ### <span style="color:thistle">Physics for Zeta</span>
* ### <span style="color:thistle">Getting Started with Git and Github</span>

___

## <span style="color:thistle">About</span>

___

## <span style="color:thistle">The Zeta Codebase</span>

* ##### <span style="color:violet">General Structure</span>
* ##### <span style="color:violet">ZMath</span>
* ##### <span style="color:violet">Primitives</span>
* ##### <span style="color:violet">Rigid and Static Bodies</span>
* ##### <span style="color:violet">The Physics Handler</span>
* ##### <span style="color:violet">Important Notes</span>
* ##### <span style="color:violet">2D Engine to Reference</span>

___

#### <span style="color:violet">General Structure</span>
* **6 Main Sections:**
  1. zmath.h
  2. primitives.h
  3. bodies.h
  4. intersections.h
  5. collisions.h
  6. physicshandler.h
<br>

* **Two Namespaces:**
  1. ZMath -- contains all the math library code
  2. Zeta -- contains all the physics related code
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

#### <span style="color:violet">ZMath</span>

___

#### <span style="color:violet">Primitives</span>

___

#### <span style="color:violet">Rigid and Static Bodies</span>

___

#### <span style="color:violet">Important Notes</span>

___

#### <span style="color:violet">2D Engine to Reference</span>

* As a reference, check out [Zeta2D](https://github.com/Salamence064/Zeta2D)
* Zeta2D's a 2D version of our engine that I made
* 2D examples can help for visualizing or converting it to 3D
* Zeta2D does not have rotational kinematics yet
___

## <span style="color:thistle">Math for Zeta</span>

___

## <span style="color:thistle">Physics for Zeta</span>

___

## <span style="color:thistle">Getting Started with Git and Github</span>


