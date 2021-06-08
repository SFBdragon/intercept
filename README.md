# intercept

An under-development, nonrotational, 2D collision library intended for use in game projects.   
Includes both a broadphase and narrow phase, with intersection and line segment testing between and against AABBs, circles, and convex polygons.   
The broadphase uses continuous collision detection and a Sweep & Prune broadphase implementation, with customisable collision resolutions and triggers.   

By using pre-solve, continuous collision detenction, tunneling is completely avoided, but 'ghost' collisions may still occur, increasingly so at large time steps. Substepping is, however, avoided in the traditional sense.   

Uses a system of 'static' colliders and 'reacter' colliders:   
- Static-Static: No detection.
- Static-Reacter: The Static collider's trigger is invoked, and the Reacter collider's reacter is invoked.
- Reacter-Reacter: The trigger of both Reacter collider's are invoked.

This setup is meant to be ideal for most 2d game projects where true physical simulation is almost never desired, instead custom movements based on what was collided with. Static colliders may move, but do not change velocity on collisions, and define whether their velocity is imparted on Reacter colliders than collide with it. Reacter colliders come with three pre-defined `Reacter`s: 'halt', 'deflect', and 'slide' in `crate::broad::reacter::*`.

WARNING: This is currently only a personal project, and while potentially functional has yet to be properly tested in another project. Performance is the largest factor in development hereof, but I have not tested whether I have successfully made a performant broadphase.  

If you see potential in this for your own use, feel free to let me know, and contributions are welcome.   
