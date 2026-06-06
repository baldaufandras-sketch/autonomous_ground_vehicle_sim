**Decision:** The Pure Pursuit Controller requires a lookahead point or target point to determine the steering wheel angle.
This point has to be on the path, and has to be exactly as far form the vehicle as is the lookahead distance.
As currently my path representation is linear I basically have to find the intersection of a circle and a line.

**Rationale:** Most direct solution with this setup (linear path between the waypoints). 
I also enjoyed doing some classic high-school math.

**Alternatives, future:** This definitely not scalable for higher order paths, so will has to be replaced with some
kind of numeric calculation