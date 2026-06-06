**Decision:** For the vehicle model the kinematic bicycle model shall be used

**Rationale:** 
- Well established in the literature. 
- In case of low speeds on flat terrain it is rather precise
- It's complexity is in the same ballpark as other parts of the simulation

**Known limitations:**
- It represents a two-axle vehicle with front wheel steering
- Not capable to represent high speed or low mue scenarios, where tyre dynamics would play a role

**Alternatives:**
- Dynamic Bicycle model: would expand the capabilities of the model to higher speeds ad special surfaces, at the cost of complexity
- Two wheeled robot model: I am considering this to test how modular the simulation turns out. Would require a different yaw control
