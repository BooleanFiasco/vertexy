FORMULAS:

// Defines a forumla named stepPassable that takes 2 parameters
// As far as I can tell, the first param is ALWAYS a vertex?
// Second param is (I think?) inferred by the usage below where it passes a "step"
VXY_FORMULA(stepPassable, 2);

// This is what the formula actually DOES below:

// Return true(?) if the vertex is type M_PASSABLE
stepPassable(vertex,step) <<= cellType(vertex).is(M_PASSABLE);
// Return true(?) if the vertex type is a door of index X that matches the previous step index X
// (For the maze program, you are assumed to have the key index that matches the previous step)
stepPassable(vertex,step) <<= cellType(vertex).is(cellType.doors[X]) && prevStep(X);

ANOTHER FORMULA EXAMPLE:
// Same params as the previous example (vertex, step) but this time uses a Domain to
// constraint the values of the second param (step)
VXY_DOMAIN_FORMULA(stepCell, CellStepDomain, 2);

// Just a mask
auto M_STEP_REACHABLE = stepCell.reachable|stepCell.origin;

// Constrain the vertex reachability to `origin` if the type is `entrance`
stepCell(vertex,step).is(stepCell.origin) <<= cellType(vertex).is(cellType.entrance);
// Constrain the vertex reachability to `reachable` or `unreachable` if the the type is NOT entrance
stepCell(vertex,step).is(stepCell.reachable|stepCell.unreachable) <<= ~cellType(vertex).is(cellType.entrance);
// Constrain the vertex to `unreachable` if the the `stepPassable` function returns FALSE (e.g. NOT passable)
stepCell(vertex,step).is(stepCell.unreachable) <<= ~stepPassable(vertex,step);
