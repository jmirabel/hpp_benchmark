Notes
-----

# Some conclusions about benchmarks 2020-04-09 2020-04-10 2020-04-11

## Definition of the benchmarks

In these 3 benchmarks, the constraint graph is built in python. When two
neighbor edges are foliated, a waypoint edge with a level set edge at the first
place is inserted in both ways.

Let us rank these benchmarks in increasing order of constraints:
  1. case 1: 2020-04-10,
  2. case 2: 2020-04-11,
  3. case 3: 2020-04-09.

The differences between these cases resides in the following constraints
  1. grasp/complement between waypoints pregrasp and intersect,
  2. placement/complement between waypoints preplace and place. In this case,
     locked joints are not used as the concatenation of placement and
     placement/complement (factory.constraints.strict = True)

| Case | grasp/comp | placement/comp |
| ---- | ---------- | -------------- |
| 1    | no         | no             |
| 2    | yes        | no             |
| 3    | yes        | yes            |

## Results

| Benchmark                 | case 1 | case 2     | case 3  |          |
| ------------------------- | ------ | ---------- | ------- | -------- |
| baxter-manipulate-boxes   | 4      | 3.6        | 15.8    | time (s) |
|                           | 153.4  | 133.99     | 306.1   | nodes    |
| construction-set          | 10.9   | 3.71       | 10.71   | time (s) |
|                           | 47     | 39.4       | 23.85   | nodes    |
| pr2-manipulation-kitchen  | 17.25  | 15.78      | 62.45(*)| time (s) |
|                           | 128.5  | 121.6      | 452.5   | nodes    |
| pr2-manipulation-two-hand | 4.24   | 4.43       | 22.19   | time (s) |
|                           | 44.75  | 40.6       | 315.45  | nodes    |
| romeo-placard             | fail   | 155.85     | 180.56  | time (s) |
|                           | fail   | 712.0      | 819.35  | nodes    |
| ur3-spheres               | fail   | 67.68 (**) | 8.49    | time (s) |
|                           | fail   | 3482.0     | 260.3   | nodes    |

(*) Note that in case 3, benchmark pr2-manipulation-kitchen where there are
    two contact surfaces, along solution paths, the object discontinuously jumps
    from one surface to the other one.
(**) It turns out that level set edges going from a grasp state to state "free"
    always fail due to the fact that locked joints are used as complement of
    placement constraints. This leads to inserting incompatible preplacement and
    locked joint constraints in the target constraints.

## Additional remarks

 1. There might be a problem with level set edges inserted in waypoint edges.
    Enforcing the same right hand side along each edge of the waypoint edge for
    the parameterizing constraints may result in a difference bigger than the
    error threshold and therefore to a configuration that is considered as on
    a different leaf as expected. This may explain the poor result of
    ur3-spheres.
