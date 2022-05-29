package astar

import "math"

func abs(x int32) int32 {
    if x < 0 {
        return -x
    }
    return x
}

func min(x, y int32) int32 {
    if x < y {
        return x
    }
    return y
}

func getDelta(source_, target_ Vec2i) Vec2i {
    return Vec2i{abs(source_.X - target_.X), abs(source_.Y - target_.Y)}
}

func Manhattan(source_, target_ Vec2i) uint32 {
    delta := getDelta(source_, target_)
    return uint32(10 * (delta.X + delta.Y))
}

func Euclidean(source_, target_ Vec2i) uint32 {
    delta := getDelta(source_, target_)
    p := 2.0
    return uint32(10 * math.Sqrt(math.Pow(float64(delta.X), p)+math.Pow(float64(delta.Y), p)))
}

func Octagonal(source_, target_ Vec2i) uint32 {
    delta := getDelta(source_, target_)
    return uint32(10*(delta.X+delta.Y) + (-6)*min(delta.X, delta.Y))
}
