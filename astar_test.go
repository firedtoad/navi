package astar

import (
    "math/rand"
    "runtime"
    "testing"
)

const W int32 = 500
const H int32 = 500
const PROB int32 = 2000

func BenchmarkAStarHeap(b *testing.B) {
    g := NewGenerator()
    g.SetHeuristic(Octagonal)
    g.SetWorldSize(W, H)
    cc := make([][]bool, W)
    r := rand.New(rand.NewSource(0))
    for i := range cc {
        cc[i] = make([]bool, H)
        for j := range cc[i] {
            if r.Int31n(10000) < PROB {
                cc[i][j] = true
            }
        }
    }
    g.SetCollision(func(v Vec2i) bool {
        x, y := v.X, v.Y
        if x < W && y < H {
            return cc[x][y]
        }
        return false
    })

    for i := 0; i < b.N; i++ {
        path := g.FindPathHeap(Vec2i{X: 0, Y: 0}, Vec2i{X: 499, Y: 499})
        runtime.KeepAlive(path)
    }
}

func BenchmarkAStarStack(b *testing.B) {
    g := NewGenerator()
    g.SetHeuristic(Octagonal)
    g.SetWorldSize(W, H)
    cc := make([][]bool, W)
    r := rand.New(rand.NewSource(0))
    for i := range cc {
        cc[i] = make([]bool, H)
        for j := range cc[i] {
            if r.Int31n(10000) < PROB {
                cc[i][j] = true
            }
        }
    }
    g.SetCollision(func(v Vec2i) bool {
        x, y := v.X, v.Y
        if x < W && y < H {
            return cc[x][y]
        }
        return false
    })

    for i := 0; i < b.N; i++ {
        path := g.FindPathStack(Vec2i{X: 0, Y: 0}, Vec2i{X: 499, Y: 499})
        runtime.KeepAlive(path)
    }
}