package astar

import (
    "math"
)

type CordStack struct {
    v []Vec2i
}

func (c *CordStack) Len() int {
    return len(c.v)
}

func (c *CordStack) Push(v Vec2i) {
    c.v = append(c.v, v)
}

func (c *CordStack) Pop() Vec2i {
    old := c.v[len(c.v)-1]
    c.v = c.v[:len(c.v)-1]
    return old
}

func (c *CordStack) Top() Vec2i {
    return c.v[len(c.v)-1]
}

func NewNodeStack(initCap uint) *CordStack {
    return &CordStack{v: make([]Vec2i, 0, initCap)}
}

func (g *Generator) FindPathStack(source_, target_ Vec2i) CoordinateList {
    if g.detectCollision(source_) || g.detectCollision(target_) {
        return nil
    }
    delta := getDelta(source_, target_)
    dist := uint(math.Max(float64(delta.X), float64(delta.Y))) + 1
    openStack := NewNodeStack(dist * 4)
    openSet := make(CordSet, dist*4)
    n := uint32(0)
    N := uint32(g.worldSize.X * g.worldSize.Y)
    reachTarget := false
    openStack.Push(source_)
    for ; openStack.Len() > 0; {
        current := openStack.Top()
        if current.Equals(target_) {
            reachTarget = true
            break
        }
        openSet[current.Hash()] = struct{}{}
        n++
        weight := g.relaxer(g.epsilon, n, N)
        var newCord Vec2i
        bFound := false
        minScore := uint32(math.MaxUint32)
        for i := uint32(0); i < g.directions; i++ {
            newCoordinates := current.Add(g.direction[i])
            if _, ok := openSet[newCoordinates.Hash()]; ok {
                continue
            }
            if g.detectCollision(newCoordinates) {
                continue
            }
            score := uint32(float64(weight) * math.Floor(float64(g.heuristic(newCoordinates, target_))))
            if score < minScore {
                minScore = score
                newCord = newCoordinates
                bFound = true
            }
        }
        //找到新节点入栈
        if bFound {
            openStack.Push(newCord)
        } else {
            //未找到新节点当前节点出栈
            openStack.Pop()
        }
    }
    if reachTarget {
        return openStack.v
    }
    return nil
}
