package astar

import (
    "container/heap"
    "math"
)

func (g *Generator) FindPathHeap(source_, target_ Vec2i) CoordinateList {
    if g.detectCollision(source_) || g.detectCollision(target_) {
        return nil
    }
    alloc := NewNodeAllocator(256)
    delta := getDelta(source_, target_)
    dist := uint(math.Max(float64(delta.X), float64(delta.Y))) + 1
    var current *Node
    openHeap := NewNodeHeap(dist * 4)
    openSet := make(CordSet, dist*4)
    closedMap := make(CordMap, dist*4)
    heap.Push(openHeap, alloc.NewNode(target_, nil))
    openSet[target_.Hash()] = struct{}{}
    n := uint32(0)
    N := uint32(g.worldSize.X * g.worldSize.Y)
    reachTarget := false
    for ; openHeap.Len() > 0; {
        current = openHeap.Top()
        if current.coordinates.Equals(source_) {
            reachTarget = true
            break
        }
        cord := current.coordinates
        closedMap[cord.Hash()] = current
        heap.Pop(openHeap)
        delete(openSet, cord.Hash())
        n++
        weight := g.relaxer(g.epsilon, n, N)
        for i := uint32(0); i < g.directions; i++ {
            newCoordinates := current.coordinates.Add(g.direction[i])
            if _, ok := openSet[newCoordinates.Hash()]; ok {
                continue
            }
            var successor *Node
            if g.detectCollision(newCoordinates) {
                continue
            }
            successor = g.findNodeOnMap(closedMap, newCoordinates)
            totalCost := uint32(0)
            if i < 4 {
                totalCost = current.G + 10
            } else {
                totalCost = current.G + 14
            }
            if successor == nil {
                newNode := alloc.NewNode(newCoordinates, current)
                newNode.G = totalCost
                newNode.H = uint32(float64(weight) * math.Floor(float64(g.heuristic(newNode.coordinates, source_))))
                newNode.updateScore()
                heap.Push(openHeap, newNode)
                openSet[newCoordinates.Hash()] = struct{}{}
            } else if totalCost < successor.G {
                successor.parent = current
                successor.G = totalCost
                successor.updateScore()
            }
        }
    }
    var path CoordinateList
    for current != nil && reachTarget {
        path = append(path, current.coordinates)
        current = current.parent
    }
    return path
}