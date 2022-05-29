package astar

type Vec2i struct {
    X, Y int32
}

func (v Vec2i) Equals(o Vec2i) bool {
    return v.X == o.X && v.Y == o.Y
}

func (v Vec2i) Hash() uint32 {
    return uint32(v.X<<16) + uint32(v.Y)
}

func (v Vec2i) Add(o Vec2i) Vec2i {
    return Vec2i{
        X: v.X + o.X,
        Y: v.Y + o.Y,
    }
}

type Node struct {
    G           uint32
    H           uint32
    S           uint32
    coordinates Vec2i
    parent      *Node
}

func (n *Node) getScore() uint32 {
    return n.S
}

func (n *Node) updateScore() {
    n.S = n.G + n.H
}

func (n Node) Clone() *Node {
    return &n
}

func NewNode(coordinates Vec2i, parent *Node) *Node {
    return &Node{
        coordinates: coordinates,
        parent:      parent,
    }
}

type NodeAllocator struct {
    nodes [][]Node
    cur   []Node
    id    int
    block int
}

func NewNodeAllocator(block int) *NodeAllocator {
    return &NodeAllocator{
        block: block,
    }
}

func (n *NodeAllocator) NewNode(coordinates Vec2i, parent *Node) *Node {
    if len(n.cur) > 0 && n.id < n.block {
        node := &n.cur[n.id]
        node.coordinates = coordinates
        node.parent = parent
        n.id++
        return node
    }

    n.cur = make([]Node, n.block)
    n.nodes = append(n.nodes, n.cur)

    n.id = 0
    node := &n.cur[n.id]
    node.coordinates = coordinates
    node.parent = parent
    n.id++
    return node
}


type NodeHeap struct {
    n []*Node
}

func NewNodeHeap(initCap uint) *NodeHeap {
    return &NodeHeap{n: make([]*Node, 0, initCap)}
}

func (h *NodeHeap) Len() int {
    return len(h.n)
}

func (h *NodeHeap) Less(i, j int) bool {
    return h.n[i].S < h.n[j].S
}

func (h *NodeHeap) Swap(i, j int) {
    h.n[i], h.n[j] = h.n[j], h.n[i]
}

func (h *NodeHeap) Push(x interface{}) {
    h.n = append(h.n, x.(*Node))
}

func (h *NodeHeap) Pop() interface{} {
    l := len(h.n)
    x := h.n[l-1]
    h.n = h.n[:l-1]
    return x
}

func (h *NodeHeap) Top() *Node {
    return h.n[0]
}

type CordMap map[uint32]*Node
type CordSet map[uint32]struct{}
type HeuristicFunction func(x Vec2i, y Vec2i) uint32
type RelaxFunction func(e float32, n, len uint32) float32
type CollisionFunction func(x Vec2i) bool
type CoordinateList []Vec2i

type Generator struct {
    heuristic  HeuristicFunction
    collision  CollisionFunction
    relaxer    RelaxFunction
    direction  CoordinateList
    worldSize  Vec2i
    epsilon    float32
    directions uint32
}

func NewGenerator() *Generator {
    g := &Generator{
    }
    g.SetHeuristic(Octagonal)
    g.SetDiagonalMovement(true)
    g.SetRelaxFunction(StaticWeight)
    g.SetWeight(2.0)
    g.direction = []Vec2i{
        {0, 1},
        {1, 0},
        {0, -1},
        {-1, 0},
        {-1, -1},
        {1, 1},
        {-1, 1},
        {1, -1},
    }
    return g
}

func (g *Generator) detectCollision(cord Vec2i) bool {
    x := cord.X
    y := cord.Y
    if x < 0 || x >= g.worldSize.X || y < 0 || y >= g.worldSize.Y || (g.collision != nil && g.collision(cord)) {
        return true
    }
    return false
}

func (g *Generator) findNodeOnMap(nodes_ CordMap, coordinates_ Vec2i) *Node {
    if n, ok := nodes_[coordinates_.Hash()]; ok {
        return n
    }
    return nil
}

func (g *Generator) SetWorldSize(w, h int32) {
    g.worldSize.X = w
    g.worldSize.Y = h
}
func (g *Generator) SetDiagonalMovement(enable_ bool) {
    if enable_ {
        g.directions = 8
        return
    }
    g.directions = 4
}

func (g *Generator) SetWeight(epsilon_ float32) {
    g.epsilon = epsilon_
}

func (g *Generator) SetRelaxFunction(relaxer_ RelaxFunction) {
    g.relaxer = relaxer_
}

func (g *Generator) SetHeuristic(heuristic_ HeuristicFunction) {
    g.heuristic = heuristic_
}

func (g *Generator) SetCollision(collision_ CollisionFunction) {
    g.collision = collision_
}


