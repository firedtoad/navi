package astar

func StaticWeight(e float32, n, len uint32) float32 {
    return e
}

func DynamicWeight(e float32, n, len uint32) float32 {
    var w float32
    if n <= len {
        w = float32(1.0-n) / float32(len+.0)
    }
    return 1.0 + e*w
}
