package detour

import (
	"unsafe"
)

// HashRef hashes a PolyRef for hash table lookups.
func HashRef(a PolyRef) uint32 {
	a += ^(a << 15)
	a ^= (a >> 10)
	a += (a << 3)
	a ^= (a >> 6)
	a += ^(a << 11)
	a ^= (a >> 16)
	return uint32(a)
}

// NodePool is a pool of pathfinding nodes.
type NodePool struct {
	nodes     []Node
	first     []NodeIndex
	next      []NodeIndex
	maxNodes  int
	hashSize  int
	nodeCount int
}

// NewNodePool creates a new NodePool.
func NewNodePool(maxNodes, hashSize int) (*NodePool, error) {
	if NextPow2(uint32(hashSize)) != uint32(hashSize) {
		return nil, ErrInvalidParam
	}
	if maxNodes <= 0 || maxNodes > int(NullIdx) || maxNodes > (1<<NodeParentBits)-1 {
		return nil, ErrInvalidParam
	}

	pool := &NodePool{
		nodes:    make([]Node, maxNodes),
		first:    make([]NodeIndex, hashSize),
		next:     make([]NodeIndex, maxNodes),
		maxNodes: maxNodes,
		hashSize: hashSize,
	}

	// Initialize with null indices
	for i := range pool.first {
		pool.first[i] = NullIdx
	}
	for i := range pool.next {
		pool.next[i] = NullIdx
	}

	return pool, nil
}

// Clear clears the node pool.
func (p *NodePool) Clear() {
	// Fill first[] with NullIdx using copy-doubling for performance.
	first := p.first
	if len(first) > 0 {
		first[0] = NullIdx
		for n := 1; n < len(first); n *= 2 {
			copy(first[n:], first[:n])
		}
	}
	p.nodeCount = 0
}

// GetNodeIdx returns the index of a node (1-based, 0 = null).
// Uses unsafe pointer arithmetic for efficiency, same as the C++ version.
func (p *NodePool) GetNodeIdx(node *Node) uint32 {
	if node == nil {
		return 0
	}
	baseAddr := uintptr(unsafe.Pointer(&p.nodes[0]))
	nodeAddr := uintptr(unsafe.Pointer(node))
	idx := (nodeAddr - baseAddr) / unsafe.Sizeof(Node{})
	if idx < uintptr(p.nodeCount) {
		return uint32(idx) + 1
	}
	return 0
}

// GetNodeAtIdx returns the node at the given 1-based index.
func (p *NodePool) GetNodeAtIdx(idx uint32) *Node {
	if idx == 0 {
		return nil
	}
	return &p.nodes[idx-1]
}

// GetMaxNodes returns the maximum number of nodes.
func (p *NodePool) GetMaxNodes() int {
	return p.maxNodes
}

// GetHashSize returns the hash size.
func (p *NodePool) GetHashSize() int {
	return p.hashSize
}

// GetFirst returns the first node index in the bucket.
func (p *NodePool) GetFirst(bucket int) NodeIndex {
	return p.first[bucket]
}

// GetNext returns the next node index in the chain.
func (p *NodePool) GetNext(i int) NodeIndex {
	return p.next[i]
}

// GetNodeCount returns the current number of nodes.
func (p *NodePool) GetNodeCount() int {
	return p.nodeCount
}

// FindNodes finds all nodes with the given id.
func (p *NodePool) FindNodes(id PolyRef, nodes []*Node, maxNodes int) int {
	n := 0
	bucket := HashRef(id) & uint32(p.hashSize-1)
	i := p.first[bucket]
	for i != NullIdx {
		if p.nodes[i].ID == id {
			if n >= maxNodes {
				return n
			}
			nodes[n] = &p.nodes[i]
			n++
		}
		i = p.next[i]
	}
	return n
}

// FindNode finds a node by id and state.
func (p *NodePool) FindNode(id PolyRef, state uint8) *Node {
	bucket := HashRef(id) & uint32(p.hashSize-1)
	i := p.first[bucket]
	for i != NullIdx {
		if p.nodes[i].ID == id && p.nodes[i].State == state {
			return &p.nodes[i]
		}
		i = p.next[i]
	}
	return nil
}

// GetNode gets a node by id and state, creating one if it doesn't exist.
func (p *NodePool) GetNode(id PolyRef, state uint8) *Node {
	bucket := HashRef(id) & uint32(p.hashSize-1)
	i := p.first[bucket]
	for i != NullIdx {
		if p.nodes[i].ID == id && p.nodes[i].State == state {
			return &p.nodes[i]
		}
		i = p.next[i]
	}

	if p.nodeCount >= p.maxNodes {
		return nil
	}

	i = NodeIndex(p.nodeCount)
	p.nodeCount++

	// Init node
	node := &p.nodes[i]
	node.Pidx = 0
	node.Cost = 0
	node.Total = 0
	node.ID = id
	node.State = state
	node.Flags = 0

	p.next[i] = p.first[bucket]
	p.first[bucket] = i

	return node
}

// NodeQueue is a priority queue (min-heap) for pathfinding nodes.
type NodeQueue struct {
	heap     []*Node
	capacity int
	size     int
}

// NewNodeQueue creates a new NodeQueue.
func NewNodeQueue(n int) *NodeQueue {
	return &NodeQueue{
		heap:     make([]*Node, n+1),
		capacity: n,
		size:     0,
	}
}

// Clear clears the queue.
func (q *NodeQueue) Clear() {
	q.size = 0
}

// Top returns the top node (minimum total cost).
func (q *NodeQueue) Top() *Node {
	if q.size > 0 {
		return q.heap[0]
	}
	return nil
}

// Pop removes and returns the top node.
func (q *NodeQueue) Pop() *Node {
	if q.size == 0 {
		return nil
	}
	result := q.heap[0]
	q.size--
	q.trickleDown(0, q.heap[q.size])
	return result
}

// Push adds a node to the queue.
func (q *NodeQueue) Push(node *Node) {
	q.size++
	q.bubbleUp(q.size-1, node)
}

// Modify updates a node's position in the queue.
func (q *NodeQueue) Modify(node *Node) {
	for i := 0; i < q.size; i++ {
		if q.heap[i] == node {
			q.bubbleUp(i, node)
			return
		}
	}
}

// Empty returns true if the queue is empty.
func (q *NodeQueue) Empty() bool {
	return q.size == 0
}

// GetCapacity returns the queue capacity.
func (q *NodeQueue) GetCapacity() int {
	return q.capacity
}

func (q *NodeQueue) bubbleUp(i int, node *Node) {
	h := q.heap
	parent := (i - 1) / 2
	for i > 0 && h[parent].Total > node.Total {
		h[i] = h[parent]
		i = parent
		parent = (i - 1) / 2
	}
	h[i] = node
}

func (q *NodeQueue) trickleDown(i int, node *Node) {
	h := q.heap
	child := (i * 2) + 1
	for child < q.size {
		if (child+1) < q.size && h[child].Total > h[child+1].Total {
			child++
		}
		h[i] = h[child]
		i = child
		child = (i * 2) + 1
	}
	q.bubbleUp(i, node)
}

// GetNodeIdxResult stores a node and its index.
type GetNodeIdxResult struct {
	Node *Node
	Idx  uint32
}
