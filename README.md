# Recast Navigation (Go 翻译版)

将 [recastnavigation/recastnavigation](https://github.com/recastnavigation/recastnavigation) 从 C++ 翻译为 Go 语言的实现。

> 原项目：一个用于游戏和实时应用的**导航网格生成**与**路径寻路**库，行业标准级方案，被 Unity、Unreal、Godot 等引擎采用。

## 包结构

| Go 包 | 对应 C++ 目录 | 功能 |
|-------|---------------|------|
| `recast` | `Recast/` | 导航网格生成（从原始几何体构建导航网格） |
| `detour` | `Detour/` | 运行时路径寻路与导航网格查询（A*、光线投射、多边形查询） |
| `detour_crowd` | `DetourCrowd/` | 智能体群体移动、碰撞避免与人群仿真 |
| `detour_tile_cache` | `DetourTileCache/` | 导航网格分块流式加载（适用于大世界） |
| `debug_utils` | `DebugUtils/` | 调试可视化绘制接口 |

## 原项目同步

- 原仓库：https://github.com/recastnavigation/recastnavigation
- 当前同步基线：commit `9f4ce64` (`Fix crash on large-scale navmesh generation (#796)`)
- 分支：`main`

### 原项目版本跟踪

| 原项目 commit | 翻译同步状态 | 说明 |
|--------------|------------|------|
| `9f4ce64` (main HEAD) | ✅ 已同步 | 当前对应最新代码 |
| v1.6.0 | ✅ 已翻译 | 基础版本 |

## 当前状态

| 包 | 状态 | 说明 |
|----|------|------|
| `recast` | ✅ 完成 | 导航网格生成核心功能，已移植 C++ 单元测试 |
| `detour` | ✅ 完成 | 路径寻路与查询，含序列化，算法逻辑与 C++ 完全一致 |
| `detour_crowd` | ✅ 完成 | 群体移动与碰撞避免，已移植 C++ 单元测试 |
| `detour_tile_cache` | ✅ 完成 | 分块流式加载与动态障碍，含 Go/C++ 对比测试 |
| `debug_utils` | ✅ 基本完成 | 调试绘制 |

## 性能基准

### 寻路性能（i5-13400, 20×20 网格, 800 个多边形）

| 测试 | 耗时 (ns/op) | 分配 (B/op) | 分配次数 |
|------|-------------:|-----------:|--------:|
| **FindPath** (5 格) | 7,771 | 16,384 | 1 |
| **FindPath** (10 格) | 8,992 | 16,384 | 1 |
| **FindPath** (19 格) | 10,426 | 16,384 | 1 |
| **FindStraightPath** (39 多边形路径) | 1,597 | 4,448 | 5 |
| **Raycast** (5 格) | 509 | 0 | 0 |
| **Raycast** (10 格) | 1,136 | 0 | 0 |
| **Raycast** (19 格) | 2,132 | 0 | 0 |
| **FindPolysAroundCircle** (r=50) | 16,429 | 0 | 0 |
| **FindPolysAroundCircle** (r=100) | 42,897 | 0 | 0 |
| **FindPolysAroundCircle** (r=200) | 46,829 | 0 | 0 |

### 导航网格构建

| 网格规模 | 多边形数 | 耗时 (ns/op) | 分配 (B/op) |
|---------|--------:|------------:|-----------:|
| 10×10 | 200 | 17,807 | 48,008 |
| 50×50 | 5,000 | 452,598 | 1,163,529 |
| 100×100 | 20,000 | 1,969,724 | 4,571,403 |

运行基准测试：

```bash
go test -bench=. -benchtime=500ms -benchmem ./detour/
```

## 使用示例

### 安装

```bash
go get github.com/actfuns/recastnavigation
```

### 路径寻路（Detour）

```go
package main

import (
	"fmt"
	"math"

	"github.com/actfuns/recastnavigation/detour"
)

func main() {
	// 1. 从序列化数据加载导航网格
	navMesh := &detour.NavMesh{}
	// 假设 navData 是 []byte 格式的导航网格数据
	navData := loadNavMeshData() // 由 Recast 构建或从文件读取
	if err := navMesh.Init(navData, 0); err != nil {
		panic("加载导航网格失败")
	}

	// 2. 创建查询对象
	query := detour.NewNavMeshQuery()
	if err := query.Init(navMesh, 2048); err != nil {
		panic("创建查询对象失败")
	}

	// 3. 设置查询过滤器（可包含/排除特定区域和标志）
	filter := &detour.QueryFilter{}
	filter.IncludeFlags = 0xffff    // 接受所有多边形标志
	filter.ExcludeFlags = 0
	for i := range filter.AreaCost {
		filter.AreaCost[i] = 1.0    // 所有区域成本相同
	}

	startPos := [3]float32{10, 0, 10}
	endPos   := [3]float32{20, 0, 20}
	halfExtents := [3]float32{2, 4, 2}

	// 4. 查询起始和结束多边形
	startRef, startPt, err := query.FindNearestPoly(startPos, halfExtents, filter)
	if err != nil || startRef == 0 {
		panic("找不到起始多边形")
	}
	endRef, _, err := query.FindNearestPoly(endPos, halfExtents, filter)
	if err != nil || endRef == 0 {
		panic("找不到结束多边形")
	}

	// 5. A* 寻路
	path, npath, err := query.FindPath(startRef, endRef, startPt, endPos, filter, 4096)
	if err != nil || npath == 0 {
		panic("找不到路径")
	}
	fmt.Printf("找到路径，%d 个多边形\n", npath)

	// 6. 拉直路径（生成可用的路点）
	straightPath, _, _, nstraight, err := query.FindStraightPath(
		startPt, endPos, path[:npath], npath, 4096, 0)
	if err != nil {
		panic("拉直路径失败")
	}
	fmt.Printf("路点数: %d\n", nstraight)
	for i := 0; i < nstraight; i++ {
		fmt.Printf("  %d: (%.2f, %.2f, %.2f)\n",
			i, straightPath[i*3], straightPath[i*3+1], straightPath[i*3+2])
	}
}

func loadNavMeshData() []byte { return nil }
```

### TileCache — 动态障碍（DetourTileCache）

```go
package main

import (
	"fmt"

	"github.com/actfuns/recastnavigation/detour"
	"github.com/actfuns/recastnavigation/detour_tile_cache"
)

func main() {
	// 1. 初始化 TileCache
	params := &detour_tile_cache.TileCacheParams{
		Orig:                  [3]float32{0, 0, 0},
		Cs:                    0.3,                  // 单元格大小
		Ch:                    0.2,                  // 单元格高度
		Width:                 20,                   // 宽度（单元格数）
		Height:                20,                   // 高度（单元格数）
		WalkableHeight:        2.0,
		WalkableRadius:        0.6,
		WalkableClimb:         1.0,
		MaxSimplificationError: 1.3,
		MaxTiles:              32,
		MaxObstacles:          128,
	}

	tc := detour_tile_cache.NewTileCache()
	alloc := &detour_tile_cache.LinearAllocator{}    // 或自定义分配器
	comp := &detour_tile_cache.FastLZCompressor{}    // 或自定义压缩器
	proc := &detour_tile_cache.MeshProcessFunc{      // 网格处理（设置多边形标志）
		ProcessFunc: func(params *detour_tile_cache.NavMeshCreateParams, polyAreas []byte, polyFlags []uint16) {
			for i, area := range polyAreas {
				if area != 0 {
					polyFlags[i] = 1
				}
			}
		},
	}

	if err := tc.Init(params, alloc, comp, proc); err != nil {
		panic("TileCache.Init 失败")
	}

	// 2. 添加压缩瓦片数据（由 Recast 生成或从文件加载）
	tileData := loadCompressedTileData() // 使用 dtBuildTileCacheLayer 构建
	tileRef, err := tc.AddTile(tileData, len(tileData), 0)
	if err != nil {
		panic("AddTile 失败")
	}

	// 3. 初始化 dtNavMesh
	navMesh := &detour.NavMesh{}
	if err := navMesh.Init(&detour.NavMeshParams{
		Orig:       [3]float32{0, 0, 0},
		TileWidth:  20 * 0.3,
		TileHeight: 20 * 0.3,
		MaxTiles:   1,
		MaxPolys:   4096,
	}); err != nil {
		panic("NavMesh.Init 失败")
	}

	// 4. 构建导航网格瓦片
	if err := tc.BuildNavMeshTile(tileRef, navMesh); err != nil {
		panic("BuildNavMeshTile 失败")
	}

	// 5. 添加动态障碍（圆柱体）
	obRef, err := tc.AddObstacle([3]float32{3, 0, 3}, 1.5, 4.0)
	if err != nil {
		panic("AddObstacle 失败")
	}
	fmt.Printf("添加障碍 ref=%d\n", obRef)

	// 6. 更新 TileCache（重建受影响的瓦片）
	for {
		done, err := tc.Update(1.0, navMesh)
		if err != nil {
			panic("Update 失败")
		}
		if done {
			break
		}
	}

	// 7. 移除障碍
	if err := tc.RemoveObstacle(obRef); err != nil {
		panic("RemoveObstacle 失败")
	}

	// 再次 Update 使移除生效
	for {
		done, err := tc.Update(1.0, navMesh)
		if err != nil {
			panic("Update 失败")
		}
		if done {
			break
		}
	}

	fmt.Println("动态障碍处理完成")
}

func loadCompressedTileData() []byte { return nil }
```

### 导航网格生成（Recast）

```go
package main

import (
	"fmt"

	"github.com/actfuns/recastnavigation/recast"
)

func main() {
	// 输入几何体：三角形网格
	vertices := []float32{
		0, 0, 0,  10, 0, 0,  10, 0, 10,  0, 0, 10, // 地面
	}
	triangles := []int{
		0, 1, 2,  0, 2, 3, // 两个三角形组成地面
	}
	nverts := len(vertices) / 3
	ntris := len(triangles) / 3

	ctx := &recast.Context{}

	// 1. 计算包围盒
	bmin, bmax := recast.CalcBounds(vertices, nverts)

	// 2. 设置构建参数
	cfg := &recast.Config{
		Cs:             0.3,               // 体素水平尺寸
		Ch:             0.2,               // 体素垂直尺寸
		WalkableSlopeAngle: 45,            // 可行走最大坡度
		WalkableHeight:     2,             // 可行走最小高度（体素数）
		WalkableClimb:      1,             // 可行走最大步高（体素数）
		WalkableRadius:     1,             // 可行走半径（体素数）
		MaxEdgeLen:         12,             // 最大边长（体素数）
		MaxSimplificationError: 1.3,       // 轮廓简化误差
		MinRegionArea:      8,             // 最小区域面积（体素数）
		MergeRegionArea:    20,            // 区域合并阈值
		MaxVertsPerPoly:    6,             // 多边形最大顶点数
		TileSize:           0,             // 0 = 单块
		BorderSize:         0,
		Bmin:               bmin,
		Bmax:               bmax,
	}

	// 3. 计算栅格大小
	w, h := recast.CalcGridSize(bmin, bmax, cfg.Cs)

	// 4. 构建高度场
	hf := recast.CreateHeightfield(ctx, w, h, bmin, bmax, cfg.Cs, cfg.Ch)

	// 5. 光栅化三角形
	rcTriareas := make([]uint8, ntris)
	recast.MarkWalkableTriangles(ctx, cfg.WalkableSlopeAngle, vertices, nverts,
		triangles, ntris, rcTriareas)
	recast.RasterizeTriangles(ctx, vertices, nverts, triangles, rcTriareas, ntris, hf, cfg.WalkableClimb)

	// 6. 过滤不可走区域
	recast.FilterLowHangingWalkableObstacles(ctx, cfg.WalkableClimb, hf)
	recast.FilterLedgeSpans(ctx, cfg.WalkableHeight, cfg.WalkableClimb, hf)
	recast.FilterWalkableLowHeightSpans(ctx, cfg.WalkableHeight, hf)

	// 7. 构建紧凑高度场
	chf, err := recast.BuildCompactHeightfield(ctx, cfg.WalkableHeight, cfg.WalkableClimb, hf)
	if err != nil {
		panic("BuildCompactHeightfield 失败")
	}

	// 8. 划分区域
	if err := recast.BuildDistanceField(ctx, chf); err != nil {
		panic("BuildDistanceField 失败")
	}
	if err := recast.BuildRegions(ctx, chf, 0, cfg.MinRegionArea, cfg.MergeRegionArea); err != nil {
		panic("BuildRegions 失败")
	}

	// 9. 构建轮廓
	contSet, err := recast.BuildContours(ctx, chf, cfg.MaxSimplificationError, cfg.MaxEdgeLen, recast.RC_CONTOUR_TESS_WALL_EDGES)
	if err != nil {
		panic("BuildContours 失败")
	}

	// 10. 构建多边形网格
	polyMesh, err := recast.BuildPolyMesh(ctx, contSet, cfg.MaxVertsPerPoly)
	if err != nil {
		panic("BuildPolyMesh 失败")
	}

	// 11. 构建细节网格
	if err := recast.BuildPolyMeshDetail(ctx, polyMesh, chf, cfg.DetailSampleDist, cfg.DetailSampleMaxError); err != nil {
		panic("BuildPolyMeshDetail 失败")
	}

	fmt.Printf("导航网格构建完成: %d 个顶点, %d 个多边形\n", polyMesh.NVerts, polyMesh.NPolys)
}
```

### 完整性检查

```bash
go vet ./...
go test ./...
```

## 与原 C++ 版本差异

- 使用 Go 的 `[3]float32`（定长数组）代替 C++ 的 `float*` 和 `float[3]`，避免切片索引越界
- 函数返回值风格：C++ 使用输出参数 + 返回状态码，Go 使用多返回值（`result, err`）
- 使用 `encoding/binary` + 手动序列化代替 C++ 的 `memcpy` 结构体序列化，确保二进制兼容
- 使用 Go 的垃圾回收代替 C++ 的手动内存池管理
- 去除了 C++ 的虚函数回调机制，改用 Go 接口
- 使用 Go 泛型实现 `Min`/`Max`/`Abs`/`Swap` 等工具函数
- 算法逻辑与 C++ 完全一致（所有核心函数已逐行对比确认）

## License

同原项目 — Zlib License
