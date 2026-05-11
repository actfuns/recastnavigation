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
| `detour_tile_cache` | ✅ 基本完成 | 分块流式加载 |
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

### 路径寻路

```go
package main

import (
    "fmt"
    "github.com/actfuns/recastnavigation/detour"
)

func main() {
    // 1. 从序列化数据加载导航网格
    navMesh := &detour.NavMesh{}
    // 假设 navData 是 []byte 格式的导航网格数据
    err := navMesh.InitSingleTile(navData, 0)
    if err != nil {
        panic("加载导航网格失败")
    }

    // 2. 创建查询对象
    query, err := detour.NewNavMeshQuery(navMesh, 2048)
    if err != nil {
        panic("创建查询对象失败")
    }

    // 3. 查询起始和结束多边形
    filter := &detour.QueryFilter{}
    startPos := [3]float32{10, 0, 10}
    endPos := [3]float32{20, 0, 20}
    halfExtents := [3]float32{2, 4, 2}

    startRef, _, _ := query.FindNearestPoly(startPos, halfExtents, filter)
    endRef, _, _ := query.FindNearestPoly(endPos, halfExtents, filter)

    // 4. 查找路径
    path, _, err := query.FindPath(startRef, endRef, startPos, endPos, filter)
    if err == nil {
        fmt.Printf("找到路径，长度: %d\n", len(path))
    }
}
```

### 导航网格生成（Recast）

```go
import (
    "github.com/actfuns/recastnavigation/recast"
)

func buildNavMesh(vertices []float32, triangles []int) {
    ctx := &recast.Context{}
    bmin, bmax := recast.CalcBounds(vertices, len(vertices)/3)
    cfg := &recast.Config{
        Cs:   0.3,  // 单元格大小
        Ch:   0.2,  // 单元格高度
        Bmin: bmin,
        Bmax: bmax,
        // ... 其他配置参数
    }

    // 1. 创建高度场
    w, h := recast.CalcGridSize(bmin, bmax, cfg.Cs)
    hf := recast.CreateHeightfield(ctx, w, h, bmin, bmax, cfg.Cs, cfg.Ch)

    // 2. 光栅化三角形
    recast.RasterizeTriangles(ctx, vertices, len(vertices)/3, triangles, nil, len(triangles)/3, hf, 1)

    // 3. 构建紧凑高度场
    chf, err := recast.BuildCompactHeightfield(ctx, cfg.WalkableHeight, cfg.WalkableClimb, hf)
    if err != nil {
        panic("构建紧凑高度场失败")
    }

    // ... 后续流程：过滤、分区、生成多边形等
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
