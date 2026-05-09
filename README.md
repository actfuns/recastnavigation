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
| `recast` | ✅ 基本完成 | 导航网格生成核心功能 |
| `detour` | ✅ 基本完成 | 路径寻路与查询，含序列化 |
| `detour_crowd` | ✅ 基本完成 | 群体移动与碰撞避免 |
| `detour_tile_cache` | ✅ 基本完成 | 分块流式加载 |
| `debug_utils` | ✅ 基本完成 | 调试绘制 |

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
    status := navMesh.InitSingleTile(navData, 0)
    if detour.StatusFailed(status) {
        panic("加载导航网格失败")
    }

    // 2. 创建查询对象
    query := detour.NewNavMeshQuery()
    query.Init(navMesh, 2048)

    // 3. 查询起始和结束多边形
    filter := detour.NewQueryFilter()
    startPos := []float32{10, 0, 10}
    endPos := []float32{20, 0, 20}
    halfExtents := []float32{2, 4, 2}

    startRef := query.FindNearestPoly(startPos, halfExtents, filter)
    endRef := query.FindNearestPoly(endPos, halfExtents, filter)

    // 4. 查找路径
    path := make([]detour.PolyRef, 512)
    pathLen, status := query.FindPath(startRef, endRef, startPos, endPos, filter, path, 512)
    if detour.StatusSucceed(status) {
        fmt.Printf("找到路径，长度: %d\n", pathLen)
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
    cfg := &recast.Config{
        // ... 配置参数
    }

    // 1. 创建高度场
    hf := recast.NewHeightfield(ctx, cfg)

    // 2. 光栅化三角形
    recast.RasterizeTriangles(ctx, vertices, triangles, hf, cfg)

    // 3. 构建紧凑高度场
    chf := recast.BuildCompactHeightfield(ctx, cfg, hf)

    // ... 后续流程：过滤、分区、生成多边形等
}
```

### 完整性检查

```bash
go vet ./...
go test ./...
```

## 与原 C++ 版本差异

- 使用 Go 的 `[]float32` 和 `[3]float32` 代替 C++ 的 `float*` 和 `float[3]`
- 使用 Go 的错误返回值（`Status` 类型）代替 C++ 的返回码
- 使用 `encoding/binary` + 手动序列化代替 C++ 的 `memcpy` 结构体序列化
- 使用 Go 的垃圾回收代替 C++ 的手动内存池管理
- 去除了 C++ 的虚函数回调机制，改用 Go 的函数/接口

## License

同原项目 — Zlib License
