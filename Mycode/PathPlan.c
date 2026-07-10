#include "stm32f4xx.h"                  // Device header
#include <string.h>
#include <stdlib.h>
#include "PathPlan.h"
#include "My_Comm.h"

// 方向数组：右、左、下、上（适配横向A1-A9、纵向B1-B7的移动逻辑）
const int8_t directions[4][2] = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};

// 全局变量 - 存储最终巡查路径
Point full_path[MAX_PATH_LENGTH];  // 路径点数组（按巡查顺序存储）
uint16_t full_path_length = 0;     // 路径总长度

// 内部变量
static uint8_t grid[ROWS][COLS];               // 网格状态：1=可访问，0=禁飞区
static Point barriers[BARRIER_COUNT];           // 禁飞区坐标（3个连续方格）
static Point accessible_cells[MAX_CELLS];       // 所有可访问方格
static uint16_t accessible_count = 0;

// 检查坐标是否在有效网格内（B1-B7，A1-A9）
static uint8_t is_valid(int8_t row, int8_t col) 
{
    return (row >= 0 && row < ROWS && col >= 0 && col < COLS);
}

// 计算曼哈顿距离（用于路径规划的启发式判断）
static uint16_t manhattan_distance(Point a, Point b) 
{
    return (uint16_t)(abs(a.row - b.row) + abs(a.col - b.col));
}

// 初始化网格：默认所有方格可访问，禁飞区后续设置
void init_grid(void) 
{
    for (int8_t i = 0; i < ROWS; i++) {
        for (int8_t j = 0; j < COLS; j++) {
            grid[i][j] = 1;  // 1=可访问（非禁飞区）
        }
    }
    accessible_count = 0;
    full_path_length = 0;
}

// 检查网格连通性（确保起点A9B1可到达所有非禁飞区）
uint8_t check_connectivity(void) 
{
    uint8_t visited[ROWS][COLS] = {0};
    Point queue[MAX_CELLS];
    uint16_t front = 0, rear = 0;
    
    Point start = {0, 8};  // 起点：A9B1（row=0→B1，col=8→A9）
    if (grid[start.row][start.col] == 0) return 0;  // 若起点在禁飞区，无效
    
    queue[rear++] = start;
    visited[start.row][start.col] = 1;
    uint16_t reachable_count = 1;
    
    while (front < rear) 
    {
        Point current = queue[front++];
        
        for (int8_t i = 0; i < 4; i++) 
        {
            int8_t new_row = current.row + directions[i][0];
            int8_t new_col = current.col + directions[i][1];
            
            if (is_valid(new_row, new_col) && !visited[new_row][new_col] && grid[new_row][new_col] == 1) 
            {
                visited[new_row][new_col] = 1;
                queue[rear++] = (Point){new_row, new_col};
                reachable_count++;
            }
        }
    }
    
    // 计算非禁飞区总数，确保连通性覆盖80%以上
    uint16_t total_accessible = ROWS * COLS - BARRIER_COUNT;
    return reachable_count >= (total_accessible * 4) / 5;
}

// 设置禁飞区（示例：根据现场给定的禁飞区代码修改，此处为横向连续示例）
void set_barriers(void) 
{
    // 示例：禁飞区为A3B5-A3B7（横向连续，row=4→B5，col=2→A3）
    // 实际使用时需根据现场抽签的禁飞区代码修改
    barriers[0] = (Point){ground_data.NoFly_Zone1[3], ground_data.NoFly_Zone1[1]};  
    barriers[1] = (Point){ground_data.NoFly_Zone2[3], ground_data.NoFly_Zone2[1]};  
    barriers[2] = (Point){ground_data.NoFly_Zone3[3], ground_data.NoFly_Zone3[1]}; 
    
    // 标记禁飞区到网格（0=不可访问）
    for (int8_t i = 0; i < BARRIER_COUNT; i++) 
    {
        if (is_valid(barriers[i].row, barriers[i].col) && !(barriers[i].row == 0 && barriers[i].col == 8)) // 避免起点在禁飞区
        {  
            grid[barriers[i].row][barriers[i].col] = 0;
        }
    }
}

// 收集所有可访问的方格（非禁飞区）
static void collect_accessible_cells(void) 
{
    accessible_count = 0;
    for (int8_t i = 0; i < ROWS; i++) 
    {
        for (int8_t j = 0; j < COLS; j++) 
        {
            if (grid[i][j] == 1)
            {
                accessible_cells[accessible_count++] = (Point){i, j};
            }
        }
    }
}

// BFS寻找两点间最短路径（用于生成相邻方格的航线）
static uint16_t find_shortest_path(Point start, Point end, Point path[], uint16_t max_length) 
{
    if (start.row == end.row && start.col == end.col) 
    {
        path[0] = start;
        return 1;
    }
    
    uint8_t visited[ROWS][COLS] = {0};
    int8_t parent[ROWS][COLS][2];  // 存储父节点坐标
    Point queue[MAX_CELLS];
    uint16_t front = 0, rear = 0;
    
    queue[rear++] = start;
    visited[start.row][start.col] = 1;
    parent[start.row][start.col][0] = -1;
    parent[start.row][start.col][1] = -1;
    
    while (front < rear) 
    {
        Point current = queue[front++];
        
        if (current.row == end.row && current.col == end.col) 
        {
            // 重建路径并反转
            uint16_t path_length = 0;
            Point trace = end;
            
            while (trace.row != -1 && trace.col != -1) 
            {
                path[path_length++] = trace;
                if (parent[trace.row][trace.col][0] == -1) break;
                trace = (Point){parent[trace.row][trace.col][0], parent[trace.row][trace.col][1]};
            }
            
            for (uint16_t i = 0; i < path_length / 2; i++) 
            {
                Point temp = path[i];
                path[i] = path[path_length - 1 - i];
                path[path_length - 1 - i] = temp;
            }
            
            return path_length;
        }
        
        for (int8_t i = 0; i < 4; i++) 
        {
            int8_t new_row = current.row + directions[i][0];
            int8_t new_col = current.col + directions[i][1];
            
            if (is_valid(new_row, new_col) && !visited[new_row][new_col] && grid[new_row][new_col] == 1) 
            {
                visited[new_row][new_col] = 1;
                parent[new_row][new_col][0] = current.row;
                parent[new_row][new_col][1] = current.col;
                queue[rear++] = (Point){new_row, new_col};
            }
        }
    }
    
    return 0;  // 无法到达（禁飞区阻断）
}

// 最近邻TSP算法（规划覆盖所有可访问方格的航线）
static void nearest_neighbor_tsp(Point visit_order[])
{
    uint8_t visited[MAX_CELLS] = {0};
    Point current = {0, 8};  // 起点：A9B1
    uint16_t order_count = 0;
    
    // 找到起点在可访问方格中的索引
    int16_t start_index = -1;
    for (uint16_t i = 0; i < accessible_count; i++) 
    {
        if (accessible_cells[i].row == 0 && accessible_cells[i].col == 8) 
        {
            start_index = i;
            break;
        }
    }
    
    if (start_index != -1) 
    {
        visit_order[order_count++] = current;
        visited[start_index] = 1;
    }
    
    // 按最近邻原则规划访问顺序
    while (order_count < accessible_count) 
    {
        int16_t nearest_index = -1;
        uint16_t min_distance = 0xFFFF;
        
        for (uint16_t i = 0; i < accessible_count; i++) 
        {
            if (!visited[i]) 
            {
                uint16_t dist = manhattan_distance(current, accessible_cells[i]);
                if (dist < min_distance) 
                {
                    min_distance = dist;
                    nearest_index = i;
                }
            }
        }
        
        if (nearest_index != -1) 
        {
            visit_order[order_count++] = accessible_cells[nearest_index];
            visited[nearest_index] = 1;
            current = accessible_cells[nearest_index];
        } 
        else 
        {
            break;  // 所有方格已访问
        }
    }
}

// 求解最短巡查路径，存储到full_path数组
void solve_shortest_path(void) 
{
    collect_accessible_cells();
    if (accessible_count <= 1) return;  // 无有效路径
    
    // 生成访问顺序
    Point visit_order[MAX_CELLS];
    nearest_neighbor_tsp(visit_order);
    
    // 拼接完整路径（相邻方格的最短路径）
    full_path_length = 0;
    uint16_t order_count = accessible_count;
    
    for (uint16_t i = 0; i < order_count; i++) 
    {
        Point current = visit_order[i];
        Point next = visit_order[(i + 1) % order_count];  // 最后返回起点附近
        
        Point segment_path[MAX_PATH_LENGTH];
        uint16_t segment_length = find_shortest_path(current, next, segment_path, MAX_PATH_LENGTH);
        
        if (segment_length == 0) 
        {
            full_path_length = 0;  // 路径无效
            return;
        }
        
        // 拼接路径（避免重复节点）
        uint16_t start_idx = (i == 0) ? 0 : 1;
        for (uint16_t j = start_idx; j < segment_length && full_path_length < MAX_PATH_LENGTH; j++) 
        {
            full_path[full_path_length++] = segment_path[j];
        }
    }
}

