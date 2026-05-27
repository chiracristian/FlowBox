def generate_funnel_grid(filename, width, height):
    # Initialize with air (0)
    grid = [[0 for _ in range(width)] for _ in range(height)]

    # 1. BORDER WALLS
    for y in range(height):
        for x in range(width):
            if x < 1 or x >= width - 1 or y < 1 or y >= height - 1:
                grid[y][x] = 1

    # 2. FUNNEL WALLS (Same logic as your C code)
    mid_x = width // 2
    for y in range(35, 50):
        wall_gap = 4 + ((50 - 35 - (y - 35)) // 2)
        grid[y][mid_x - wall_gap] = 1
        grid[y][mid_x - wall_gap - 1] = 1
        grid[y][mid_x + wall_gap] = 1
        grid[y][mid_x + wall_gap + 1] = 1
        
        # 3. FILL FUNNEL WITH PARTICLES (2)
        # Fill the space between the left and right wall
        for x in range(mid_x - wall_gap + 1, mid_x + wall_gap):
            grid[y][x] = 2

    # 4. REMAINING STRUCTURES (Pegs, Basin, Separator)
    # Pegs
    for row_y in [75, 95]:
        for x in range(12, width - 2, 12):
            for py in range(2):
                for px in range(2):
                    grid[row_y + py][x + px] = 1
    # Basin
    for x in range(17, 63):
        grid[135][x] = 1
        grid[136][x] = 1
    for h in range(15):
        grid[135 - h][17] = 1
        grid[135 - h][16] = 1
        grid[135 - h][62] = 1
        grid[135 - h][63] = 1
    # Separator
    for y in range(160, 195):
        grid[y][40] = 1
        grid[y][41] = 1

    # Write to file
    with open(filename, 'w') as f:
        f.write("FLOWBOX_GRID\n")
        for y in range(height):
            f.write(" ".join(str(grid[y][x]) for x in range(width)) + "\n")
    print(f"Generated {filename} successfully.")

if __name__ == "__main__":
    generate_funnel_grid("grid_1.txt", 80, 205)