def generate_grid_file(filename, width, height, triangle_base):
    # Initialize grid with air (0)
    grid = [[0 for _ in range(width)] for _ in range(height)]

    # 1. Add wall borders (1)
    for y in range(height):
        for x in range(width):
            if x == 0 or x == width - 1 or y == 0 or y == height - 1:
                grid[y][x] = 1

    # 2. Add triangular pile of particles (2)
    # Apex at (width/2, 100), base length = triangle_base
    apex_x = width // 2
    apex_y = 100
    
    # Simple algorithm to fill triangle downward from apex
    for y_offset in range(triangle_base // 2):
        y = apex_y + y_offset
        # Calculate width of this row
        row_w = (y_offset * 2) + 1
        start_x = apex_x - y_offset
        for x in range(start_x, start_x + row_w):
            if 0 < x < width - 1 and 0 < y < height - 1:
                grid[y][x] = 2

    # 3. Write to file
    with open(filename, 'w') as f:
        f.write("FLOWBOX_GRID\n")
        for y in range(height):
            line = " ".join(str(grid[y][x]) for x in range(width))
            f.write(line + "\n")
    
    print(f"Generated {filename} successfully.")

if __name__ == "__main__":
    generate_grid_file("grid_0.txt", 80, 205, 50)
