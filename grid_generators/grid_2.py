import random

def generate_maze(filename, width, height):
    # Initialize with walls (1)
    grid = [[1 for _ in range(width)] for _ in range(height)]
    
    # Start at (2, 2)
    start_x, start_y = 2, 2
    grid[start_y][start_x] = 0
    
    # Manual stack for iterative traversal
    stack = [(start_x, start_y)]
    
    while stack:
        x, y = stack[-1]
        
        # Possible directions: Up, Down, Left, Right (2 steps away)
        directions = [(0, 2), (0, -2), (2, 0), (-2, 0)]
        random.shuffle(directions)
        
        found_neighbor = False
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            
            if 1 < nx < width - 1 and 1 < ny < height - 1 and grid[ny][nx] == 1:
                # Carve path to neighbor
                grid[y + dy//2][x + dx//2] = 0
                grid[ny][nx] = 0
                stack.append((nx, ny))
                found_neighbor = True
                break
        
        if not found_neighbor:
            stack.pop()

    # Clean up bottom basin for particles
    for x in range(17, 63):
        for y in range(130, 140):
            grid[y][x] = 0

    # Write file
    with open(filename, 'w') as f:
        f.write("FLOWBOX_GRID\n")
        for y in range(height):
            f.write(" ".join(str(grid[y][x]) for x in range(width)) + "\n")
    print(f"Generated {filename} successfully.")

if __name__ == "__main__":
    generate_maze("grid_2.txt", 80, 205)
